/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  support for serial connected AHRS systems
 */

#define ALLOW_DOUBLE_MATH_FUNCTIONS

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_SBG_ENABLED

#include "AP_ExternalAHRS_SBG.h"
#include <AP_Math/AP_Math.h>
#include <AP_Math/crc.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <stdio.h>
#include <AP_BoardConfig/AP_BoardConfig.h>

extern const AP_HAL::HAL &hal;

#define SBG_ECOM_SYNC_1 (0xFF)
#define SBG_ECOM_SYNC_2 (0x5A)
#define SBG_ECOM_ETX (0x33)
#define SBG_ECOM_MAX_PACKET_SIZE (4095)
#define SBG_ECOM_CLASS_LOG_ECOM_0 (0)

// expected at 200Hz
#define SBG_ECOM_LOG_EKF_EULER (6)
typedef struct PACKED {
    uint32_t timestamp_us;
    float roll_rad;
    float pitch_rad;
    float yaw_rad;
    float roll_acc_rad;
    float pitch_acc_rad;
    float yaw_acc_rad;
    uint32_t solution_status;
} SBGEKFEuler;

// expected at 200Hz
#define SBG_ECOM_LOG_EKF_NAV (8)
typedef struct PACKED {
    uint32_t timestamp_us;
    float vel_n_ms; // meters per second
    float vel_e_ms;
    float vel_d_ms;
    float vel_n_acc_ms; // meters per second
    float vel_e_acc_ms;
    float vel_d_acc_ms;
    double latitude_deg;
    double longitude_deg;
    double altitude_m;
    float undulation_m;
    float latitude_acc_m;
    float longitude_acc_m;
    float altitude_acc_m;
    uint32_t solution_status;
} SBGEKFNav;

// expected at 5Hz ("on new data")
#define SBG_ECOM_LOG_GPS1_POS (14)
typedef struct PACKED {
    uint32_t timestamp_us;
    uint32_t pos_status;
    uint32_t time_of_week_ms;
    double latitude_deg;
    double longitude_deg;
    double altitude_m;
    float undulation_m;
    float latitude_acc_m;
    float longitude_acc_m;
    float altitude_acc_m;
    uint8_t num_satellites;
    uint16_t base_station_id;
    uint16_t differential_age_cs; // centiseconds
} SBGGPS1Pos;

// expected at 5Hz ("on new data")
#define SBG_ECOM_LOG_GPS1_VEL (13)
typedef struct PACKED {
    uint32_t timestamp_us;
    uint32_t vel_status;
    uint32_t time_of_week_ms;
    float vel_n_ms; // meters per second
    float vel_e_ms;
    float vel_d_ms;
    float vel_n_acc_ms; // meters per second
    float vel_e_acc_ms;
    float vel_d_acc_ms;
    float course_deg;
    float course_acc_deg;
} SBGGPS1Vel;

// expected at 10Hz
#define SBG_ECOM_LOG_UTC_TIME (2)
typedef struct PACKED {
    uint32_t timestamp_us;
    uint16_t clock_status;
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint32_t nanosecond;
    uint32_t gps_tow_ms;
} SBGUTCTime;

// expected at 200Hz ("on new data")
#define SBG_ECOM_LOG_IMU_SHORT (44)
typedef struct PACKED {
    uint32_t timestamp_us;
    uint16_t imu_status;
    int32_t delta_vel_x; // 1<<20 == 1m/s^2
    int32_t delta_vel_y;
    int32_t delta_vel_z;
    int32_t delta_angle_x; // 1<<26 == 1rad/s
    int32_t delta_angle_y;
    int32_t delta_angle_z;
    int16_t temperature; // 1<<8 == 1degC
} SBGIMUShort;

// expected at 50Hz
#define SBG_ECOM_LOG_MAG (4)
typedef struct PACKED {
    uint32_t timestamp_us;
    uint16_t mag_status;
    float mag_x; // "arbitrary units", quite possibly gauss
    float mag_y;
    float mag_z;
    float accel_x_ms2; // m/s^2
    float accel_y_ms2;
    float accel_z_ms2;
} SBGMag;

// expected at 50Hz ("on new data")
#define SBG_ECOM_LOG_AIR_DATA (36)
typedef struct PACKED {
    uint32_t timestamp_us;
    uint16_t airdata_status;
    float pressure_abs_pa; // barometer
    float altitude_m;
    float pressure_diff_pa; // airspeed
    float true_airspeed_ms; // meters per second
    float air_temperature_degC;
} SBGAirData;

typedef struct {
    SBGEKFEuler ekf_euler;
    SBGEKFNav ekf_nav;
    SBGGPS1Pos gps1_pos;
    SBGGPS1Vel gps1_vel;
    SBGUTCTime utc_time;
    SBGIMUShort imu_short;
    SBGMag mag;
    SBGAirData air_data;
} SBGPacketSet;

// constructor
AP_ExternalAHRS_SBG::AP_ExternalAHRS_SBG(AP_ExternalAHRS *_frontend,
                                                     AP_ExternalAHRS::state_t &_state) :
    AP_ExternalAHRS_backend(_frontend, _state)
{
    auto &sm = AP::serialmanager();
    uart = sm.find_serial(AP_SerialManager::SerialProtocol_AHRS, 0);
    if (!uart) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SBG ExternalAHRS no UART");
        return;
    }
    baudrate = sm.find_baudrate(AP_SerialManager::SerialProtocol_AHRS, 0);
    port_num = sm.find_portnum(AP_SerialManager::SerialProtocol_AHRS, 0);

    data_buf = new uint8_t[SBG_ECOM_MAX_PACKET_SIZE];
    packet_buf = new SBGPacketSet;

    if (!data_buf || !packet_buf) {
        AP_BoardConfig::allocation_error("SBG ExternalAHRS");
    }

    // ensure timestamps are clear
    memset(packet_buf, 0, sizeof(SBGPacketSet));

    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_ExternalAHRS_SBG::update_thread, void), "AHRS", 2048, AP_HAL::Scheduler::PRIORITY_SPI, 0)) {
        AP_HAL::panic("SBG Failed to start ExternalAHRS update thread");
    }
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SBG ExternalAHRS initialised");
}

/*
  check the UART for more data
  returns true if the function should be called again straight away
 */
bool AP_ExternalAHRS_SBG::check_uart()
{
    if (!setup_complete) {
        return false;
    }
    WITH_SEMAPHORE(state.sem);
    // ensure we own the uart
    uart->begin(0);
    // receive as much data as will fit
    uint32_t n = uart->available();
    if (n == 0) {
        return false;
    }
    uint16_t buf_room = SBG_ECOM_MAX_PACKET_SIZE - buf_len;
    if (buf_room) {
        ssize_t bytes_read = uart->read(&data_buf[buf_len], MIN(n, buf_room));
        if (bytes_read <= 0) {
            return false;
        }
        buf_len += bytes_read;
    }

    bool valid;
    uint16_t consume = unframe_packet(&data_buf[0], buf_len, valid);

    if (valid) {
        uint8_t msg_id = data_buf[2];
        uint8_t msg_class = data_buf[3];
        uint8_t msg_len = consume - 9;
        if (msg_class == SBG_ECOM_CLASS_LOG_ECOM_0) { // only class we care about
            uint16_t expected_len = process_message(msg_id, &data_buf[6], msg_len);
            if (expected_len != msg_len) {
                printf("SBG: msg id %u expected %u got %u bytes\n",
                    msg_id, expected_len, msg_len);
            }
        }
    }

    if (consume) {
        uint16_t remaining = buf_len - consume;
        if (remaining) {
            memmove(&data_buf[0], &data_buf[consume], remaining);
        }
        buf_len = remaining;
    }

    // try to process again if we've consumed something and still have more
    return consume > 0 && buf_len > 0;
}

// valid is true if data points to a valid packet.
// returns amount of data to consume/packet length
uint16_t AP_ExternalAHRS_SBG::unframe_packet(const uint8_t *data, uint16_t len, bool &valid)
{
    valid = false;

    // sync 1 + sync 2 + id + class + length + data + crc + ETX
    //      1 +      1 +  1 +     1 +      2 +    0 +   2 +   1
    if (len < 9) { // bail if not enough data for a complete (empty) packet
        return 0;
    }

    // get message metadata
    uint16_t msg_len = data[4] | (data[5] << 8);
    uint16_t msg_crc = data[6+msg_len] | (data[7+msg_len] << 8);
    uint16_t packet_len = 9 + msg_len;

    // validate sync markers
    if ((data[0] != SBG_ECOM_SYNC_1) || (data[1] != SBG_ECOM_SYNC_2)) {
        goto search;
    }
    // validate message length (instead of packet length to avoid overflows)
    if (msg_len > SBG_ECOM_MAX_PACKET_SIZE-9) {
        goto search;
    }

    if (len < packet_len) { // bail if not enough data for complete packet
        return 0;
    }

    // validate footer and CRC
    if (data[8+msg_len] != SBG_ECOM_ETX) {
        goto search;
    }
    if (msg_crc != crc16_ccitt_r(&data[2], 4+msg_len, 0, 0)) {
        goto search;
    }

    valid = true; // message appears valid, awesome
    return packet_len; // consume it

search:
    // search for a sync marker and consume up to it
    uint8_t *p = (uint8_t *)memchr(&data[1], SBG_ECOM_SYNC_1, len-1);
    if (p) {
        return len - (p - data);
    } else {
        return len;
    }
}

void AP_ExternalAHRS_SBG::update_thread()
{
    // Open port in the thread
    uart->begin(baudrate, 1024, 512);

    setup_complete = true;
    while (true) {
        if (!check_uart()) {
            hal.scheduler->delay(1);
        }
    }
}

const char* AP_ExternalAHRS_SBG::get_name() const
{
    return "SBG";
}

uint16_t AP_ExternalAHRS_SBG::process_message(uint8_t id, const uint8_t *data, uint16_t len)
{
    SBGPacketSet *packet = (SBGPacketSet*)packet_buf;

    // all supported messages have a timestamp as the first 4 bytes
    uint32_t timestamp_us = 0;
    if (len >= 4) {
        timestamp_us = data[0] | (data[1]<<8) | (data[2]<<16) | (data[3]<<24);
    }

    // for each message we check if it's the expected size and if it's actually
    // different (as something in the chain duplicates bits of data...). then
    // we store the new data and process it if appropriate as some messages
    // come in groups and we want the whole group before we forward it to the
    // rest of the system
    switch (id) {
    case SBG_ECOM_LOG_EKF_EULER:
        if (len != sizeof(SBGEKFEuler)) { return sizeof(SBGEKFEuler); }
        if (timestamp_us == packet->ekf_euler.timestamp_us) { break; }
        memcpy(&packet->ekf_euler, data, sizeof(SBGEKFEuler));

        // we have both the euler and the nav data
        if (timestamp_us == packet->ekf_nav.timestamp_us) {
            update_state_ekf();
        }
        break;

    case SBG_ECOM_LOG_EKF_NAV:
        if (len != sizeof(SBGEKFNav)) { return sizeof(SBGEKFNav); }
        if (timestamp_us == packet->ekf_nav.timestamp_us) { break; }
        memcpy(&packet->ekf_nav, data, sizeof(SBGEKFNav));

        // we have both the euler and the nav data
        if (timestamp_us == packet->ekf_euler.timestamp_us) {
            update_state_ekf();
        }
        break;

    case SBG_ECOM_LOG_GPS1_POS:
        if (len != sizeof(SBGGPS1Pos)) { return sizeof(SBGGPS1Pos); }
        if (timestamp_us == packet->gps1_pos.timestamp_us) { break; }
        memcpy(&packet->gps1_pos, data, sizeof(SBGGPS1Pos));

        // we have position, velocity, and UTC (for time of week) data
        if ((timestamp_us == packet->gps1_vel.timestamp_us) &&
                (timestamp_us == packet->utc_time.timestamp_us)) {
            update_state_gps();
        }
        break;

    case SBG_ECOM_LOG_GPS1_VEL:
        if (len != sizeof(SBGGPS1Vel)) { return sizeof(SBGGPS1Vel); }
        if (timestamp_us == packet->gps1_vel.timestamp_us) { break; }
        memcpy(&packet->gps1_vel, data, sizeof(SBGGPS1Vel));

        // we have position, velocity, and UTC (for time of week) data
        if ((timestamp_us == packet->gps1_pos.timestamp_us) &&
                (timestamp_us == packet->utc_time.timestamp_us)) {
            update_state_gps();
        }
        break;

    case SBG_ECOM_LOG_UTC_TIME:
        if (len != sizeof(SBGUTCTime)) { return sizeof(SBGUTCTime); }
        if (timestamp_us == packet->utc_time.timestamp_us) { break; }
        memcpy(&packet->utc_time, data, sizeof(SBGUTCTime));

        // we have position, velocity, and UTC (for time of week) data
        if ((timestamp_us == packet->gps1_pos.timestamp_us) &&
                (timestamp_us == packet->gps1_vel.timestamp_us)) {
            update_state_gps();
        }
        break;

    case SBG_ECOM_LOG_IMU_SHORT:
        if (len != sizeof(SBGIMUShort)) { return sizeof(SBGIMUShort); }
        if (timestamp_us == packet->imu_short.timestamp_us) { break; }
        memcpy(&packet->imu_short, data, sizeof(SBGIMUShort));

        update_state_imu();
        break;

    case SBG_ECOM_LOG_MAG:
        if (len != sizeof(SBGMag)) { return sizeof(SBGMag); }
        if (timestamp_us == packet->mag.timestamp_us) { break; }
        memcpy(&packet->mag, data, sizeof(SBGMag));

        update_state_mag();
        break;

    case SBG_ECOM_LOG_AIR_DATA:
        if (len != sizeof(SBGAirData)) { return sizeof(SBGAirData); }
        if (timestamp_us == packet->air_data.timestamp_us) { break; }
        memcpy(&packet->air_data, data, sizeof(SBGAirData));

        update_state_baro();
        break;

    default:
        break; // just say we got what we expected
    }

    last_packet_ms = AP_HAL::millis(); // alive and receiving good data

    return len; // we got what we expected
}

void AP_ExternalAHRS_SBG::update_state_ekf(void)
{
    SBGPacketSet *packet = (SBGPacketSet*)packet_buf;
    const SBGEKFEuler &euler = packet->ekf_euler;
    const SBGEKFNav &nav = packet->ekf_nav;

    {
        WITH_SEMAPHORE(state.sem);

        state.quat.from_euler(euler.roll_rad, euler.pitch_rad, euler.yaw_rad);
        state.have_quaternion = true;

        state.velocity = Vector3f{nav.vel_n_ms, nav.vel_e_ms, nav.vel_d_ms};
        state.have_velocity = true;

        state.location = Location{int32_t(nav.latitude_deg * 1.0e7),
                                  int32_t(nav.longitude_deg * 1.0e7),
                                  int32_t(nav.altitude_m * 1.0e2),
                                  Location::AltFrame::ABSOLUTE};
        state.last_location_update_us = AP_HAL::micros();
        state.have_location = true;
    }
}

void AP_ExternalAHRS_SBG::update_state_imu(void)
{
    SBGPacketSet *packet = (SBGPacketSet*)packet_buf;
    const SBGIMUShort &imu = packet->imu_short;

    AP_ExternalAHRS::ins_data_message_t ins;

    float as = 1.0f/(1<<20); // convert to meters per second
    ins.accel = Vector3f{imu.delta_vel_x*as, imu.delta_vel_y*as, imu.delta_vel_z*as};
    float gs = 1.0f/(1<<26); // convert to radians per second
    ins.gyro = Vector3f{imu.delta_angle_x*gs, imu.delta_angle_y*gs, imu.delta_angle_z*gs};
    ins.temperature = imu.temperature*(1.0f/(1<<8)); // convert to degC

    {
        WITH_SEMAPHORE(state.sem);

        state.accel = ins.accel;
        state.gyro = ins.gyro;
    }

    AP::ins().handle_external(ins);
}

void AP_ExternalAHRS_SBG::update_state_mag(void)
{
#if AP_COMPASS_EXTERNALAHRS_ENABLED
    SBGPacketSet *packet = (SBGPacketSet*)packet_buf;
    const SBGMag &sbg_mag = packet->mag;

    AP_ExternalAHRS::mag_data_message_t mag;
    mag.field = Vector3f{sbg_mag.mag_x, sbg_mag.mag_y, sbg_mag.mag_z};
    mag.field *= 1000; // quite possibly to mGauss

    AP::compass().handle_external(mag);
#endif
}

void AP_ExternalAHRS_SBG::update_state_baro(void)
{
#if AP_BARO_EXTERNALAHRS_ENABLED
    SBGPacketSet *packet = (SBGPacketSet*)packet_buf;

    AP_ExternalAHRS::baro_data_message_t baro;
    baro.instance = 0;
    baro.pressure_pa = packet->air_data.pressure_abs_pa;
    // temperature in air_data packet is always 0 as it's for airspeed
    baro.temperature = packet->imu_short.temperature*(1.0f/(1<<8));

    AP::baro().handle_external(baro);
#endif
}

void AP_ExternalAHRS_SBG::update_state_gps(void)
{
    SBGPacketSet *packet = (SBGPacketSet*)packet_buf;
    const SBGGPS1Pos &pos = packet->gps1_pos;
    const SBGGPS1Vel &vel = packet->gps1_vel;

    AP_ExternalAHRS::gps_data_message_t gps;

    gps.gps_week = 0xFFFF; // not known (annoying to calculate)
    gps.ms_tow = pos.time_of_week_ms;
    {
        uint8_t status = pos.pos_status & 0x3F;
        uint8_t type = (pos.pos_status >> 6) & 0x3F;

        uint8_t fix = GPS_FIX_TYPE_NO_FIX; // default if fix type unknown
        if (status >= 2) { // internal error
            fix = GPS_FIX_TYPE_NO_GPS;
        } else if (status == 1) { // insufficient observations
            fix = GPS_FIX_TYPE_NO_FIX;
        } else if (type == 2) { // single point solution
            fix = GPS_FIX_TYPE_3D_FIX;
        } else if ((type >= 3) || (type <= 5)) { // various DGPS/augmented types
            fix = GPS_FIX_TYPE_DGPS;
        } else if (type == 6) { // RTK float
            fix = GPS_FIX_TYPE_RTK_FLOAT;
        } else if (type == 7) { // RTK int
            fix = GPS_FIX_TYPE_RTK_FIXED;
        }
        gps.fix_type = fix;
    }
    gps.satellites_in_view = pos.num_satellites;

    gps.horizontal_pos_accuracy =
        sqrt(pos.latitude_acc_m*pos.latitude_acc_m+
             pos.longitude_acc_m*pos.longitude_acc_m);
    gps.vertical_pos_accuracy = pos.altitude_acc_m;
    gps.horizontal_vel_accuracy =
        sqrt(vel.vel_n_acc_ms*vel.vel_n_acc_ms+
             vel.vel_e_acc_ms*vel.vel_e_acc_ms);

    gps.hdop = 0; // not provided
    gps.vdop = 0;

    gps.latitude = pos.latitude_deg * 1.0e7;
    gps.longitude = pos.longitude_deg * 1.0e7;
    gps.msl_altitude = pos.altitude_m * 1.0e2;

    gps.ned_vel_north = vel.vel_n_ms;
    gps.ned_vel_east = vel.vel_e_ms;
    gps.ned_vel_down = vel.vel_d_ms;

    if (gps.fix_type >= GPS_FIX_TYPE_3D_FIX && !state.have_origin) {
        WITH_SEMAPHORE(state.sem);
        state.origin = Location{int32_t(pos.latitude_deg * 1.0e7),
                                int32_t(pos.longitude_deg * 1.0e7),
                                int32_t(pos.altitude_m * 1.0e2),
                                Location::AltFrame::ABSOLUTE};
        state.have_origin = true;
    }
    uint8_t instance;
    if (AP::gps().get_first_external_instance(instance)) {
        AP::gps().handle_external(gps, instance);
    }
}

// get serial port number for the uart
int8_t AP_ExternalAHRS_SBG::get_port(void) const
{
    if (!uart) {
        return -1;
    }
    return port_num;
};

// accessors for AP_AHRS
bool AP_ExternalAHRS_SBG::healthy(void) const
{
    const uint32_t now = AP_HAL::millis();
    // TODO: better
    return (now - last_packet_ms < 50) && initialised();
}

bool AP_ExternalAHRS_SBG::initialised(void) const
{
    SBGPacketSet *packet = (SBGPacketSet*)packet_buf;

    if (!setup_complete) {
        return false;
    }

    // we've received at least one of every packet
    return packet->ekf_euler.timestamp_us
        && packet->ekf_nav.timestamp_us
        && packet->gps1_pos.timestamp_us
        && packet->gps1_vel.timestamp_us
        && packet->utc_time.timestamp_us
        && packet->imu_short.timestamp_us
        && packet->mag.timestamp_us
        && packet->air_data.timestamp_us;
}

bool AP_ExternalAHRS_SBG::pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const
{
    if (!setup_complete) {
        hal.util->snprintf(failure_msg, failure_msg_len, "SBG setup failed");
        return false;
    }
    if (!healthy()) {
        hal.util->snprintf(failure_msg, failure_msg_len, "SBG unhealthy");
        return false;
    }

    bool have_origin;
    {
        WITH_SEMAPHORE(state.sem);
        have_origin = state.have_origin;
    }
    if (!have_origin) {
        hal.util->snprintf(failure_msg, failure_msg_len, "SBG no GPS1 lock");
        return false;
    }
    return true;
}

/*
  get filter status
 */
void AP_ExternalAHRS_SBG::get_filter_status(nav_filter_status &status) const
{
    // TODO: better
    memset(&status, 0, sizeof(status));
    bool have_origin;
    {
        WITH_SEMAPHORE(state.sem);
        have_origin = state.have_origin;
    }

    status.flags.initalized = initialised();
    if (healthy() && have_origin) {
        status.flags.attitude = true;
        status.flags.vert_vel = true;
        status.flags.vert_pos = true;

        status.flags.horiz_vel = true;
        status.flags.horiz_pos_rel = true;
        status.flags.horiz_pos_abs = true;
        status.flags.pred_horiz_pos_rel = true;
        status.flags.pred_horiz_pos_abs = true;
        status.flags.using_gps = true;
    }
}

// send an EKF_STATUS message to GCS
void AP_ExternalAHRS_SBG::send_status_report(GCS_MAVLINK &link) const
{
    if (!initialised()) {
        return;
    }
    // prepare flags
    uint16_t flags = 0;
    nav_filter_status filterStatus;
    get_filter_status(filterStatus);
    if (filterStatus.flags.attitude) {
        flags |= EKF_ATTITUDE;
    }
    if (filterStatus.flags.horiz_vel) {
        flags |= EKF_VELOCITY_HORIZ;
    }
    if (filterStatus.flags.vert_vel) {
        flags |= EKF_VELOCITY_VERT;
    }
    if (filterStatus.flags.horiz_pos_rel) {
        flags |= EKF_POS_HORIZ_REL;
    }
    if (filterStatus.flags.horiz_pos_abs) {
        flags |= EKF_POS_HORIZ_ABS;
    }
    if (filterStatus.flags.vert_pos) {
        flags |= EKF_POS_VERT_ABS;
    }
    if (filterStatus.flags.terrain_alt) {
        flags |= EKF_POS_VERT_AGL;
    }
    if (filterStatus.flags.const_pos_mode) {
        flags |= EKF_CONST_POS_MODE;
    }
    if (filterStatus.flags.pred_horiz_pos_rel) {
        flags |= EKF_PRED_POS_HORIZ_REL;
    }
    if (filterStatus.flags.pred_horiz_pos_abs) {
        flags |= EKF_PRED_POS_HORIZ_ABS;
    }
    if (!filterStatus.flags.initalized) {
        flags |= EKF_UNINITIALIZED;
    }

    // send message
    // TODO: better
    const float vel_gate = 5;
    const float pos_gate = 5;
    const float hgt_gate = 5;
    const float mag_var = 0;
    mavlink_msg_ekf_status_report_send(link.get_chan(), flags,
                                       0/vel_gate, 0/pos_gate, 0/hgt_gate,
                                       mag_var, 0, 0);
}

#endif  // AP_EXTERNAL_AHRS_SBG_ENABLED
