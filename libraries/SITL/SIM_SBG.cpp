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
  simulate SBG serial AHRS
*/

#include "SIM_SBG.h"
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <AP_Common/NMEA.h>

using namespace SITL;

#define SBG_ECOM_SYNC_1 (0xFF)
#define SBG_ECOM_SYNC_2 (0x5A)
#define SBG_ECOM_ETX (0x33)
#define SBG_ECOM_MAX_PACKET_SIZE (4095)
#define SBG_ECOM_CLASS_LOG_ECOM_0 (0)

#define SBG_ECOM_LOG_EKF_EULER (6)
#define SBG_ECOM_LOG_EKF_NAV (8)
#define SBG_ECOM_LOG_GPS1_POS (14)
#define SBG_ECOM_LOG_GPS1_VEL (13)
#define SBG_ECOM_LOG_UTC_TIME (2)
#define SBG_ECOM_LOG_IMU_SHORT (44)
#define SBG_ECOM_LOG_MAG (4)
#define SBG_ECOM_LOG_AIR_DATA (36)
#define SBG_ECOM_LOG_EVENT_A (24)


// valid is true if data points to a valid packet.
// returns amount of data to consume/packet length
static uint16_t unframe_packet(const uint8_t *data, uint16_t len, bool &valid)
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
        return p - data;
    } else {
        return len;
    }
}

SBG::SBG() :
    SerialDevice::SerialDevice()
{
    fp = fopen("input.ecom", "rb");
    data_buf = new uint8_t[SBG_ECOM_MAX_PACKET_SIZE];
}

struct PACKED VN_packet1 {
    float uncompMag[3];
    float uncompAccel[3];
    float uncompAngRate[3];
    float pressure;
    float mag[3];
    float accel[3];
    float gyro[3];
    float ypr[3];
    float quaternion[4];
    float yprU[3];
    double positionLLA[3];
    float velNED[3];
    float posU;
    float velU;
};

struct PACKED VN_packet2 {
    uint64_t timeGPS;
    float temp;
    uint8_t numGPS1Sats;
    uint8_t GPS1Fix;
    double GPS1posLLA[3];
    float GPS1velNED[3];
    float GPS1DOP[7];
    uint8_t numGPS2Sats;
    uint8_t GPS2Fix;
};

#define VN_PKT1_HEADER { 0xFA, 0x34, 0x2E, 0x07, 0x06, 0x01, 0x12, 0x06 }
#define VN_PKT2_HEADER { 0xFA, 0x4E, 0x02, 0x00, 0x10, 0x00, 0xB8, 0x20, 0x18, 0x00 }

/*
  get timeval using simulation time
 */
static void simulation_timeval(struct timeval *tv)
{
    uint64_t now = AP_HAL::micros64();
    static uint64_t first_usec;
    static struct timeval first_tv;
    if (first_usec == 0) {
        first_usec = now;
        first_tv.tv_sec = AP::sitl()->start_time_UTC;
    }
    *tv = first_tv;
    tv->tv_sec += now / 1000000ULL;
    uint64_t new_usec = tv->tv_usec + (now % 1000000ULL);
    tv->tv_sec += new_usec / 1000000ULL;
    tv->tv_usec = new_usec % 1000000ULL;
}

void SBG::send_packet1(void)
{
    const auto &fdm = _sitl->state;

    struct VN_packet1 pkt {};

    pkt.uncompAccel[0] = fdm.xAccel;
    pkt.uncompAccel[1] = fdm.yAccel;
    pkt.uncompAccel[2] = fdm.zAccel;
    const float gyro_noise = 0.05;
    pkt.uncompAngRate[0] = radians(fdm.rollRate + gyro_noise * rand_float());
    pkt.uncompAngRate[1] = radians(fdm.pitchRate + gyro_noise * rand_float());
    pkt.uncompAngRate[2] = radians(fdm.yawRate + gyro_noise * rand_float());

    float sigma, delta, theta;
    AP_Baro::SimpleAtmosphere(fdm.altitude * 0.001f, sigma, delta, theta);
    pkt.pressure = SSL_AIR_PRESSURE * delta * 0.001 + rand_float() * 0.01;

    pkt.mag[0] = fdm.bodyMagField.x*0.001;
    pkt.mag[1] = fdm.bodyMagField.y*0.001;
    pkt.mag[2] = fdm.bodyMagField.z*0.001;
    pkt.uncompMag[0] = pkt.mag[0];
    pkt.uncompMag[1] = pkt.mag[1];
    pkt.uncompMag[2] = pkt.mag[2];

    pkt.accel[0] = fdm.xAccel;
    pkt.accel[1] = fdm.yAccel;
    pkt.accel[2] = fdm.zAccel;
    pkt.gyro[0] = radians(fdm.rollRate + rand_float() * gyro_noise);
    pkt.gyro[1] = radians(fdm.pitchRate + rand_float() * gyro_noise);
    pkt.gyro[2] = radians(fdm.yawRate + rand_float() * gyro_noise);

    pkt.ypr[0] = fdm.yawDeg;
    pkt.ypr[1] = fdm.pitchDeg;
    pkt.ypr[2] = fdm.rollDeg;

    pkt.quaternion[0] = fdm.quaternion.q2;
    pkt.quaternion[1] = fdm.quaternion.q3;
    pkt.quaternion[2] = fdm.quaternion.q4;
    pkt.quaternion[3] = fdm.quaternion.q1;

    pkt.positionLLA[0] = fdm.latitude;
    pkt.positionLLA[1] = fdm.longitude;
    pkt.positionLLA[2] = fdm.altitude;
    pkt.velNED[0] = fdm.speedN;
    pkt.velNED[1] = fdm.speedE;
    pkt.velNED[2] = fdm.speedD;
    pkt.posU = 0.5;
    pkt.velU = 0.25;

    const uint8_t header[] VN_PKT1_HEADER;

    write_to_autopilot((char *)&header, sizeof(header));
    write_to_autopilot((char *)&pkt, sizeof(pkt));

    uint16_t crc = crc16_ccitt(&header[1], sizeof(header)-1, 0);
    crc = crc16_ccitt((const uint8_t *)&pkt, sizeof(pkt), crc);
    uint16_t crc2;
    swab(&crc, &crc2, 2);

    write_to_autopilot((char *)&crc2, sizeof(crc2));
}

void SBG::send_packet2(void)
{
    const auto &fdm = _sitl->state;

    struct VN_packet2 pkt {};

    struct timeval tv;
    simulation_timeval(&tv);

    pkt.timeGPS = tv.tv_usec * 1000ULL;
    pkt.temp = 23.5;
    pkt.numGPS1Sats = 19;
    pkt.GPS1Fix = 3;
    pkt.GPS1posLLA[0] = fdm.latitude;
    pkt.GPS1posLLA[1] = fdm.longitude;
    pkt.GPS1posLLA[2] = fdm.altitude;
    pkt.GPS1velNED[0] = fdm.speedN;
    pkt.GPS1velNED[1] = fdm.speedE;
    pkt.GPS1velNED[2] = fdm.speedD;
    // pkt.GPS1DOP =
    pkt.numGPS2Sats = 18;
    pkt.GPS2Fix = 3;

    const uint8_t header[] VN_PKT2_HEADER;

    write_to_autopilot((char *)&header, sizeof(header));
    write_to_autopilot((char *)&pkt, sizeof(pkt));

    uint16_t crc = crc16_ccitt(&header[1], sizeof(header)-1, 0);
    crc = crc16_ccitt((const uint8_t *)&pkt, sizeof(pkt), crc);

    uint16_t crc2;
    swab(&crc, &crc2, 2);

    write_to_autopilot((char *)&crc2, sizeof(crc2));
}

void SBG::nmea_printf(const char *fmt, ...)
{
    va_list ap;

    va_start(ap, fmt);
    char *s = nmea_vaprintf(fmt, ap);
    va_end(ap);
    if (s != nullptr) {
        write_to_autopilot((const char*)s, strlen(s));
        free(s);
    }
}

/*
  send SBG data
 */
void SBG::update(void)
{
    if (!init_sitl_pointer()) {
        return;
    }

    uint32_t now = AP_HAL::micros();
    while (true) {
        while (!curr_packet_len) {
            uint16_t buf_room = SBG_ECOM_MAX_PACKET_SIZE - buf_len;
            if (buf_room) {
                int bytes_read = fread(&data_buf[buf_len], 1, buf_room, fp);
                if (bytes_read <= 0) return;
                buf_len += bytes_read;
            }
            bool valid;
            uint16_t consume = unframe_packet(&data_buf[0], buf_len, valid);
            if (valid) {
                curr_packet_len = consume;
            } else {
                uint16_t remaining = buf_len - consume;
                if (consume && remaining) {
                    memmove(&data_buf[0], &data_buf[consume], remaining);
                }
                buf_len = remaining;
            }
        }
        uint8_t *msg = &data_buf[6];
        uint16_t msg_len = curr_packet_len - 9;
        uint16_t msg_id = data_buf[2];
        if (msg_len >= 4 && (
                msg_id == SBG_ECOM_LOG_EKF_EULER ||
                msg_id == SBG_ECOM_LOG_EKF_NAV ||
                msg_id == SBG_ECOM_LOG_GPS1_POS ||
                msg_id == SBG_ECOM_LOG_GPS1_VEL ||
                msg_id == SBG_ECOM_LOG_UTC_TIME ||
                msg_id == SBG_ECOM_LOG_IMU_SHORT ||
                msg_id == SBG_ECOM_LOG_MAG ||
                msg_id == SBG_ECOM_LOG_AIR_DATA ||
                msg_id == SBG_ECOM_LOG_EVENT_A)) {
            uint32_t timestamp_us = msg[0]|(msg[1]<<8)|(msg[2]<<16)|(msg[3]<<24);
            if (!timestamp_ofs_us) {
                timestamp_ofs_us = timestamp_us; // first message at sim time 0
            }
            // ensure we don't wait forever if the first message we see isn't
            // the earliest in the file
            if (((timestamp_us-timestamp_ofs_us) > now) && (timestamp_us > timestamp_ofs_us)) {
                // not time yet
                break;
            }
            write_to_autopilot((const char*)&data_buf[0], curr_packet_len);
        }
        uint16_t remaining = buf_len - curr_packet_len;
        if (remaining) {
            memmove(&data_buf[0], &data_buf[curr_packet_len], remaining);
        }
        curr_packet_len = 0;
        buf_len = remaining;
    }
}
