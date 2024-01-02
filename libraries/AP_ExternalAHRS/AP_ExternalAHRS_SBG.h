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

#pragma once

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_SBG_ENABLED

#include "AP_ExternalAHRS_backend.h"

class AP_ExternalAHRS_SBG : public AP_ExternalAHRS_backend {

public:
    AP_ExternalAHRS_SBG(AP_ExternalAHRS *frontend, AP_ExternalAHRS::state_t &state);

    // get serial port number, -1 for not enabled
    int8_t get_port(void) const override;

    // accessors for AP_AHRS
    bool healthy(void) const override;
    bool initialised(void) const override;
    bool pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const override;
    void get_filter_status(nav_filter_status &status) const override;
    void send_status_report(class GCS_MAVLINK &link) const override;

    // check for new data
    void update() override {
        check_uart();
    }

    // Get model/type name
    const char* get_name() const override;

private:
    AP_HAL::UARTDriver *uart;
    int8_t port_num;
    bool setup_complete;
    uint32_t baudrate;

    void update_thread();
    bool check_uart();

    uint16_t unframe_packet(const uint8_t *data, uint16_t len, bool &valid);
    uint16_t process_message(uint8_t id, const uint8_t *data, uint16_t len);
    void update_state_ekf(void);
    void update_state_imu(void);
    void update_state_mag(void);
    void update_state_baro(void);
    void update_state_gps(void);
    void update_state_event_a(void);

    uint8_t *data_buf;
    uint16_t buf_len;

    void *packet_buf;
    uint32_t last_packet_ms;
};

#endif  // AP_EXTERNAL_AHRS_SBG_ENABLED
