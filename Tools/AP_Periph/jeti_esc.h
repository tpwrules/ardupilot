/*
  ESC Telemetry for Jeti Spin Pro 99 OPTO ESC. This will be
  incorporated into a broader ESC telemetry library in ArduPilot
  master in the future
 */

#pragma once

#include <AP_RCProtocol/SoftSerial.h>

#ifdef HAL_PERIPH_ENABLE_JETIESC

class JETIESC_Telem {
public:
    JETIESC_Telem();

    void init();
    bool update();

    struct JETIESC {
        uint8_t power_pct;
        float rpm;
        float voltage;
        float temp_degc;
    };

    const JETIESC &get_telem(void) {
        return decoded;
    }

private:
    SoftSerial *ss;

    struct PACKED {
        uint8_t header; // 0xFE
        uint8_t message[32];
        uint8_t footer; // 0xFF
    } pkt;

    // uint8_t len;
    // uint32_t last_read_ms;
    // uint32_t error_count;

    struct JETIESC decoded;

    bool parse_packet(void);
};

#endif // HAL_PERIPH_ENABLE_JETIESC
