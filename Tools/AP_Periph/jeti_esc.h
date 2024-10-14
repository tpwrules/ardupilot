/*
  ESC Telemetry for Jeti Spin Pro 99 OPTO ESC. This will be
  incorporated into a broader ESC telemetry library in ArduPilot
  master in the future
 */

#pragma once

#include <AP_HAL_ChibiOS/AP_HAL_ChibiOS.h>

#include <AP_RCProtocol/SoftSerial.h>

#if HAL_USE_ICU == TRUE
#include <AP_HAL_ChibiOS/SoftSigReader.h>
#endif

#if HAL_USE_EICU == TRUE
#include <AP_HAL_ChibiOS/SoftSigReaderInt.h>
#endif

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
    // software serial and pulse stuff
#if HAL_USE_ICU == TRUE
    ChibiOS::SoftSigReader sig_reader;
#endif

#if HAL_USE_EICU == TRUE
    ChibiOS::SoftSigReaderInt sig_reader;
#endif

    void process_pulse(uint32_t width_s0, uint32_t width_s1);
    void process_pulse_list(const uint32_t *widths, uint16_t n, bool need_swap);

    SoftSerial ss;

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
