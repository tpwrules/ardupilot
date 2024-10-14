/*
  ESC Telemetry for Jeti Spin Pro 99 OPTO ESC. This will be
  incorporated into a broader ESC telemetry library in ArduPilot
  master in the future

  This protocol only allows for one ESC per UART RX line, so using a
  CAN node per ESC works well.
 */

#include "AP_Periph.h"
#include "jeti_esc.h"
#include <AP_HAL/utility/sparse-endian.h>
#include <dronecan_msgs.h>

#ifdef HAL_PERIPH_ENABLE_JETIESC

#include <SITL/SITL.h>

extern const AP_HAL::HAL& hal;

#define TELEM_HEADER 0xFE
#define TELEM_LEN    34
#define TELEM_FOOTER 0xFF

// constructor
JETIESC_Telem::JETIESC_Telem(void)
: ss{9700, SoftSerial::SERIAL_CONFIG_9O2}
{
}

void JETIESC_Telem::init()
{
    can_printf("jeti init\n");
#if HAL_USE_ICU == TRUE
    //attach timer channel on which the signal will be received
    sig_reader.attach_capture_timer(&RCIN_ICU_TIMER, RCIN_ICU_CHANNEL, STM32_RCIN_DMA_STREAM, STM32_RCIN_DMA_CHANNEL);
#endif

#if HAL_USE_EICU == TRUE
    sig_reader.init(&RCININT_EICU_TIMER, RCININT_EICU_CHANNEL);
#endif
}

/*
  update ESC telemetry
 */
bool JETIESC_Telem::update()
{
#if HAL_USE_ICU == TRUE
    const uint32_t *p;
    uint32_t n;
    while ((p = (const uint32_t *)sig_reader.sigbuf.readptr(n)) != nullptr) {
        process_pulse_list(p, n*2, sig_reader.need_swap);
        sig_reader.sigbuf.advance(n);
    }
#endif

#if HAL_USE_EICU == TRUE
    uint32_t width_s0, width_s1;
    while(sig_reader.read(width_s0, width_s1)) {
        process_pulse(width_s0, width_s1);
    }
#endif
    // read new data and process from _ss
    return false;

    // uint32_t n = uart->available();
    // if (n == 0) {
    //     return false;
    // }

    // // we expect at least 50ms idle between frames
    // uint32_t now = AP_HAL::millis();
    // bool frame_gap = (now - last_read_ms) > 10;

    // last_read_ms = now;

    // // don't read too much in one loop to prevent too high CPU load
    // if (n > 500) {
    //     n = 500;
    // }
    // if (len == 0 && !frame_gap) {
    //     uart->discard_input();
    //     return false;
    // }

    // if (frame_gap) {
    //     len = 0;
    // }

    // bool ret = false;

    // while (n--) {
    //     uint8_t b = uart->read();
    //     //hal.console->printf("t=%u 0x%02x\n", now, b);
    //     if (len == 0 && b != TELEM_HEADER) {
    //         continue;
    //     }
    //     if (len == 1 && b != TELEM_LEN) {
    //         continue;
    //     }
    //     uint8_t *buf = (uint8_t *)&pkt;
    //     buf[len++] = b;
    //     if (len == sizeof(pkt)) {
    //         ret = parse_packet();
    //         len = 0;
    //     }
    // }
    // return ret;
}

void JETIESC_Telem::process_pulse(uint32_t width_s0, uint32_t width_s1)
{
    uint8_t b;
    if (ss.process_pulse(width_s0, width_s1, b)) {
        can_printf("%c\n", b);
    }
}

/*
  process an array of pulses. n must be even
 */
void JETIESC_Telem::process_pulse_list(const uint32_t *widths, uint16_t n, bool need_swap)
{
    if (n & 1) {
        return;
    }
    while (n) {
        uint32_t widths0 = widths[0];
        uint32_t widths1 = widths[1];
        if (need_swap) {
            uint32_t tmp = widths1;
            widths1 = widths0;
            widths0 = tmp;
        }
        widths1 -= widths0;
        process_pulse(widths0, widths1);
        widths += 2;
        n -= 2;
    }
}

/*
  parse packet
 */
bool JETIESC_Telem::parse_packet(void)
{
    // decode the bits from the thing lol
    return false;

    // uint16_t crc = calc_crc((uint8_t *)&pkt, sizeof(pkt)-2);
    // if (crc != pkt.crc) {
    //     return false;
    // }

    // decoded.counter = be32toh(pkt.counter);
    // decoded.throttle_req = be16toh(pkt.throttle_req);
    // decoded.throttle = be16toh(pkt.throttle);
    // decoded.rpm = be16toh(pkt.rpm) * 5.0 / 7.0; // scale from eRPM to RPM
    // decoded.voltage = be16toh(pkt.voltage) * 0.1;
    // decoded.phase_current = int16_t(be16toh(pkt.phase_current)) * 0.01;
    // decoded.current = int16_t(be16toh(pkt.current)) * 0.01;
    // decoded.mos_temperature = temperature_decode(pkt.mos_temperature);
    // decoded.cap_temperature = temperature_decode(pkt.cap_temperature);
    // decoded.status = be16toh(pkt.status);
    // if (decoded.status != 0) {
    //     decoded.error_count++;
    // }

    // return true;
}

void AP_Periph_FW::jetiesc_telem_update()
{
    if (!jetiesc_telem.update()) {
        return;
    }
    const JETIESC_Telem::JETIESC &t = jetiesc_telem.get_telem();

    uavcan_equipment_esc_Status pkt {};
    pkt.esc_index = 0; // TODO: figure out how to choose nicely
    pkt.voltage = t.voltage;
    pkt.current = 420;
    pkt.temperature = C_TO_KELVIN(0);
    pkt.rpm = 69;
    pkt.power_rating_pct = 0;
    pkt.error_count = 0;

    uint8_t buffer[UAVCAN_EQUIPMENT_ESC_STATUS_MAX_SIZE];
    uint16_t total_size = uavcan_equipment_esc_Status_encode(&pkt, buffer, !canfdout());
    canard_broadcast(UAVCAN_EQUIPMENT_ESC_STATUS_SIGNATURE,
                    UAVCAN_EQUIPMENT_ESC_STATUS_ID,
                    CANARD_TRANSFER_PRIORITY_LOW,
                    &buffer[0],
                    total_size);
}

#endif // HAL_PERIPH_ENABLE_JETIESC

