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
, packet_decoded(false)
, len(0)
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

    return packet_decoded;
}

void JETIESC_Telem::process_byte(uint8_t b)
{
    // 1. process bytes into the pkt variable and form packets.
    //    helpful to keep track of length

    // once ready to parse the packet:
    packet_decoded = parse_packet();
}

void JETIESC_Telem::process_pulse(uint32_t width_s0, uint32_t width_s1)
{
    uint8_t b;
    if (ss.process_pulse(width_s0, width_s1, b)) {
        can_printf("%c\n", b);
        process_byte(b);
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
    // 2. read the message out of the pkt and fill out the decoded. if
    //    everything looks good, return true, else return false.
    //    possibly using sscanf? note that it's two lines and also not a string.
    return false;

    // decoded.temp_degc = whatever;
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
    pkt.current = 0;
    pkt.temperature = C_TO_KELVIN(t.temp_degc);
    pkt.rpm = t.rpm;
    pkt.power_rating_pct = t.power_pct;
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

