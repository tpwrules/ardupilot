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
#include <cstring>   // For strncpy()
#include <cstdlib>   // For atoi(), atof()


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
    if (len == 0) {
        // Header
        if (b == 0xFE) {
            pkt.header = b; // Store the header
            len = 1; // Move to the next byte
        }
    } else if (len > 0 && len <= 32) {
    // Message
    pkt.message[len - 1] = b; // Fill the message array
    len++;
    } else if (len == 33) {
        // Footer
        if (b == 0xFF) {
            pkt.footer = b; // Store the footer
            len = 0;        // Reset the length to be ready for the next packet
            // once ready to parse the packet:
            packet_decoded = parse_packet();
        } else {
            // Footer mismatch, reset length (start over)
            len = 0;
        }
    }

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

  Packet Format (during operation):
  message is two 16 character zones with no separator
  zone 1: "Sppp%   RRRRRrpm"
    * S: status letter
    * p: power/braking percent
    * R: motor RPM
  zone 2: "vv,vvV    ttttdC"
    * v: voltage (note comma instead of decimal!!)
    * t: temperature degrees Celsius
    * d: degree symbol
 */
bool JETIESC_Telem::parse_packet(void)
{
    // 2. read the message out of the pkt and fill out the decoded. if
    //    everything looks good, return true, else return false.
    //    possibly using atoi/atof or sscanf? note that it's two lines and
    //    also not null terminated.
    if (pkt.header != 0xFE || pkt.footer != 0xFF) {
        return false;
    }
    // Parse Zone 1 ("Sppp%   RRRRRrpm")
    
    char power_pct_str[4];     // To hold the power percentage (3 characters + null terminator)
    power_pct_str[3] = '\0';
    memcpy(power_pct_str, &pkt.message[1], 3);
    int power_pct = atoi(power_pct_str);
    
    char rpm_str[6];           // To hold the RPM (5 characters + null terminator)
    rpm_str[5] = '\0';
    memcpy(rpm_str, &pkt.message[8], 5);
    int rpm = atoi(rpm_str);                      // Convert RPM to integer

    // Parse Zone 2 ("vv,vvV    ttttdC")
    // Copy the voltage part (formatted as "vv,vv") and the temperature part
    char voltage_str[6];       // To hold voltage (formatted as "vv,vv" + null terminator)
    voltage_str[5] = '\0';
    memcpy(voltage_str, &pkt.message[16], 5);
    
    // Voltage is in the format "vv,vv", replace the comma and use atof()
    voltage_str[2] = '.';                         // Replace the comma with a decimal point
    float voltage = atof(voltage_str);            // Convert voltage to float

    char temp_str[5];          // To hold temperature (4 characters + null terminator)
    temp_str[4] = '\0';
    memcpy(temp_str, &pkt.message[26], 4);
    int temp_degc = atoi(temp_str);

    // Populate the decoded structure
    decoded.power_pct = static_cast<uint8_t>(power_pct);
    decoded.rpm = static_cast<float>(rpm);
    decoded.voltage = voltage;
    decoded.temp_degc = static_cast<float>(temp_degc);

    // You may want to handle the status letter separately if needed
    // For example, check if the status indicates an error or special condition

    return true;
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

