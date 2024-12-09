/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Code by Charles "Silvanosky" Villard, David "Buzz" Bussenschutt,
 * Andrey "ARg" Romanov, and Thomas "tpw_rules" Watson
 *
 */

#include "RCOutput.h"

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_BoardConfig/AP_BoardConfig.h>

#include "driver/rtc_io.h"

#include <stdio.h>

#include "esp_log.h"

// default settings for PWM ("normal") and brushed mode
#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000   // 1MHz, 1000ns per tick
#define SERVO_TIMEBASE_PERIOD        20000     // 20K ticks, 20ms (50Hz)

#define BRUSH_TIMEBASE_RESOLUTION_HZ 40000000  // 40MHz, 25ns per tick
#define BRUSH_TIMEBASE_PERIOD        1250      // 1250 ticks, 31.25us (32KHz)

#define TAG "RCOut"

extern const AP_HAL::HAL& hal;

using namespace ESP32;

#ifdef HAL_ESP32_RCOUT

gpio_num_t outputs_pins[] = HAL_ESP32_RCOUT;

//If the RTC source is not required, then GPIO32/Pin12/32K_XP and GPIO33/Pin13/32K_XN can be used as digital GPIOs.

#else
gpio_num_t outputs_pins[] = {};

#endif

/*
 * The ESP32/ESP32S3 MCPWM (motor control PWM) peripheral is used to generate
 * PWM signals for RC output. It is divided up into the following blocks:
 *  * The chip has SOC_MCPWM_GROUPS (2) groups
 *  * Each group has SOC_MCPWM_TIMERS_PER_GROUP (3) timers and operators
 *  * Each operator has SOC_MCPWM_COMPARATORS_PER_OPERATOR (2) comparators and
 *    generators
 *  * Each generator can drive one GPIO pin
 * Though there is more possible, we use the following capabilities:
 *  * Groups have an 8 bit integer divider from the 160MHz peripheral clock
 *  * Each timer has an 8 bit integer divider from the group clock, a 16 bit
 *    period, and is connected to exactly one operator
 *  * Each comparator in an operator acts on the corresponding timer's value and
 *    is connected to exactly one generator which drives exactly one GPIO pin
 *
 * Therefore, each MCPWM group gives us 3 independent "PWM groups" (in the STM32
 * sense) which contain 2 GPIO pins. The pins are assigned consecutively from
 * the HAL_ESP32_RCOUT list. The frequency of each group can be controlled
 * independently by changing that timer's period.
 *  * For regular PWM output, running the timer at 1MHz allows 16-1000Hz with at
 *    least 1000 ticks per cycle. It also makes setting the compare value easy
 *  * For brushed PWM output, running the timer at 40MHz allows 650-32000Hz with
 *    at least 1000 ticks per cycle. It also makes an easy divider setting.
 *
 * MCPWM is only capable of PWM; DMA-based modes will require using the RMT
 * peripheral.
 *
 */

// each of our PWM groups has its own timer
#define MAX_GROUPS (SOC_MCPWM_GROUPS*SOC_MCPWM_TIMERS_PER_GROUP)
// we connect one timer to one operator
static_assert(SOC_MCPWM_OPERATORS_PER_GROUP >= SOC_MCPWM_TIMERS_PER_GROUP);
// and one generator to one comparator
static_assert(SOC_MCPWM_GENERATORS_PER_OPERATOR >= SOC_MCPWM_COMPARATORS_PER_OPERATOR);

#define MAX_CHANNELS ARRAY_SIZE(outputs_pins)

#define NEEDED_GROUPS ((MAX_CHANNELS+SOC_MCPWM_COMPARATORS_PER_OPERATOR-1)/SOC_MCPWM_COMPARATORS_PER_OPERATOR)
static_assert(NEEDED_GROUPS <= MAX_GROUPS, "not enough hardware PWM groups");

RCOutput::pwm_group RCOutput::pwm_group_list[NEEDED_GROUPS];
RCOutput::pwm_out RCOutput::pwm_out_list[MAX_CHANNELS];

void RCOutput::init()
{
    _max_channels = MAX_CHANNELS;

#ifdef CONFIG_IDF_TARGET_ESP32
    // only on plain esp32
    // 32 and 33 are special as they dont default to gpio, but can be if u disable their rtc setup:
    rtc_gpio_deinit(GPIO_NUM_32);
    rtc_gpio_deinit(GPIO_NUM_33);
#endif

    printf("oooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo\n");
    printf("RCOutput::init() - channels available: %d \n",(int)MAX_CHANNELS);
    printf("oooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooooo\n");

    _initialized = true; // assume we are initialized, any error will call abort()

    RCOutput::pwm_group *curr_group = &pwm_group_list[0];
    RCOutput::pwm_out *curr_out = &pwm_out_list[0];
    int curr_out_num = 0;

    // loop through all the hardware blocks and set them up. returns when we run out of GPIOs.
    for (int mcpwm_group_id=0; mcpwm_group_id<SOC_MCPWM_GROUPS; mcpwm_group_id++) {
        for (int timer_num=0; timer_num<SOC_MCPWM_TIMERS_PER_GROUP; timer_num++) {
            // set default 1MHz tick rate and 50Hz frequency
            mcpwm_timer_config_t timer_config = {
                .group_id = mcpwm_group_id,
                .clk_src = MCPWM_TIMER_CLK_SRC_PLL160M,
                .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
                .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
                .period_ticks = SERVO_TIMEBASE_PERIOD,
            };

            // operator connects timer and comparator
            mcpwm_operator_config_t operator_config = {
                .group_id = mcpwm_group_id,
            };

            // configure group
            curr_group->mcpwm_group_id = mcpwm_group_id;
            curr_group->rc_frequency = SERVO_TIMEBASE_RESOLUTION_HZ/SERVO_TIMEBASE_PERIOD;
            curr_group->current_mode = MODE_PWM_NORMAL;
            curr_group->ch_mask = 0;

            // create timer and configure it to constantly run
            ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &curr_group->h_timer));
            ESP_ERROR_CHECK(mcpwm_timer_enable(curr_group->h_timer));
            ESP_ERROR_CHECK(mcpwm_timer_start_stop(curr_group->h_timer, MCPWM_TIMER_START_NO_STOP));

            // create and connect operator
            ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &curr_group->h_oper));
            ESP_ERROR_CHECK(mcpwm_operator_connect_timer(curr_group->h_oper, curr_group->h_timer));

            for (int comparator_num=0; comparator_num<SOC_MCPWM_COMPARATORS_PER_OPERATOR; comparator_num++) {
                mcpwm_comparator_config_t comparator_config = {
                    .flags = {
                        .update_cmp_on_tez = true, // grab new comparator value when timer is zero
                    },
                };

                mcpwm_generator_config_t generator_config = {
                    .gen_gpio_num = outputs_pins[curr_out_num],
                };

                // configure output
                curr_out->group = curr_group;
                curr_out->gpio_num = outputs_pins[curr_out_num];
                curr_out->value = 0;
                curr_group->ch_mask |= (1U << curr_out_num);

                // create and connect comparator set to output 0
                ESP_ERROR_CHECK(mcpwm_new_comparator(curr_group->h_oper, &comparator_config, &curr_out->h_cmpr));
                ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(curr_out->h_cmpr, 0));

                // create and connect generator
                ESP_ERROR_CHECK(mcpwm_new_generator(curr_group->h_oper, &generator_config, &curr_out->h_gen));
                // go low on compare threshold (takes priority over going high)
                ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(curr_out->h_gen,
                    MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                        curr_out->h_cmpr, MCPWM_GEN_ACTION_LOW)));
                // go high on counter empty
                ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(curr_out->h_gen,
                    MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                        MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));

                curr_out++;
                if (++curr_out_num == MAX_CHANNELS) {
                    return; // done setting up the hardware
                }
            }

            curr_group++;
        }
    }
}

void RCOutput::set_freq(uint32_t chmask, uint16_t freq_hz)
{
    if (!_initialized) {
        return;
    }

    printf("set freq %04X %d %04X\n", (int)chmask, freq_hz, (int)fast_channel_mask);

    for (auto &group : pwm_group_list) {
        if ((group.ch_mask & chmask) != 0) { // group has channels to set?
            uint16_t group_freq = freq_hz;

            switch (group.current_mode) {
            case MODE_PWM_BRUSHED:
                group_freq = constrain_value((int)group_freq, 650, 32000);
                ESP_ERROR_CHECK(mcpwm_timer_set_period(group.h_timer, BRUSH_TIMEBASE_RESOLUTION_HZ/group_freq));
                break;

            case MODE_PWM_NORMAL:
            default: // i.e. NONE
                group_freq = constrain_value((int)group_freq, 16, 1000);
                ESP_ERROR_CHECK(mcpwm_timer_set_period(group.h_timer, SERVO_TIMEBASE_RESOLUTION_HZ/group_freq));
                break;
            }

            printf("set rc freq %04X %d\n", (int)(group.ch_mask & chmask), group_freq);
            group.rc_frequency = group_freq;

            // disallow changing frequency of this group if it is greater than the default
            if (group_freq > 50 || group.current_mode > MODE_PWM_NORMAL) {
                fast_channel_mask |= group.ch_mask;
            }
        }
    }
}

void RCOutput::set_default_rate(uint16_t freq_hz)
{
    if (!_initialized) {
        return;
    }

    printf("set default rate %d %04X\n", freq_hz, (int)fast_channel_mask);

    for (auto &group : pwm_group_list) {
        // only set frequency of groups without fast channels
        if (!(group.ch_mask & fast_channel_mask) && group.ch_mask) {
            set_freq(group.ch_mask, freq_hz);
            // setting a high default frequency can't make channels fast
            fast_channel_mask &= ~group.ch_mask;
        }
    }
}

/*
  setup output mode for a group, using group.current_mode.
 */
void RCOutput::set_group_mode(pwm_group &group)
{
    // calculate timer config
    mcpwm_timer_config_t timer_config;
    switch (group.current_mode) {
    case MODE_PWM_BRUSHED:
        group.rc_frequency = constrain_value((unsigned int)group.rc_frequency, 650U, 32000U);
        timer_config = {
            .group_id = group.mcpwm_group_id,
            .clk_src = MCPWM_TIMER_CLK_SRC_PLL160M,
            .resolution_hz = BRUSH_TIMEBASE_RESOLUTION_HZ,
            .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
            .period_ticks = BRUSH_TIMEBASE_RESOLUTION_HZ/group.rc_frequency,
        };
        printf("config of brush %lu = %lu\n", group.rc_frequency, BRUSH_TIMEBASE_RESOLUTION_HZ/group.rc_frequency);
        break;

    default:
        if (group.current_mode != MODE_PWM_NORMAL) {
            // set unrecognized modes to NONE (but treat them as 0 output normal)
            group.current_mode = MODE_PWM_NONE;
        }

        group.rc_frequency = constrain_value((unsigned int)group.rc_frequency, 16U, 1000U);
        timer_config = {
            .group_id = group.mcpwm_group_id,
            .clk_src = MCPWM_TIMER_CLK_SRC_PLL160M,
            .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ,
            .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
            .period_ticks = SERVO_TIMEBASE_RESOLUTION_HZ/group.rc_frequency,
        };
        break;
    }

    if (group.current_mode > MODE_PWM_NORMAL) {
        fast_channel_mask |= group.ch_mask;
    }

    // delete comparators/generators attached to this group
    for (uint8_t chan=0; chan<MAX_CHANNELS; chan++) {
        pwm_out &out = pwm_out_list[chan];
        if (out.group != &group) {
            continue;
        }

        ESP_ERROR_CHECK(mcpwm_del_generator(out.h_gen));
        ESP_ERROR_CHECK(mcpwm_del_comparator(out.h_cmpr));
    }

    // delete the operator and timer
    ESP_ERROR_CHECK(mcpwm_del_operator(group.h_oper));
    ESP_ERROR_CHECK(mcpwm_timer_disable(group.h_timer));
    ESP_ERROR_CHECK(mcpwm_del_timer(group.h_timer));

    // re-create the timer with the correct settings
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &group.h_timer));
    ESP_ERROR_CHECK(mcpwm_timer_enable(group.h_timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(group.h_timer, MCPWM_TIMER_START_NO_STOP));

    // operator connects timer and comparator
    mcpwm_operator_config_t operator_config = {
        .group_id = group.mcpwm_group_id,
    };

    // create and connect operator
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &group.h_oper));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(group.h_oper, group.h_timer));

    // re-initialize comparators/generators
    for (uint8_t chan=0; chan<MAX_CHANNELS; chan++) {
        pwm_out &out = pwm_out_list[chan];
        if (out.group != &group) {
            continue;
        }

        mcpwm_comparator_config_t comparator_config = {
            .flags = {
                .update_cmp_on_tez = true, // grab new comparator value when timer is zero
            },
        };

        mcpwm_generator_config_t generator_config = {
            .gen_gpio_num = out.gpio_num,
        };

        if (group.current_mode == MODE_PWM_NONE) {
            out.value = 0;
        }

        // create and connect comparator set to output 0
        ESP_ERROR_CHECK(mcpwm_new_comparator(group.h_oper, &comparator_config, &out.h_cmpr));
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(out.h_cmpr, 0));

        // create and connect generator
        ESP_ERROR_CHECK(mcpwm_new_generator(group.h_oper, &generator_config, &out.h_gen));
        // go low on compare threshold (takes priority over going high)
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(out.h_gen,
            MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                out.h_cmpr, MCPWM_GEN_ACTION_LOW)));
        // go high on counter empty
        ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(out.h_gen,
            MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP,
                MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    }
}

void RCOutput::set_output_mode(uint32_t mask, const enum output_mode mode)
{
    while (mask) {
        uint8_t chan = __builtin_ffs(mask)-1;
        if (!_initialized || chan >= MAX_CHANNELS) {
            return;
        }

        pwm_group &group = *pwm_out_list[chan].group;
        printf("set group mode %04X %d\n", (int)(mask & group.ch_mask), mode);
        group.current_mode = mode;
        set_group_mode(group);

        // acknowledge the setting of any channels sharing this group
        for (chan=0; chan<MAX_CHANNELS; chan++) {
            if (pwm_out_list[chan].group == &group) {
                mask &= ~(1U << chan);
            }
        }
    }
}

uint16_t RCOutput::get_freq(uint8_t chan)
{
    if (!_initialized || chan >= MAX_CHANNELS) {
        return 50;
    }

    pwm_group &group = *pwm_out_list[chan].group;
    return group.rc_frequency;
}

void RCOutput::enable_ch(uint8_t chan)
{
    if (!_initialized || chan >= MAX_CHANNELS) {
        return;
    }

    pwm_out &out = pwm_out_list[chan];
    // set output to high when timer == 0 like normal
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(out.h_gen,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
}

void RCOutput::disable_ch(uint8_t chan)
{
    if (!_initialized || chan >= MAX_CHANNELS) {
        return;
    }

    write(chan, 0);
    pwm_out &out = pwm_out_list[chan];
    // set output to low when timer == 0, so the output is always low (after this cycle)
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(out.h_gen,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_LOW)));
}

void RCOutput::write(uint8_t chan, uint16_t period_us)
{
    if (!_initialized || chan >= MAX_CHANNELS) {
        return;
    }

    if (_corked) {
        _pending[chan] = period_us;
        _pending_mask |= (1U<<chan);
    } else {
        write_int(chan, period_us);
    }

}

uint16_t RCOutput::read(uint8_t chan)
{
    if (chan >= MAX_CHANNELS || !_initialized) {
        return 0;
    }

    pwm_out &out = pwm_out_list[chan];
    return out.value;
}

void RCOutput::read(uint16_t *period_us, uint8_t len)
{
    for (int i = 0; i < MIN(len, _max_channels); i++) {
        period_us[i] = read(i);
    }
}

void RCOutput::cork()
{
    _corked = true;
}

void RCOutput::push()
{
    if (!_corked) {
        return;
    }

    bool safety_on = hal.util->safety_switch_state() == AP_HAL::Util::SAFETY_DISARMED;

    for (uint8_t i = 0; i < MAX_CHANNELS; i++) {
        if ((1U<<i) & _pending_mask) {
            uint32_t period_us = _pending[i];

            // If safety is on and safety mask not bypassing
            if (safety_on && !(safety_mask & (1U<<(i)))) {
                // safety is on, overwride pwm
                period_us = safe_pwm[i];
            }
            write_int(i, period_us);
        }
    }

    _corked = false;
}

void RCOutput::timer_tick(void)
{
    safety_update();
}

void RCOutput::write_int(uint8_t chan, uint16_t period_us)
{
    if (!_initialized || chan >= MAX_CHANNELS) {
        return;
    }

    bool safety_on = hal.util->safety_switch_state() == AP_HAL::Util::SAFETY_DISARMED;
    if (safety_on && !(safety_mask & (1U<<(chan)))) {
        // safety is on, overwride pwm
        period_us = safe_pwm[chan];
    }

    pwm_out &out = pwm_out_list[chan];
    out.value = period_us;

    switch(out.group->current_mode) {
    case MODE_PWM_BRUSHED: {
        float duty = 0;
        if (period_us <= _esc_pwm_min) {
            duty = 0;
        } else if (period_us >= _esc_pwm_max) {
            duty = 1;
        } else {
            duty = ((float)(period_us - _esc_pwm_min))/(_esc_pwm_max - _esc_pwm_min);
        }

        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(out.h_cmpr,
            duty*BRUSH_TIMEBASE_RESOLUTION_HZ/out.group->rc_frequency));
        break;
    }

    case MODE_PWM_NONE:
    default:
        period_us = 0;
    // fallthrough
    case MODE_PWM_NORMAL:
        if (period_us > SERVO_TIMEBASE_PERIOD) {
            period_us = SERVO_TIMEBASE_PERIOD;
            out.value = SERVO_TIMEBASE_PERIOD;
        }
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(out.h_cmpr, period_us));
        break;
    }
}

/*
  get safety switch state for Util.cpp
 */
AP_HAL::Util::safety_state RCOutput::_safety_switch_state(void)
{
    if (!hal.util->was_watchdog_reset()) {
        hal.util->persistent_data.safety_state = safety_state;
    }
    return safety_state;
}

/*
  force the safety switch on, disabling PWM output from the IO board
*/
bool RCOutput::force_safety_on(void)
{
    safety_state = AP_HAL::Util::SAFETY_DISARMED;
    return true;
}

/*
  force the safety switch off, enabling PWM output from the IO board
*/
void RCOutput::force_safety_off(void)
{
    safety_state = AP_HAL::Util::SAFETY_ARMED;
}

/*
  set PWM to send to a set of channels when the safety switch is
  in the safe state
*/
void RCOutput::set_safety_pwm(uint32_t chmask, uint16_t period_us)
{
    for (uint8_t i=0; i<ARRAY_SIZE(safe_pwm); i++) {
        if (chmask & (1U<<i)) {
            safe_pwm[i] = period_us;
        }
    }
}

/*
  update safety state
 */
void RCOutput::safety_update(void)
{
    uint32_t now = AP_HAL::millis();
    if (now - safety_update_ms < 100) {
        // update safety at 10Hz
        return;
    }
    safety_update_ms = now;

    AP_BoardConfig *boardconfig = AP_BoardConfig::get_singleton();

    if (boardconfig) {
        // remember mask of channels to allow with safety on
        safety_mask = boardconfig->get_safety_mask();
    }

#ifdef HAL_GPIO_PIN_SAFETY_IN
    gpio_set_direction((gpio_num_t)HAL_GPIO_PIN_SAFETY_IN, GPIO_MODE_INPUT);
    gpio_set_pull_mode((gpio_num_t)HAL_GPIO_PIN_SAFETY_IN, GPIO_PULLDOWN_ONLY);
    bool safety_pressed = gpio_get_level((gpio_num_t)HAL_GPIO_PIN_SAFETY_IN);
    if (safety_pressed) {
        AP_BoardConfig *brdconfig = AP_BoardConfig::get_singleton();
        if (safety_press_count < UINT8_MAX) {
            safety_press_count++;
        }
        if (brdconfig && brdconfig->safety_button_handle_pressed(safety_press_count)) {
            if (safety_state ==AP_HAL::Util::SAFETY_ARMED) {
                safety_state = AP_HAL::Util::SAFETY_DISARMED;
            } else {
                safety_state = AP_HAL::Util::SAFETY_ARMED;
            }
        }
    } else {
        safety_press_count = 0;
    }
#endif

#ifdef HAL_GPIO_PIN_LED_SAFETY
    led_counter = (led_counter+1) % 16;
    const uint16_t led_pattern = safety_state==AP_HAL::Util::SAFETY_DISARMED?0x5500:0xFFFF;
    gpio_set_level((gpio_num_t)HAL_GPIO_PIN_LED_SAFETY, (led_pattern & (1U << led_counter))?0:1);
#endif
}

/*
  set PWM to send to a set of channels if the FMU firmware dies
*/
void RCOutput::set_failsafe_pwm(uint32_t chmask, uint16_t period_us)
{
    //RIP (not the pointer)
}
