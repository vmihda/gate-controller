#include "relay_out.h"

#include <atomic>
#include <cstddef>

#include <driver/gpio.h>
#include <driver/rmt_tx.h>
#include <esp_check.h>
#include <esp_log.h>
#include <esp_private/periph_ctrl.h>
#include <esp_task_wdt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <hal/rmt_ll.h>
#include <hal/wdt_hal.h>
#include <soc/rtc.h>
#include <soc/soc_caps.h>

#include "board_config.h"
#include "clock_ms.h"
#include "gate_config.h"
#include "pulse_spacing.h"

namespace {

const char *const TAG = "relay_out";

// RMT timing. The pulse is a single RMT symbol {level0 = 1, duration0 = N ticks, level1 = 0,
// duration1 = 0}: the zero duration ends the transmission, and the driver adds one more end marker
// word. Two words in all, against SOC_RMT_MEM_WORDS_PER_CHANNEL = 48 words of channel memory, so the
// whole pulse is written before the transmission starts: no refill interrupt, no DMA.
//   40 kHz resolution = 25 us per tick; the default 80 MHz RMT clock divides to it exactly (8 x 250).
//   500 ms = 500 * 40 = 20000 ticks, below the 15-bit duration limit of 32767.
//   The longest pulse one symbol half can hold is 32767 / 40 = 819 ms, so even a corrupted duration
//   cannot make one transmission longer than 819 ms.
constexpr uint32_t rmt_resolution_hz = 40'000;
constexpr uint32_t ticks_per_ms = rmt_resolution_hz / 1000;
constexpr uint32_t max_duration_ticks = 0x7FFF;
constexpr uint32_t max_pulse_ms = max_duration_ticks / ticks_per_ms;
constexpr size_t pulse_words = 2;  // the pulse symbol and the driver's end marker
static_assert(GATE_PULSE_MS <= max_pulse_ms, "the pulse must fit one RMT symbol half");
static_assert(pulse_words <= SOC_RMT_MEM_WORDS_PER_CHANNEL, "the pulse must fit the channel memory");

// RTC watchdog backstop: longer than the 1 s task watchdog, so a working task watchdog path always
// resets first with its log; within the 1.5 s bound from a pulse start otherwise.
constexpr uint32_t rtc_wdt_timeout_ms = 1200;

rmt_channel_handle_t s_channel = nullptr;
rmt_encoder_handle_t s_encoder = nullptr;
rmt_symbol_word_t s_pulse_symbol{};  // read by the driver during the transmission
std::atomic<bool> s_pulse_active{false};
// Real start of the last accepted pulse. Touched only by the caller that won s_pulse_active, so by one
// caller at a time; the done interrupt never reads it.
pulse_spacing_t s_spacing{};
wdt_hal_context_t s_rtc_wdt = RWDT_HAL_CONTEXT_DEFAULT();

bool on_pulse_done(rmt_channel_handle_t, const rmt_tx_done_event_data_t *, void *)
{
    s_pulse_active.store(false);
    return false;  // no task was woken
}

esp_err_t drive_relay_pin_low()
{
    // Output latch first, then the output driver, so D1 never drives high. No pull-up ever, and no
    // gpio_reset_pin: it enables the pull-up.
    ESP_RETURN_ON_ERROR(gpio_set_level(board::relay_pin, 0), TAG, "set D1 low");
    const gpio_config_t config = {
        .pin_bit_mask = 1ULL << board::relay_pin,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    return gpio_config(&config);
}

// A software restart (esp_restart, a panic, the task or interrupt watchdog) does not reset the RMT:
// esp_system_reset_modules_on_exit() leaves it out on ESP32-S3 and ESP32-C6. A pulse that was
// running goes on through the reboot, and once rmt_new_tx_channel() connects D1 to the RMT again,
// the rest of it would close the contact a second time: Nice would take that as another step.
// Two guards, both while D1 is a plain GPIO output driven low:
// - the RMT peripheral is reset (Wokwi does not model this reset and resumes the old pulse);
// - D1 is handed to the RMT only rmt_handover_delay_ms after that. A leftover pulse started before
//   this boot and ends at most max_pulse_ms (819 ms) after its start, 500 ms in practice, so by then
//   it can only have driven an RMT output that was not routed to D1. The delay counts from here, not
//   from the boot, so it does not depend on the restart resetting the systimer behind esp_timer
//   (ESP-IDF resets it, Wokwi does not).
//   On ESP32-S3 the restart drops APB to the XTAL clock until the app restores the PLL, so the tail of
//   a leftover pulse runs at half rate and the contact can stay closed longer than 819 ms, by about the
//   ROM and bootloader time; that is still well under 1.5 s, and the handover counts from here, after
//   the boot, so it still comes after the tail.
static_assert(RMT_LL_GET(INST_NUM) == 1, "one RMT group, id 0");
constexpr int rmt_group = 0;
constexpr uint64_t rmt_handover_delay_ms = 1000;
static_assert(rmt_handover_delay_ms > max_pulse_ms, "a leftover pulse must be over before the handover");

void stop_leftover_rmt_pulse()
{
    PERIPH_RCC_ATOMIC() {
        rmt_ll_enable_bus_clock(rmt_group, true);
        rmt_ll_reset_register(rmt_group);
    }
}

void wait_for_rmt_handover()
{
    const uint64_t since_ms = uptime_ms();
    while (uptime_ms() - since_ms < rmt_handover_delay_ms) {
        vTaskDelay(1);
    }
}

esp_err_t start_rmt_channel()
{
    rmt_tx_channel_config_t channel_config = {};
    channel_config.gpio_num = board::relay_pin;
    channel_config.clk_src = RMT_CLK_SRC_DEFAULT;
    channel_config.resolution_hz = rmt_resolution_hz;
    channel_config.mem_block_symbols = SOC_RMT_MEM_WORDS_PER_CHANNEL;
    channel_config.trans_queue_depth = 1;
    channel_config.flags.init_level = 0;  // idle low before the first pulse
    ESP_RETURN_ON_ERROR(rmt_new_tx_channel(&channel_config, &s_channel), TAG, "RMT channel");

    const rmt_copy_encoder_config_t encoder_config = {};
    ESP_RETURN_ON_ERROR(rmt_new_copy_encoder(&encoder_config, &s_encoder), TAG, "RMT encoder");

    const rmt_tx_event_callbacks_t callbacks = {.on_trans_done = on_pulse_done};
    ESP_RETURN_ON_ERROR(rmt_tx_register_event_callbacks(s_channel, &callbacks, nullptr), TAG, "RMT callback");
    return rmt_enable(s_channel);
}

}  // namespace

esp_err_t relay_out_init()
{
    pulse_spacing_init(&s_spacing);
    ESP_RETURN_ON_ERROR(drive_relay_pin_low(), TAG, "D1 output");
    stop_leftover_rmt_pulse();
    wait_for_rmt_handover();  // only then may D1 go to the RMT
    return start_rmt_channel();
}

esp_err_t relay_out_watch_this_task()
{
    ESP_RETURN_ON_ERROR(esp_task_wdt_add(nullptr), TAG, "task watchdog");

    // ESP-IDF stops the bootloader's RTC watchdog before app_main; arm it again here, where its feeder
    // starts. Same sequence as the ESP-IDF panic handler uses for this watchdog.
    const uint64_t ticks = uint64_t{rtc_wdt_timeout_ms} * rtc_clk_slow_freq_get_hz() / 1000;
    wdt_hal_init(&s_rtc_wdt, WDT_RWDT, 0, false);
    wdt_hal_write_protect_disable(&s_rtc_wdt);
    wdt_hal_config_stage(&s_rtc_wdt, WDT_STAGE0, static_cast<uint32_t>(ticks), WDT_STAGE_ACTION_RESET_RTC);
    wdt_hal_enable(&s_rtc_wdt);
    wdt_hal_write_protect_enable(&s_rtc_wdt);
    return ESP_OK;
}

void relay_out_feed_watchdogs()
{
    esp_task_wdt_reset();
    wdt_hal_write_protect_disable(&s_rtc_wdt);
    wdt_hal_feed(&s_rtc_wdt);
    wdt_hal_write_protect_enable(&s_rtc_wdt);
}

bool relay_out_pulse(uint32_t ms)
{
    if (s_channel == nullptr || ms == 0 || ms > max_pulse_ms) {
        return false;
    }
    if (s_pulse_active.exchange(true)) {
        return false;
    }
    s_pulse_symbol.level0 = 1;
    s_pulse_symbol.duration0 = ms * ticks_per_ms;
    s_pulse_symbol.level1 = 0;
    s_pulse_symbol.duration1 = 0;

    rmt_transmit_config_t transmit_config = {};
    transmit_config.loop_count = 0;
    transmit_config.flags.eot_level = 0;  // low after the pulse
    transmit_config.flags.queue_nonblocking = 1;

    // The controller spaces pulses by its step time, which can be earlier than the real start: a stall of
    // the calling task between the step and this call would shorten the pause after the previous pulse.
    // Spacing is held here on real starts instead: the check reads the clock right before the start and
    // the record right after it, so the check is never later and the record never earlier than the real
    // start. A refused pulse stays due in the controller and is asked for again.
    if (!pulse_spacing_allows(&s_spacing, clock_ms())) {
        s_pulse_active.store(false);
        // Debug level: the controller retries every step until the spacing allows it, so this can repeat.
        ESP_LOGD(TAG, "pulse refused: less than %u ms after the start of the previous one", GATE_PULSE_SPACING_MS);
        return false;
    }
    const esp_err_t err = rmt_transmit(s_channel, s_encoder, &s_pulse_symbol, sizeof(s_pulse_symbol), &transmit_config);
    if (err != ESP_OK) {
        s_pulse_active.store(false);
        ESP_LOGE(TAG, "pulse refused: %s", esp_err_to_name(err));
        return false;
    }
    pulse_spacing_started(&s_spacing, clock_ms());
    return true;
}
