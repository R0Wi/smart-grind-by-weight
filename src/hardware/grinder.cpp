#include "grinder.h"
#include "../controllers/grind_events.h"
#include "../config/constants.h"
#include <algorithm>
#if DEBUG_ENABLE_LOADCELL_MOCK
#include "mock_hx711_driver.h"
#endif

void Grinder::init(int pin) {
    motor_pin = pin;
    grinding = false;
    pulse_active = false;
    pulse_tx_done = false;
    rmt_initialized = false;
    initialized = false;
    rmt_channel = nullptr;
    copy_encoder = nullptr;
    motor_start_time = 0;

    // Initialize background indicator
    background_active = false;
    ui_event_callback = nullptr;

#if DEBUG_ENABLE_LOADCELL_MOCK
    initialized = true;
    return;
#endif
    
    // Initialize RMT for all motor control (both continuous and pulse)
    rmt_tx_channel_config_t tx_chan_config = {
        .gpio_num = (gpio_num_t)motor_pin,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = 1000000, // 1MHz resolution = 1µs per tick
        .mem_block_symbols = 64,
        .trans_queue_depth = 4,
    };
    
    if (rmt_new_tx_channel(&tx_chan_config, &rmt_channel) != ESP_OK) {
        LOG_BLE("ERROR: Grinder RMT channel creation failed\n");
        return;
    }

    // Completion of finite pulses is reported by the RMT driver; callbacks must be
    // registered while the channel is still in the init state (before rmt_enable).
    rmt_tx_event_callbacks_t callbacks = {};
    callbacks.on_trans_done = &Grinder::on_rmt_tx_done;
    rmt_copy_encoder_config_t encoder_config = {};
    if (rmt_tx_register_event_callbacks(rmt_channel, &callbacks, this) != ESP_OK ||
        rmt_new_copy_encoder(&encoder_config, &copy_encoder) != ESP_OK ||
        rmt_enable(rmt_channel) != ESP_OK) {
        LOG_BLE("ERROR: Grinder RMT setup failed\n");
        return;
    }

    rmt_initialized = true;
    initialized = true;
}

bool IRAM_ATTR Grinder::on_rmt_tx_done(rmt_channel_handle_t, const rmt_tx_done_event_data_t*, void* user_ctx) {
    auto* self = static_cast<Grinder*>(user_ctx);
    if (self) {
        self->pulse_tx_done = true;
    }
    return false;  // No higher-priority task woken
}

void Grinder::abort_transmission() {
    // Disabling the channel aborts any in-flight transaction (continuous loop or pulse)
    // and drives the output back to its idle (LOW) level.
    rmt_disable(rmt_channel);
    rmt_enable(rmt_channel);
}

size_t Grinder::build_pulse_symbols(uint32_t duration_us) {
    if (duration_us > RMT_MAX_PULSE_DURATION_US) {
        LOG_BLE("[Grinder] Pulse %lums exceeds RMT limit, clamping to %lums\n",
                (unsigned long)(duration_us / 1000), (unsigned long)(RMT_MAX_PULSE_DURATION_US / 1000));
        duration_us = RMT_MAX_PULSE_DURATION_US;
    }

    // Emit exactly duration_us of HIGH by packing it into consecutive half-periods,
    // then finish with a short LOW so the line ends de-energised.
    size_t count = 0;
    uint32_t remaining_us = duration_us;
    while (remaining_us > 0) {
        rmt_symbol_word_t& symbol = pulse_symbols[count++];
        uint32_t first_half = std::min(remaining_us, RMT_MAX_HALF_PERIOD_US);
        remaining_us -= first_half;
        symbol.level0 = 1;
        symbol.duration0 = first_half;

        if (remaining_us > 0) {
            uint32_t second_half = std::min(remaining_us, RMT_MAX_HALF_PERIOD_US);
            remaining_us -= second_half;
            symbol.level1 = 1;
            symbol.duration1 = second_half;
        } else {
            symbol.level1 = 0;
            symbol.duration1 = 1;
            return count;
        }
    }

    // HIGH time ended on a full symbol (or duration was 0): append the LOW terminator
    rmt_symbol_word_t& tail = pulse_symbols[count++];
    tail.level0 = 0;
    tail.duration0 = 1;
    tail.level1 = 0;
    tail.duration1 = 1;
    return count;
}

void Grinder::start() {
#if DEBUG_ENABLE_LOADCELL_MOCK
    if (!initialized) return;
    MockHX711Driver::notify_grinder_start();
    pulse_active = false;
    grinding = true;
    motor_start_time = millis();
    emit_background_change(true);
    return;
#endif
    if (!initialized || !rmt_initialized) return;

    // Abort any pulse still in flight before switching to continuous mode
    if (grinding || pulse_active) {
        abort_transmission();
    }
    pulse_active = false;
    motor_start_time = millis();

    // Use RMT infinite loop for continuous grinding
    continuous_symbol.duration0 = RMT_MAX_HALF_PERIOD_US;
    continuous_symbol.level0 = 1; // HIGH
    continuous_symbol.duration1 = RMT_MAX_HALF_PERIOD_US;
    continuous_symbol.level1 = 1; // HIGH

    rmt_transmit_config_t tx_config = {
        .loop_count = -1, // Infinite loop
    };

    if (rmt_transmit(rmt_channel, copy_encoder, &continuous_symbol, sizeof(continuous_symbol), &tx_config) != ESP_OK) {
        LOG_BLE("ERROR: Grinder continuous start failed\n");
        return;
    }
    grinding = true;
    emit_background_change(true);
}

void Grinder::stop() {
#if DEBUG_ENABLE_LOADCELL_MOCK
    if (!initialized) return;
    MockHX711Driver::notify_grinder_stop();
    grinding = false;
    pulse_active = false;
    emit_background_change(false);
    return;
#endif
    if (!initialized || !rmt_initialized) return;
    
    // Stop RMT transmission (works for both infinite loop and finite pulses)
    abort_transmission();

    grinding = false;
    pulse_active = false;
    emit_background_change(false);
}

void Grinder::start_pulse_rmt(uint32_t duration_ms) {
#if DEBUG_ENABLE_LOADCELL_MOCK
    if (!initialized) return;
    MockHX711Driver::notify_pulse(duration_ms);
    pulse_active = true;
    grinding = true;
    motor_start_time = millis();
    emit_background_change(true);
    return;
#endif
    if (!initialized || !rmt_initialized) return;

    // Never queue a pulse behind a running transmission (would extend or delay it)
    if (grinding || pulse_active) {
        abort_transmission();
    }

    motor_start_time = millis();

    size_t symbol_count = build_pulse_symbols(duration_ms * 1000);
    rmt_transmit_config_t tx_config = {.loop_count = 0};

    pulse_tx_done = false;
    pulse_active = true;
    grinding = true;

    if (rmt_transmit(rmt_channel, copy_encoder, pulse_symbols,
                     symbol_count * sizeof(rmt_symbol_word_t), &tx_config) != ESP_OK) {
        LOG_BLE("ERROR: Grinder pulse start failed\n");
        pulse_active = false;
        grinding = false;
        return;
    }
    emit_background_change(true);
}

bool Grinder::is_pulse_complete() {
#if DEBUG_ENABLE_LOADCELL_MOCK
    if (!pulse_active) return true;
    if (!MockHX711Driver::is_pulse_active()) {
        pulse_active = false;
        grinding = false;
        emit_background_change(false);
        return true;
    }
    return false;
#endif
    if (!pulse_active) return true;
    
    // The RMT driver signals the end of the finite transmission from its ISR
    if (pulse_tx_done) {
        pulse_active = false;
        grinding = false;
        emit_background_change(false);
        return true;
    }
    
    return false;
}

bool Grinder::is_motor_settled() const {
    // Return true if sufficient time has passed since motor start
    if (motor_start_time == 0) {
        return false;  // Motor has never started
    }
    return (millis() - motor_start_time) >= HW_GRINDER_SETTLING_TIME_MS;
}

void Grinder::set_ui_event_callback(const std::function<void(const GrindEventData&)>& callback) {
    ui_event_callback = callback;
}

void Grinder::emit_background_change(bool active) {
    if (background_active == active) {
        return; // No change
    }
    
    background_active = active;
    
    if (ui_event_callback) {
        // Properly initialize all required fields to prevent null pointer crashes
        GrindEventData event_data = {};
        event_data.event = UIGrindEvent::BACKGROUND_CHANGE;
        event_data.phase = GrindPhase::IDLE;  // Safe default
        event_data.current_weight = 0.0f;
        event_data.progress_percent = 0;
        event_data.phase_display_text = "BACKGROUND";  // Safe string for logging
        event_data.show_taring_text = false;
        event_data.background_active = active;
        
        ui_event_callback(event_data);
        
        LOG_BLE("[Grinder] Background change: %s\n", active ? "ACTIVE" : "INACTIVE");
    }
}
