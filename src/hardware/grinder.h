#pragma once
#include <Arduino.h>
#include <driver/rmt_tx.h>
#include <driver/rmt_encoder.h>
#include <functional>
#include "../config/constants.h"

// Forward declarations
struct GrindEventData;
enum class UIGrindEvent;

class Grinder {
private:
    int motor_pin;
    bool grinding;
    bool initialized;

    // RMT pulse control
    // Each RMT symbol carries two half-periods of at most 32767 ticks (1 tick = 1us).
    static constexpr uint32_t RMT_MAX_HALF_PERIOD_US = 32767;
    // Keep the whole pulse within one RMT memory block (64 symbols incl. driver EOF marker)
    // so the copy encoder never has to refill from the buffer during transmission.
    static constexpr size_t RMT_MAX_PULSE_SYMBOLS = 48;
    static constexpr uint32_t RMT_MAX_PULSE_DURATION_US =
        (RMT_MAX_PULSE_SYMBOLS - 1) * 2 * RMT_MAX_HALF_PERIOD_US;

    rmt_channel_handle_t rmt_channel;
    rmt_encoder_handle_t copy_encoder;
    rmt_symbol_word_t continuous_symbol;                     // Must outlive the transmission
    rmt_symbol_word_t pulse_symbols[RMT_MAX_PULSE_SYMBOLS];  // Must outlive the transmission
    volatile bool pulse_active;
    volatile bool pulse_tx_done;   // Set from the RMT ISR when a finite pulse has been fully emitted
    bool rmt_initialized;

    static bool IRAM_ATTR on_rmt_tx_done(rmt_channel_handle_t channel,
                                         const rmt_tx_done_event_data_t* event_data,
                                         void* user_ctx);
    void abort_transmission();
    size_t build_pulse_symbols(uint32_t duration_us);

    // Motor settling tracking
    unsigned long motor_start_time;

    // Background indicator state (always compiled in)
    bool background_active;
    std::function<void(const GrindEventData&)> ui_event_callback;

    void emit_background_change(bool active);

public:
    void init(int pin);
    void start();
    void stop();
    
    // RMT-based precise pulse control
    void start_pulse_rmt(uint32_t duration_ms);
    bool is_pulse_complete();
    
    bool is_grinding() const { return grinding; }
    bool is_initialized() const { return initialized; }
    bool is_motor_settled() const;

    // Background indicator setup (always compiled in)
    void set_ui_event_callback(const std::function<void(const GrindEventData&)>& callback);
};
