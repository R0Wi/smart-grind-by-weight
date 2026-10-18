# Running on a Regular ESP32 (Headless, BLE-only)

**Scope:** classic ESP32 (WROOM-32, 4 MB flash, no PSRAM), no display, controlled entirely over BLE, firmware updated over BLE with full images. Wi-Fi/web UI is out of scope for now (see [Later](#later)).

The control core (HX711, grind state machine, predictive flow, pulse correction, diagnostics, BLE, preferences) is plain Arduino/FreeRTOS code and has no hard S3 dependency. The original ESP32 is dual-core (240 MHz), so the existing Core 0 (sampling + grind control) / Core 1 (BLE, file I/O, event pump) split carries over unchanged.

## 1. Code structure: interfaces with null implementations

Instead of `#ifdef` sprinkled through the code, each UI/board-specific seam gets a small interface. The S3 build wires the real implementation, the headless build wires a no-op (or BLE) one. The only per-target code is a tiny composition root (`platform_s3.cpp` / `platform_esp32.cpp`) selected by `build_src_filter` per PlatformIO env; `src/ui/`, `src/font/`, `display_manager`, `touch_driver` and the LVGL/GFX deps are simply not compiled in the headless env.

| Seam | Today | Interface | Headless impl |
|---|---|---|---|
| Display + touch | `DisplayManager` owned by `HardwareManager`; `ota_handler` reaches into `get_display()->get_touch_driver()` | `IDisplay` (`init`, `update`, `suspend_input`) | `NullDisplay` |
| User interface | `UIManager` referenced by `main.cpp`, `TaskManager`, OTA status callback | `IUserInterface` (`init`, `update`, `show_ota_status`, `show_ota_failure`) | `NullUserInterface` |
| Grind events | `GrindController` already emits `GrindEventData` through a callback + Core 0→1 queue | `IGrindEventSink` | `BleEventSink` (see [feedback channel](#4-ble-feedback-channel)) |
| Grind logging | global `grind_logger` called from `GrindController`, tasks, BLE | `IGrindLogSink` | `NullLogSink` / `BleLogSink` (see [Memory](#7-memory-and-the-logger)) |
| Firmware writer | `OTAHandler` writes to a `patch` partition and applies a delta | `IFirmwareWriter` | `FullImageWriter` (direct to inactive OTA slot) |

Three things currently live *inside* `UIManager` and must move out so headless keeps them:

1. **Auto actions** (start on cup / return on removal): `UIManager::update_auto_actions()` → new UI-independent `AutomationController`.
2. **Diagnostics**: `DiagnosticsController` is owned by `UIManager` → create it in the composition root and feed warnings to the BLE event sink.
3. **Command handling**: the UI controllers translate touches into `GrindController`/`WeightSensor`/`Preferences` calls and `StateMachine` transitions. A new `GrinderService` facade (set target/mode, start/stop, acknowledge, tare, calibrate, purge confirm, extra pulse, settings) takes over; the BLE command characteristic calls only this facade, so a Wi-Fi API can reuse it later.

Notes:
- Keep the Core 1 "UI task" as a generic **event pump**: it is the thread that drains `GrindController::process_queued_ui_events()`, which is what feeds the `IGrindEventSink`. With `NullUserInterface` it still has to run.
- `StateMachine`/`UIState` are plain enums with no LVGL dependency and can stay as the shared app state.
- `config/theme.h` has no LVGL includes, so `constants.h` needs no changes.
- This is not a large refactor: five thin interfaces, one extraction (automation), one new facade.

## 2. Motor output: RMT on the classic ESP32

`Grinder` drives the relay through the IDF 5 RMT TX driver (`driver/rmt_tx.h`).

- **Available:** yes. Classic ESP32 has 8 RMT channels, each TX-capable with a 64-symbol memory block (S3 has only 4 TX). TX is routed through the GPIO matrix, so any **output-capable** GPIO works. Avoid 34–39 (input only), 6–11 (flash) and strapping pins 0/2/5/12/15. Suggested motor pin: **GPIO 26**. `RMT_CLK_SRC_DEFAULT` (APB, 80 MHz) at 1 MHz resolution is a valid divider; no config change.
- **Gotcha 1 (breaks pulses):** the classic ESP32 lacks `SOC_RMT_SUPPORT_TX_LOOP_COUNT`, so `rmt_transmit()` with a *finite* `loop_count > 0` returns `ESP_ERR_NOT_SUPPORTED` (the S3 supports it). `Grinder::start_pulse_rmt()` uses `loop_count` for pulses longer than 32.767 ms and ignores the return value, so those pulses would silently not fire. Infinite loop (`-1`, used for continuous grinding) is still supported.
  **Fix (works on both chips, no ifdef):** encode the pulse as N symbols instead of looping. A symbol holds up to 2 × 32767 µs, so even the longest pulse (latency + 250 ms) needs only a handful of symbols, far below the 64-symbol block. Also check the `rmt_transmit` return value.
- **Gotcha 2:** `is_pulse_complete()` polls `digitalRead(motor_pin)` on an RMT-driven pin. The RMT driver only enables the *output* path, so whether the readback is reliable depends on the pad's input buffer. Prefer `rmt_tx_register_event_callbacks()` (`on_trans_done`) to flag completion; it is chip-independent. Verify on hardware either way.
- Keep the relay input defaulting to off at reset (pull-down), and avoid pins that glitch during boot.

## 3. Pins (classic ESP32 starting point)

| Signal | GPIO | Note |
|---|---|---|
| HX711 DOUT | 32 | any input-capable pin |
| HX711 SCK | 33 | output |
| Motor relay (RMT) | 26 | output, not a strapping pin |
| Start button (optional) | 27 | with pull-up |
| Status LED (optional) | 2 or 25 | |

Board pins move into a board config header chosen by the env's build flag (the one place a preprocessor switch is acceptable). Drop `-DARDUINO_USB_CDC_ON_BOOT=1` and `-DBOARD_HAS_PSRAM`; the console is UART via the board's USB-serial chip.

## 4. BLE feedback channel

Without a screen, the BLE link *is* the UI. One new **Grinder service** (128-bit UUIDs in the existing style, defined in `config/bluetooth.h`), next to the existing OTA/data/debug/sysinfo services:

| Characteristic | Props | Content |
|---|---|---|
| `Command` | write (with response) | `[seq][opcode][payload]`: set target weight/time, set mode, start, stop, acknowledge (complete/timeout), tare, confirm purge (+keep-purge flag), extra pulse (time mode), calibrate begin/zero/known-weight, motor test, get/set setting (chute mode/amount, auto start/return, latency, …) |
| `Status` | notify | Packed struct from `GrindEventData`: protocol version, app state, grind phase, weight, target, progress %, flow rate, pulse count, motor on, cup present, warning bitmap, event counter. ~10 Hz while grinding, ~1 Hz idle, immediately on state change. Also readable, so a client can sync after (re)connect. |
| `Event` | indicate | Must-deliver discrete events: grind completed (final weight, result, pulses, duration), timeout (reason code), purge confirmation needed, extra pulse available, diagnostic raised/cleared, calibration step result, **command ACK/NACK** (echoing `seq` + error code). Carries a monotonically increasing event id; the last event stays readable for resync. |
| `Telemetry` (optional) | notify | Batched `GrindMeasurement`/`GrindEvent` structs (see [Memory](#7-memory-and-the-logger)), only while a client is subscribed. |

Design points:
- ATT-level write response only says "received"; semantic success/failure arrives on `Event` with the command `seq`.
- `BleEventSink` implements `IGrindEventSink` and does the struct packing on Core 1, never on the Core 0 control loop.
- The existing Nordic UART debug service stays for text logs.
- Headless has no way to re-enable BLE: make the auto-disable timeouts (`BLE_AUTO_DISABLE_TIMEOUT_MS`, bootup timeout) a policy that is `AlwaysOn` in this build, and restart advertising on disconnect.
- Optional local feedback (LED/buzzer) can be another `IFeedback` null-object later; not needed for BLE-only.

## 5. Firmware update over BLE (full image)

ElegantOTA is HTTP over Wi-Fi, so it does not apply. The BLE equivalent is already in the repo: the OTA GATT service (START/DATA/END/ABORT + status + build-number) and a Web Bluetooth flasher (`tools/web-flasher`). Only the sink changes:

- `FullImageWriter` (`IFirmwareWriter`) streams chunks straight into the inactive OTA slot with `esp_ota_begin/write/end` + `esp_ota_set_boot_partition` (or the Arduino `Update` class). No `patch` partition, no delta/detools.
- Optional: `esp32-flashz` (already a dependency) streams a zlib-compressed image into the OTA slot, roughly halving transfer time.
- `esp_ota_end` validates the image before the boot partition is switched, so a corrupt transfer leaves the running slot untouched. Keep the existing build-number/version check after reboot.
- Expect transfer on the order of tens of KB/s with a negotiated large MTU and write-without-response, i.e. about a minute for a ~1 MB image. Keep the existing "suspend Core 0 tasks during OTA" behavior.
- OTA is therefore **possible over Bluetooth only**; nothing needs to be dropped. First flash is via UART. Verify the headless image stays under the 1.9 MB slot (it will be far smaller than the LVGL build).
- Rollback: whether the Arduino prebuilt bootloader has app rollback enabled needs to be checked; without it there is no automatic fallback to the previous slot for an image that boots but misbehaves.

## 6. Flash layout (4 MB, no factory/patch partition)

Replace `partitions.csv` for this env with the standard dual-OTA layout:

```csv
# Name,   Type, SubType, Offset,   Size
nvs,      data, nvs,     0x9000,   0x5000
otadata,  data, ota,     0xE000,   0x2000
app0,     app,  ota_0,   0x10000,  0x1E0000
app1,     app,  ota_1,   0x1F0000, 0x1E0000
spiffs,   data, spiffs,  0x3D0000, 0x30000
```

~1.9 MB per app slot, ~192 KB LittleFS (autotune log and, if enabled, session files). Remove `components/delta` and `components/detools` from this env's `lib_extra_dirs`.

## 7. Memory and the logger

Logger buffers are the only PSRAM consumer: `60 s × 50 Hz × 24 B ≈ 72 KB` of measurements plus ~2 KB events, allocated with `MALLOC_CAP_SPIRAM`. On a no-PSRAM ESP32 that allocation returns null; `GrindLogger::init()` then fails, `GrindController` already logs "grind logging disabled" and continues, and `logging_active` stays false. So the logger is *already* effectively a no-op there, but by accident (a failed malloc) and the BLE data-export/sysinfo paths still reference it. Options, behind `IGrindLogSink`:

| Option | RAM | Pros | Cons |
|---|---|---|---|
| A. **Null sink** | 0 | Simplest, explicit, nothing to tune. Dropping LVGL/display frees far more heap than the logger ever used. | No on-device grind history, data export/analysis tooling has nothing to read. |
| B. **BLE stream** (`BleLogSink`) | ~1–2 KB batch buffer | Same packed `GrindMeasurement`/`GrindEvent` structs (schema v2), batched into notifications (50 Hz × 24 B ≈ 1.2 KB/s, fits easily). The phone/SPA stores sessions (IndexedDB) and can export the existing binary format, so `tools/grinder.py analyze` keeps working. No flash wear, unlimited history. | Needs a connected, subscribed client during the grind; samples are lost otherwise. Notification backpressure must never block Core 0 (drop-oldest queue). |
| C. **Stream to LittleFS** | ~2 KB chunk | Offline logging, reuses the existing session files and export. | Flash wear and write latency on Core 1, only ~190 KB of storage (~10 sessions), more code. |
| D. **Shrink the RAM buffer** | ~15 KB | One-line change (`SYS_LOG_EVERY_N_GRIND_LOOPS` 5; HX711 delivers 10 SPS, so most 50 Hz samples are repeats). | Keeps 15 KB resident next to the BLE stack for a feature nobody sees without a screen. |

**Recommendation:** ship **A** first (explicit null sink), add **B** right after the Status/Event channel works. It is the natural fit for a BLE-only device, and C/D can come later if offline logging is wanted. Dropping the data-export service from this build is optional; it simply reports zero sessions.

Other heap notes: Arduino's Bluedroid-based `BLEDevice` is the heaviest part of the remaining budget (NimBLE would save roughly 50–80 KB but means porting the 1.8k-line `manager.cpp`); BLE-only mode releases the classic BT memory.

## Later

- **Wi-Fi + web UI** (served from the ESP32): reuse `GrinderService` and `IGrindEventSink` as a second front end. Wi-Fi and BLE share one radio on the classic ESP32, so this needs a "BLE or Wi-Fi" policy.
- **Phone app:** extend the existing Web Bluetooth page (`tools/web-flasher`) into a controller SPA/PWA (Chrome/Edge on Android and desktop; iOS needs a Web Bluetooth browser such as Bluefy or a Capacitor wrapper). It consumes `Command`/`Status`/`Event` and handles OTA and session storage.
- **Local feedback:** LED/buzzer behind an `IFeedback` null-object.

## Checklist to verify on hardware

- Pulse timing and completion detection with the RMT multi-symbol fix; infinite-loop continuous mode on the classic ESP32.
- Relay stays off through reset/boot on the chosen pin.
- HX711 sampling stays glitch-free with BLE active (sampling pinned to Core 0, BLE on Core 1).
- Image size vs. 1.9 MB slot; free heap with BLE connected and notifying.
- BLE OTA throughput and rollback behavior.
