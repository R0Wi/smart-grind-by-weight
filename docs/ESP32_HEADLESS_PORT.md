# Running on a Regular ESP32 (Headless, BLE-only)

**Scope:** classic ESP32 (WROOM-32, 4 MB flash, no PSRAM), no display. Daily use with a **physical button** plus auto-start/auto-return; the phone (BLE) is used for setup, live feedback and firmware updates. Wi-Fi/web UI is out of scope for now (see [Later](#later)).

The control core (HX711, grind state machine, predictive flow, pulse correction, diagnostics, BLE, preferences) is plain Arduino/FreeRTOS code and has no hard S3 dependency. The original ESP32 is dual-core (240 MHz), so the existing Core 0 (sampling + grind control) / Core 1 (event pump, BLE polling, file I/O) split carries over unchanged.

## 0. Decisions

| Topic | Decision |
|---|---|
| Daily interaction | Physical button (required) + auto-start on cup / return on removal. Phone is optional for grinding. |
| BLE security | Open, same as today: no pairing, anyone in range can connect, send commands and start an OTA. Accepted risk; see the guards in [§6](#6-ble-grinder-service). |
| Relation to PR #1 | Independent. This plan does not rely on PR #1's fixes; overlaps are marked so they can be dropped from whichever PR merges second. |
| Firmware update | BLE only, using the existing detools patch protocol (no client changes). |

## 1. What the UI does today that headless must replace

Several jobs that are not "drawing" live in `src/ui/` today. With a do-nothing UI they would silently disappear:

| Job | Where today | Effect if missing |
|---|---|---|
| Acknowledge `INITIALIZING` so the grind proceeds to `SETUP` | `GrindingUIController::handle_grind_event` → `ui_acknowledge_phase_transition()` | Every grind hangs in `INITIALIZING` until the 60 s timeout |
| Drive `StateMachine` (READY → GRINDING → GRIND_COMPLETE / GRIND_TIMEOUT / PURGE_CONFIRM → READY) | `GrindingUIController`, `UIManager::switch_to_state` | App state never leaves READY |
| 60 s auto-return after complete/timeout | LVGL timers in `GrindingUIController` | Device stays in COMPLETE forever |
| Auto-start on cup / return on removal | `UIManager::update_auto_actions()` | Headline headless feature missing |
| Diagnostics updates | `DiagnosticsController` owned and ticked by `UIManager` | No warnings |
| Auto-tune state machine tick | `AutoTuneUIController::update()` → `AutoTuneController::update()` | Auto-tune never advances |
| Calibration sequence + noise check | `CalibrationUIController` (3 s min / 5 s stable / 15 s forced pass) | No way to calibrate |
| Blocking tare / calibrate | `UIOperations` via LVGL overlay timer | — |
| Motor test (1 s pulse + stats) | `MenuUIController::run_motor_test` (LVGL timer) | — |
| Settings writes (purge mode/amount, freshness, auto actions, logging, …) | `MenuUIController` writes `Preferences` directly | No way to configure |

A related trap outside `src/ui/`: **lifetime statistics are only updated inside the logger** (`GrindLogger::end_grind_session` → `statistics_manager.update_grind_session`), and the end-of-session request is only queued while `grind_logger.is_logging_active()`. Without PSRAM the logger fails to allocate, so stats are already silently dead on a no-PSRAM board. Fix in [§8](#8-memory-logger-and-statistics).

## 2. Code structure

### 2.1 New UI-independent components (`src/controllers/`, `src/system/`)

| Component | Responsibility | Runs on |
|---|---|---|
| `GrinderService` | Single entry point for every user action (button, BLE, touch UI). Accepts `Command`s through a FreeRTOS queue, validates them against the current state, executes them, reports the result. | Core 1 event pump |
| `GrindSessionCoordinator` | Consumes grind events first. Sends the `INITIALIZING` acknowledgement, owns all grind-related `StateMachine` transitions, runs the complete/timeout auto-return timers (60 s, millis-based, not LVGL) and the headless purge-confirm timeout. | Core 1 event pump |
| `AutomationController` | Auto-start / auto-return, moved verbatim from `UIManager::update_auto_actions()`; issues commands to `GrinderService`. Asks `IUserInterface::allows_auto_start()` for the UI-specific part (S3: ready screen on a profile tab; headless: always true). | Core 1 event pump |
| `CalibrationProcedure` | Calibration steps and noise check extracted from `CalibrationUIController` (same timings). Touch UI and BLE both drive it. | Core 1 event pump |
| `ButtonInput` (headless) | Debounced GPIO button, short/long press → `Command` (see [§3](#3-local-controls-button-and-led)). | Core 1 event pump |
| `StatusLed` (headless, optional) | Pattern per app state. | Core 1 event pump |

`DiagnosticsController` and `AutoTuneController` keep their code. They are created in the composition root and ticked by the event pump instead of by `UIManager`/`AutoTuneUIController`.

### 2.2 Interfaces with null implementations

| Seam | Today | Interface | S3 impl | Headless impl |
|---|---|---|---|---|
| Display + touch | `HardwareManager` owns `DisplayManager` **by value**; `hardware_manager.h` includes `display_manager.h` → `lvgl.h`, which leaks into the OTA handler, BLE manager, diagnostics and task manager. `OTAHandler::complete_ota()` calls `get_display()->get_touch_driver()->disable()`. | `IDisplay` (`init`, `update`, `set_brightness`, `suspend_input`) injected as a pointer | `DisplayManager` | `NullDisplay` |
| User interface | `UIManager` referenced by `main.cpp`, `TaskManager` (includes `ui_manager.h`), OTA status callback | `IUserInterface` (`init`, `update`, `on_state_changed`, `on_grind_event`, `show_ota_status`, `allows_auto_start`) | `UIManager` | `NullUserInterface` |
| Grind events | `GrindController::set_ui_event_callback` takes **one** function pointer | `IGrindEventSink`, with a small fan-out list in `GrindController` | coordinator → `UIManager` → `BleEventSink` | coordinator → `BleEventSink` |
| Grind logging | global `grind_logger` | `IGrindLogSink` | `GrindLogger` (PSRAM) | `NullLogSink`, later `BleLogSink` |
| Firmware writer | `OTAHandler` stores the patch in a `patch` partition, then `delta_check_and_apply` | `IFirmwareWriter` | existing patch-partition path | `StreamingPatchWriter` ([§7](#7-firmware-update-over-ble)) |

The only per-target code is a composition root (`src/platform/platform_s3.cpp` / `platform_esp32.cpp`) selected by `build_src_filter` per PlatformIO env. `src/ui/`, `src/font/`, `display_manager`, `touch_driver` and the LVGL/GFX deps are not compiled in the headless env.

`StateMachine`/`UIState` stay the shared app state, but ownership splits:
- `GrindSessionCoordinator` (via `GrinderService`) sets READY, GRINDING, GRIND_COMPLETE, GRIND_TIMEOUT, PURGE_CONFIRM, CALIBRATION, AUTOTUNING, OTA_UPDATE, OTA_UPDATE_FAILED.
- The touch UI may additionally set the screen-only states EDIT, MENU and CONFIRM, and only while no grind is active. The UI reacts to state changes in `on_state_changed()` instead of calling `transition_to` itself.

### 2.3 Threading

- BLE write callbacks run on the Bluedroid task, so they only parse the bytes and post a `Command` to `GrinderService`'s queue. Nothing executes inside the callback.
- `GrinderService::tick()` runs on the Core 1 event pump (today's "UI render" task, kept with `NullUserInterface`). Blocking scale operations (`tare()`, `calibrate()`), auto-tune ticks, settings writes (NVS) and motor test run here.
- **Grind commands cross to Core 0 through a queue.** Today the touch UI calls `stop_grind()`, `continue_from_purge()`, `return_to_idle()` etc. directly from Core 1 while `update()` runs on Core 0 (motor can be left running after STOP). `GrinderService` instead posts a `GrindCommand` into the existing, unused `ui_to_grind_queue` (`TaskManager`, `SYS_QUEUE_UI_TO_GRIND_SIZE`, currently sized for `void*`, change to `sizeof(GrindCommand)`). `GrindControlTask` drains it before each `GrindController::update()`, so every `GrindController` mutation happens on Core 0. Preference writes that belong to a command (e.g. "keep purge grinds") are done on Core 1 before the command is forwarded. *Overlap: PR #1 solves the same race with a mutex. Both can coexist; if PR #1 merges first the queue still works, and the mutex becomes redundant for these paths.*
- `WeightSensor::calibrate()` currently calls `update_async()` from Core 1 while Core 0 is sampling the HX711. It must read from the filter only. *Overlap: PR #1 item 4.*
- The grind controller emits a progress event every 20 ms into a 10-deep queue. The pump drains it at 16 ms. `BleEventSink` rate-limits notifications itself; it never blocks.

## 3. Local controls: button and LED

The button is required hardware for the headless build. It goes through `GrinderService` exactly like a BLE command. Short press < 1 s, long press ≥ 1 s (constants in `config/user.h`).

| App state | Short press | Long press |
|---|---|---|
| READY (calibrated) | Start grind (current profile, current mode) | — |
| READY (not calibrated) | Refused (LED error pattern) | — |
| GRINDING | Stop | Stop |
| PURGE_CONFIRM | Continue | Cancel grind |
| GRIND_COMPLETE, weight mode | Acknowledge (→ READY) | Acknowledge |
| GRIND_COMPLETE, time mode | Extra pulse | Acknowledge |
| GRIND_TIMEOUT | Acknowledge | Acknowledge |
| CALIBRATION / AUTOTUNING / OTA | Ignored (phone drives these) | Ignored |

Auto-start, auto-return on removal and the 60 s auto-return apply as on the S3.

**Headless purge-confirm timeout:** with no screen, the user may not notice the device waiting. After `USER_HEADLESS_PURGE_CONFIRM_TIMEOUT_MS` (60 s) in PURGE_CONFIRM without input, the coordinator cancels the grind. The motor is already off in that phase. The S3 build keeps waiting as today.

**BLE disconnect** does not change the grind: a running grind finishes on its own (weight mode stops at target, time mode at time, 60 s timeout as backstop), and the button still works.

**Status LED (optional, recommended):** READY = on, GRINDING = fast blink, PURGE_CONFIRM = slow blink, GRIND_COMPLETE = double blink, GRIND_TIMEOUT/error = triple blink, not calibrated = short flash every 2 s. Behind an `IFeedback` interface so a buzzer can be added later.

## 4. Motor output: RMT on the classic ESP32

`Grinder` drives the relay through the IDF 5 RMT TX driver (`driver/rmt_tx.h`).

- **Available:** classic ESP32 has 8 RMT channels, each TX-capable with a 64-symbol memory block. TX is routed through the GPIO matrix, so any output-capable GPIO works. `RMT_CLK_SRC_DEFAULT` (APB, 80 MHz) at 1 MHz resolution is valid; no config change.
- **Gotcha 1 (breaks pulses):** the classic ESP32 lacks `SOC_RMT_SUPPORT_TX_LOOP_COUNT`, so `rmt_transmit()` with a finite `loop_count > 0` returns `ESP_ERR_NOT_SUPPORTED`. `Grinder::start_pulse_rmt()` uses `loop_count` for pulses longer than 32.767 ms and ignores the return value, so those pulses would silently not fire. Infinite loop (`-1`, continuous grinding) is still supported.
  **Fix (both chips, no ifdef):** encode the pulse as N non-looped HIGH symbols (a symbol holds 2 × 32767 µs), keep the symbol buffer alive for the whole transmission (class member, not stack), and check the `rmt_transmit` return value. The current looped encoding is also wrong on the S3 (the remainder repeats every loop).
- **Gotcha 2:** `is_pulse_complete()` uses `digitalRead()` on an RMT-only output pin. Use `rmt_tx_register_event_callbacks()` (`on_trans_done`) instead.
- *Overlap: both fixes are PR #1 items 2 and 3. If PR #1 merges first, this section reduces to the hardware check of continuous mode on the classic ESP32.*
- Relay input must default to off at reset (pull-down; check the relay module's polarity) and the pin must not glitch during boot.

## 5. Pins and board config (classic ESP32 starting point)

| Signal | GPIO | Note |
|---|---|---|
| HX711 DOUT | 32 | input |
| HX711 SCK | 33 | output |
| Motor relay (RMT) | 26 | output, not a strapping pin |
| Button | 27 | input, internal pull-up, to GND; **required** |
| Status LED (optional) | 25 | GPIO 2 also works (on-board LED on many DevKits), but it is a strapping pin |

Board pins move into `config/board_s3.h` / `config/board_esp32.h`, selected by an env build flag inside `config/hardware.h` (the one place a preprocessor switch is acceptable; code keeps including `constants.h`). The headless env drops `-DARDUINO_USB_CDC_ON_BOOT=1` and `-DBOARD_HAS_PSRAM`; the console is UART via the board's USB-serial chip.

## 6. BLE Grinder service

The BLE link is the remote UI. Add one **Grinder service** (UUIDs in `config/bluetooth.h`, existing style) next to the OTA/data/debug/sysinfo services, in **both** builds so the phone app works with either device. All multi-byte values are little-endian, structs are packed, every payload starts with `protocol_version = 1`.

| Characteristic | Props | Content |
|---|---|---|
| `Command` | write (with response) | `[seq u8][opcode u8][payload]` |
| `Status` | read, notify | 32-byte snapshot (below). 10 Hz while a grind / calibration / auto-tune is active, 1 Hz otherwise, immediately on state or phase change. Read to resync after (re)connect. |
| `Event` | read, indicate | `[event_id u32][type u8][payload]`. Must-deliver events; `event_id` increases monotonically, the last event stays readable. |
| `Telemetry` (later) | notify | Batched `GrindMeasurement`/`GrindEvent` structs ([§8](#8-memory-logger-and-statistics)). |

Create the service with an explicit handle count (`createService(uuid, numHandles)`); the Arduino default of 15 is too small for four characteristics with CCCDs. With the existing sysinfo service already near that limit, check its count too.

### Commands

| Opcode | Name | Payload | Allowed in |
|---|---|---|---|
| 0x01 | START | `[profile u8]` (0xFF = current) | READY, calibrated |
| 0x02 | STOP | — | GRINDING, PURGE_CONFIRM |
| 0x03 | ACKNOWLEDGE | — | GRIND_COMPLETE, GRIND_TIMEOUT |
| 0x04 | PURGE_CONTINUE | `[keep_purge u8]` (1 = switch to Prime mode) | PURGE_CONFIRM |
| 0x05 | EXTRA_PULSE | — | GRIND_COMPLETE, time mode |
| 0x06 | TARE | — | READY |
| 0x10 | SELECT_PROFILE | `[index u8]` | READY |
| 0x11 | SET_PROFILE | `[index u8][weight_g f32][time_s f32]` (validated by `ProfileController`) | READY |
| 0x12 | SET_MODE | `[mode u8]` (0 = weight, 1 = time) | READY |
| 0x20 | CAL_BEGIN | — | READY → CALIBRATION |
| 0x21 | CAL_ZERO | — (empty scale) | CALIBRATION |
| 0x22 | CAL_WEIGHT | `[weight_g f32]` (reference placed) | CALIBRATION, weight detected |
| 0x23 | CAL_FINISH | — (after noise check passed) | CALIBRATION |
| 0x24 | CAL_CANCEL | — | CALIBRATION |
| 0x30 | AUTOTUNE_START | — | READY, calibrated |
| 0x31 | AUTOTUNE_CANCEL | — | AUTOTUNING |
| 0x32 | MOTOR_TEST | — (1 s pulse) | READY |
| 0x40 | GET_SETTING | `[id u8]` → `SETTING_VALUE` event | any |
| 0x41 | SET_SETTING | `[id u8][value 4 bytes]` | READY |
| 0x50 | DIAG_ACK | `[code u8]` | any |
| 0x51 | DIAG_RESET | — | READY |
| 0x60 | CLEAR_LOGS | — | READY |
| 0x61 | FACTORY_RESET | `[magic u32 = 0x54455352 "RSET"]` | READY |

Settings (`id`, type, existing preference key):

| id | Setting | Type | Key |
|---|---|---|---|
| 1 | Purge mode (0 = Prime, 1 = Purge) | u32 | `grinder_mode` |
| 2 | Purge amount g (`GRIND_PURGE_AMOUNT_MIN_G`–`MAX_G`) | f32 | `grinder_amount_g` |
| 3 | Freshness hours | f32 | `freshness_hrs` |
| 4 | Auto-start on cup | u32 (bool) | `autogrind/auto_start` |
| 5 | Return on removal | u32 (bool) | `autogrind/auto_return` |
| 6 | Session logging to flash | u32 (bool) | `logging/enabled` |
| 7 | Motor response latency ms | f32 | `motor_lat_ms` |

Display-only settings (brightness, swipe, BLE-at-startup) are not exposed. After any settings write, `AutomationController::refresh_settings()` runs, as `refresh_auto_action_settings()` does today.

### Status (32 bytes)

| Offset | Field | Type |
|---|---|---|
| 0 | protocol_version | u8 |
| 1 | app_state (`UIState`) | u8 |
| 2 | grind_phase (`GrindPhase`) | u8 |
| 3 | grind_mode | u8 |
| 4 | profile index | u8 |
| 5 | flags: bit0 motor on, bit1 calibrated, bit2 can pulse, bit3 load cell fault, bit4 grind active, bit5 calibration weight detected, bit6 calibration noise check passed | u8 |
| 6 | warning bitmap (bit n = `DiagnosticCode` n) | u16 |
| 8 | weight g | f32 |
| 12 | target weight g | f32 |
| 16 | target time ms | u32 |
| 20 | flow rate g/s | f32 |
| 24 | progress % | u8 |
| 25 | pulse count | u8 |
| 26 | reserved | u16 |
| 28 | last event_id | u32 |

### Events

| Type | Name | Payload |
|---|---|---|
| 0x01 | COMMAND_RESULT | `[seq u8][opcode u8][error u8]` |
| 0x02 | GRIND_COMPLETED | `[result u8][final_weight f32][target f32][pulses u8][duration_ms u32]` |
| 0x03 | GRIND_TIMEOUT | `[phase u8][message char[24]]` (copy of `last_error_message`) |
| 0x04 | GRIND_STOPPED | — |
| 0x05 | PURGE_CONFIRM_REQUIRED | `[hours_since_last u16]` (0xFFFF = first grind since boot) |
| 0x06 | PULSE_AVAILABLE | `[pulse_count u8]` |
| 0x07 | DIAGNOSTIC | `[code u8][active u8]` |
| 0x08 | CALIBRATION_STEP | `[step u8][ok u8][value f32]` (step: zeroed, weight applied, noise ok / noise forced, done) |
| 0x09 | AUTOTUNE_RESULT | `[success u8][latency_ms f32][previous_ms f32]` |
| 0x0A | SETTING_VALUE | `[id u8][value 4 bytes]` |

Error codes: 0 OK, 1 wrong state, 2 invalid argument, 3 unknown opcode, 4 not calibrated, 5 load cell fault, 6 malformed, 7 busy (OTA active).

### Design points

- The ATT write response only means "received"; the outcome arrives as `COMMAND_RESULT` with the same `seq`.
- `BleEventSink` packs structs on Core 1 and copies strings at once (`error_message` points into `GrindController`).
- **Always-on policy:** the auto-disable timeouts (`BLE_AUTO_DISABLE_TIMEOUT_MS`, bootup timeout) and the `bluetooth/startup` preference become a `BlePowerPolicy` that is `AlwaysOn` in the headless build. Advertising restarts on disconnect (already done in `onDisconnect`).
- **Open access guards** (no pairing, by decision): OTA START and MOTOR_TEST/AUTOTUNE are refused while a grind is active; FACTORY_RESET needs the magic value; all motor-moving commands respect the 60 s grind timeout. Anyone in range can still start a grind or flash firmware. Revisit if the device is used in a shared space.
- `update_hardware_info` must report `display_active`/`touch_active` from the actual build instead of hard-coded `true`. Add `"variant":"s3-touch"` / `"esp32-headless"` to the system info JSON so tools can tell the builds apart.
- The existing Nordic UART debug service stays for text logs.

## 7. Firmware update over BLE

The wire protocol stays as it is: OTA service START/DATA/END/ABORT + status + build number. Both clients always send a **detools patch**:
- `tools/ble/grinder-ble.py` sends a delta against the cached previous image, or for a "full" update a patch against an empty file;
- the web flasher downloads `*-web-ota.bin`, made in `release.yml` by `detools create_patch -c heatshrink empty.bin <image>`.

A raw-image writer would therefore break both clients. Instead:

- **`StreamingPatchWriter`** (`IFirmwareWriter`) applies the patch while it arrives, with the vendored detools streaming API (`detools_apply_patch_init` / `_process` / `_finalize`). Source reads come from the running partition (deltas keep working); output goes straight into the inactive OTA slot via `esp_ota_write`. No `patch` partition is needed.
- Open the slot with `esp_ota_begin(dest, OTA_WITH_SEQUENTIAL_WRITES, …)` so it erases sector by sector while writing, instead of erasing ~1.9 MB at once inside the BLE write callback.
- `esp_ota_end` validates the image (including the chip id, so an S3 image is rejected by a classic ESP32) before `esp_ota_set_boot_partition`. A corrupt or aborted transfer leaves the running slot untouched. Keep the existing build-number/version check after reboot.
- Keep `components/delta` (or only its partition helpers) and `components/detools` in the headless env. `esp32-flashz` is not needed (the heatshrink patch is already compressed: 1.56 MB patch for a 2.14 MB S3 image).
- Keep the existing "suspend Core 0 tasks during OTA" behaviour. Replace the direct touch-driver call in `complete_ota()` with `IDisplay::suspend_input()`.
- Expect tens of KB/s with a large MTU, i.e. about a minute for a full headless image, much less for a delta. First flash is via UART.
- **Release pipeline:** `release.yml` and `deploy-web-flasher.yml` must build and publish the headless image and its `-web-ota.bin` as a separate variant; the web flasher picks the variant reported by the device.
- Rollback: check whether the Arduino core's prebuilt bootloader has `CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE`. If it does, mark the app valid only after BLE is up; if not, there is no fallback for an image that boots but misbehaves.

## 8. Memory, logger and statistics

**Statistics first (both builds):** move `statistics_manager.update_grind_session(...)` out of `GrindLogger::end_grind_session`. `GrindController` always queues `END_GRIND_SESSION` at COMPLETED/TIMEOUT, not only while logging is active. It tracks motor-on time itself (summing loop intervals with `motor_is_on`) and carries final weight, error, pulse count and mode in the request. The FileIO task (Core 1) updates statistics for successful results, then calls the logger, which no longer touches statistics. This fixes stats on any board where the logger is disabled.

**Logger:** the buffers (`60 s × 50 Hz × 24 B ≈ 72 KB` plus ~2 KB events) are the only PSRAM consumer. On a no-PSRAM ESP32 the allocation fails and logging is off by accident. Behind `IGrindLogSink`:

| Option | RAM | Pros | Cons |
|---|---|---|---|
| A. **Null sink** | 0 | Explicit, nothing to tune. | No grind history; data export reports zero sessions. |
| B. **BLE stream** (`BleLogSink`) | ~1–2 KB batch buffer | Same `GrindMeasurement`/`GrindEvent` structs, batched into `Telemetry` notifications (≈1.2 KB/s). The phone stores sessions and can export the existing format, so `tools/grinder.py analyze` keeps working. No flash wear. | Needs a subscribed client during the grind; drop-oldest queue so Core 0 never blocks. |
| C. Stream to LittleFS | ~2 KB chunk | Offline history with existing export. | Flash wear, write latency, ~190 KB storage (~10 sessions), more code. |
| D. Smaller RAM buffer | ~15 KB | One-line change (`SYS_LOG_EVERY_N_GRIND_LOOPS` 5). | 15 KB resident for a feature nobody sees without a client. |

**Recommendation:** A first, B after the Grinder service works.

Other heap notes: the Bluedroid-based Arduino `BLEDevice` is the largest remaining consumer. NimBLE would save roughly 50–80 KB RAM; check whether the Arduino core's BLE library can run on NimBLE (configuration change) before considering a port of the 1.8k-line `manager.cpp`. BLE-only mode releases the classic BT memory.

## 9. Flash layout (4 MB, no factory/patch partition)

```csv
# Name,   Type, SubType, Offset,   Size
nvs,      data, nvs,     0x9000,   0x5000
otadata,  data, ota,     0xE000,   0x2000
app0,     app,  ota_0,   0x10000,  0x1E0000
app1,     app,  ota_1,   0x1F0000, 0x1E0000
spiffs,   data, spiffs,  0x3D0000, 0x30000
```

~1.9 MB per app slot, ~192 KB LittleFS (auto-tune log and, if enabled, session files). File: `partitions_esp32_4mb.csv`, referenced from the headless env.

## 10. Implementation order

Each step builds and runs on its own.

| Step | Content | Done when |
|---|---|---|
| **A. Seams on the S3** | `GrinderService` + command queue to Core 0, `GrindSessionCoordinator`, event fan-out, `AutomationController`, `CalibrationProcedure`, diagnostics/auto-tune ticked by the pump, statistics decoupled from the logger, `IDisplay` injected into `HardwareManager`, `IUserInterface`. Touch UI calls the service only. | S3 behaves exactly as before (grind, purge, time pulse, auto actions, calibration, auto-tune, motor test, OTA); stats update with logging disabled. |
| **B. Headless env** | `env:esp32-headless`, `platform_esp32.cpp`, board config, partitions, `NullDisplay`/`NullUserInterface`, `ButtonInput`, `StatusLed`, always-on BLE, `StreamingPatchWriter`, RMT fix (unless PR #1 has landed). | Button-driven grind works on a classic ESP32; OTA via `grinder.py upload` and the web flasher. |
| **C. Grinder BLE service** | `Command`/`Status`/`Event`, `BleEventSink`, both builds; a minimal client in `tools/ble/`. | Every command in [§6](#6-ble-grinder-service) works from the Python client on both builds. |
| **D. Telemetry** | `BleLogSink` + client-side session storage/export. | `tools/grinder.py analyze` works on data streamed from a headless device. |

## Later

- **Wi-Fi + web UI** (served from the ESP32): reuse `GrinderService` and `IGrindEventSink` as a second front end. Wi-Fi and BLE share one radio on the classic ESP32, so this needs a "BLE or Wi-Fi" policy.
- **Phone app:** extend `tools/web-flasher` into a controller SPA/PWA (Chrome/Edge on Android and desktop; iOS needs a Web Bluetooth browser such as Bluefy or a Capacitor wrapper) consuming `Command`/`Status`/`Event`, OTA and telemetry.
- **BLE pairing** if the open-access decision is revisited.

## Checklist to verify on hardware

- Pulse timing and completion with the multi-symbol RMT encoding; continuous (infinite-loop) mode on the classic ESP32.
- Relay stays off through reset/boot on the chosen pin.
- HX711 sampling stays glitch-free with BLE active (the bit-bang read already runs with interrupts disabled).
- Button debounce; no grind start from boot glitches.
- Image size vs. 1.9 MB slot; free heap with BLE connected and notifying.
- BLE OTA throughput (full and delta), abort mid-transfer, S3 image rejected, rollback behaviour.
