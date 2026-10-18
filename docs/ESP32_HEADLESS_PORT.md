# Running on a Regular ESP32 (Headless)

**Short answer: yes.** The control core (HX711, grind state machine, predictive flow, pulse correction, diagnostics, BLE, preferences, logging) is plain Arduino/FreeRTOS code and has no hard S3 dependency. Only the display/touch/LVGL layer is board-specific, and a headless build simply drops it.

The original ESP32 is dual-core (240 MHz Xtensa LX6), so the existing Core 0 (sampling + grind control) / Core 1 (BLE, file I/O) task split carries over unchanged.

## What changes

| Area | ESP32-S3 (today) | Regular ESP32 |
|---|---|---|
| Display / touch / LVGL | AMOLED + FT3168 | **Remove** (`display_manager`, `touch_driver`, `src/ui/`, `src/font/`, LVGL + GFX deps) |
| User input | Touchscreen | BLE and/or Wi-Fi web UI, optional physical start button |
| PSRAM | 8 MB OPI | None on WROOM-32 → see [Memory](#memory) |
| Flash | 16 MB, delta-OTA partitions | Typically 4 MB → new partition table, see [Flash & OTA](#flash--ota) |
| Console | USB-CDC (`ARDUINO_USB_CDC_ON_BOOT`) | UART via USB-serial chip; drop that flag |
| GPIOs | `hardware.h` pins (2, 3, 18, …) | Re-map, see [Pins](#pins) |

## Proposed approach

1. **New PlatformIO env** `esp32-headless` (`board = esp32dev`, `-DHEADLESS_BUILD`, no LVGL/GFX/`BOARD_HAS_PSRAM`, own partition CSV).
2. **Compile out the UI** behind `HEADLESS_BUILD`: `main.cpp`, `task_manager.cpp` (UI render task) and `hardware_manager` are the only non-UI files referencing `UIManager`/`DisplayManager`.
3. **Replace UI-originated events** with a small command interface that feeds the same `StateMachine`/`GrindController` entry points the UI uses today:
   - set mode (weight/time), target weight/time, purge/prime settings
   - start / stop grind, acknowledge timeout/completion
   - tare, calibrate (known-weight value), motor test
4. **Expose the interface** (pick one or both):
   - **BLE (recommended first step)**: add one writable "grinder control" characteristic + a notify characteristic for live weight/state next to the existing OTA/data/sysinfo services. No extra radio load, works with the existing `tools/grinder.py`, and a Web Bluetooth page or phone app can drive it.
   - **Wi-Fi web UI**: small embedded page (target grams, start/stop, live weight via WebSocket/SSE) served from the ESP32 in AP or STA mode. Nicer UX, but Wi-Fi + BLE share one radio on the ESP32 — keep BLE auto-disable (already present) or disable BLE while Wi-Fi is active.
5. **Optional hardware inputs**: one button (start/stop) and/or a status LED. The existing *Start on Cup* automation already allows touch-free operation.
6. **Persist** the new settings through the existing `Preferences` keys (`grind_mode`, `chute_mode`, `chute_amount_g`, target weight/time); no storage changes needed.

## Mobile App: Web Bluetooth SPA

A static single-page app using the [Web Bluetooth API](https://developer.mozilla.org/docs/Web/API/Web_Bluetooth_API) can talk to the ESP32 directly over the BLE control/notify characteristics above. No backend, no app store.

- **Hosting**: any static HTTPS host (e.g. GitHub Pages) — Web Bluetooth requires a secure context. Add a manifest + service worker to make it an installable, offline-capable PWA.
- **UI**: target grams, mode, start/stop, tare, live weight and state, calibration wizard, warnings from diagnostics. Plain TypeScript or a light framework (Svelte/Preact) is plenty.
- **Platform support**: Chrome/Edge on Android, Windows, macOS, Linux and ChromeOS work out of the box. **iOS Safari does not support Web Bluetooth**; use a Web Bluetooth-capable browser such as Bluefy, or wrap the same SPA with [Capacitor](https://capacitorjs.com/) + `@capacitor-community/bluetooth-le` for a native iOS/Android build.
- **Protocol hygiene**: reuse the existing service-UUID layout, keep payloads small (JSON or a packed struct) and notification rate modest (~10 Hz matches the HX711), and version the protocol via a characteristic so the SPA can detect firmware mismatches.
- **Bonus**: the existing OTA characteristics could later allow firmware updates from the same app.

## Memory

- The grind logger allocates its staging buffers with `MALLOC_CAP_SPIRAM` (`grind_logging.cpp`). Without PSRAM that allocation fails → fall back to `MALLOC_CAP_8BIT`.
- Size at current settings: `60 s × 50 Hz × 24 B ≈ 72 KB` of measurements (+ ~2 KB events). Too much next to the BLE/Wi-Fi stacks in ~320 KB of usable DRAM. Mitigation: raise `SYS_LOG_EVERY_N_GRIND_LOOPS` to 5 (≈ 15 KB) and/or use `GRIND_TIMEOUT_SEC` 30.
- Dropping LVGL and the display framebuffer frees far more than the logger needs, so the net heap budget is comfortable.
- Prefer NimBLE over the Arduino Bluedroid `BLEDevice` stack to save ~50–80 KB if heap gets tight.

## Flash & OTA

- `partitions.csv` assumes 16 MB (factory + 2 OTA slots at 3 MB + 2 MB patch). A 4 MB ESP32 needs e.g. the stock `min_spiffs` layout (2 × ~1.9 MB OTA + small LittleFS).
- Delta-OTA needs a patch partition and a factory slot, so it does **not** fit in 4 MB. Use plain (compressed) BLE OTA there, or buy a 8/16 MB module to keep delta OTA.
- Headless firmware is much smaller than the LVGL build, so full-image OTA is realistic.

## Pins

Avoid on a classic ESP32: GPIO 6–11 (flash), 34–39 (input-only, fine for HX711 DOUT but not outputs), and be careful with strapping pins 0, 2, 5, 12, 15 for the relay (must not be driven at boot).

Suggested starting point:

| Signal | GPIO |
|---|---|
| HX711 DOUT | 32 |
| HX711 SCK | 33 |
| Motor relay | 26 |
| Start button (optional) | 27 |

Put these in a board-specific section of `src/config/hardware.h` selected via the env flag.

## Risks / things to verify

- **HX711 timing**: bit-banged clock must stay glitch-free under Wi-Fi/BLE load → keep sampling pinned to Core 0 and run the radio stacks on Core 1 (already the layout).
- **Relay safety at boot/reset**: ensure the relay pin defaults LOW with a pull-down.
- **Wi-Fi + BLE coexistence** if both are enabled.
- **No on-device feedback** for errors (timeout, calibration, noise warnings) → surface them over BLE/web and with an LED/buzzer.
- **Calibration** must be possible without a screen (BLE command with known weight).

## Effort estimate

Small-to-moderate: a build env, an `#ifdef` pass over the UI references, a command/notify characteristic (or web page), pin remap, partition table, and the logger PSRAM fallback. No changes to the grind algorithms.
