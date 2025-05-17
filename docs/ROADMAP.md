Below is an incremental roadmap that lets one developer (or a small team) bring Brunito up in logically ordered, test-driven phases.  Each phase ends with a concrete “done” condition and an integration demo so you lock in progress and avoid large, interdependent merges.

---

## Phase 0 — Environment & Skeleton

| Goal                                    | Tasks                                                                                                                                                                | Done when                                                              |
| --------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------- |
| Reproducible builds on all three boards | \* Install PlatformIO, clone repo, set up multi-env `platformio.ini` with `build_src_filter`.<br>\* Stub `main.cpp` for `navc/`, `fc/`, `gs/` that just blinks PC13. | `pio run -t upload` succeeds and each board blinks at different rates. |

---

## Phase 1 — Flight-Controller Finite-State Machine

| Goal                        | Tasks                                                                                                                                                                                                                                                                                                                                         | Done when                                                                                                        |
| --------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------- |
| Authoritative FSM in **FC** | \* Implement `State.h/.cpp` with the four states and transition guards.<br>\* Add `Heartbeat.h` to flash a unique LED pattern per state.<br>\* Create `cmdTask()` that accepts `<CMD:…>` over USB-CDC only (LoRa off for now) and routes to `State` APIs.<br>\* Unit-test state transitions on host with GoogleTest or doctest (no hardware). | Typing commands over serial reliably toggles IDLE ↔ TEST ↔ ARMED ↔ RECOVERY and LED patterns change accordingly. |

---

## Phase 2 — Command Parser & Routing

| Goal                           | Tasks                                                                                                                                                                      | Done when                                                                                 |
| ------------------------------ | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------- |
| Robust command layer in **FC** | \* Implement `CmdParser.h/.cpp` (tokenize, verify checksum, range-check params).<br>\* Integrate with Phase 1 FSM.<br>\* Emit `<CMD_ACK:OK>` or `<CMD_ACK:ERR>` responses. | Fuzz-test 1 000 random frames; no crash, all invalid frames rejected, valid frames ACKed. |

---

## Phase 3 — LoRa Link (FC ↔ GS)

| Goal                             | Tasks                                                                                                                                                                                                                                                    | Done when                                                                                                           |
| -------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------------------------------------- |
| End-to-end reliable ASCII frames | \* Bring up RadioLib on both boards with settings from `lora.h`.<br>\* Implement `lora_settings()` handshake (FC pushes, GS ACKs).<br>\* Build lightweight queue with 3 × retry + back-off.<br>\* Forward USB-CDC commands out over LoRa and vice-versa. | From GS you can send `ARM`, observe state change on FC, and receive ACK over LoRa even with USB cable disconnected. |

---

## Phase 4 — NAVC Sensor Core & Binary Packets

| Goal                        | Tasks                                                                                                                                                                                             | Done when                                                                                       |
| --------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ----------------------------------------------------------------------------------------------- |
| 100 Hz sensor fusion + UART | \* Integrate drivers: BMI088, BMP280, BMM150, GPS, DS3231.<br>\* Hard-code 100 Hz loop; pack fields into 38-byte struct; append CRC-16.<br>\* Stream over UART2, validate with PC sniffer script. | Binary stream at exactly 50 Hz (or 100 Hz raw) has <1 % packet loss during 10-minute bench run. |

---

## Phase 5 — Telemetry Pipeline

| Goal                               | Tasks                                                                                                                                                                                                                       | Done when                                                                                      |
| ---------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------- |
| Binary → ASCII → LoRa → GS console | \* FC: add `formatTelem()` and bandwidth-aware scheduler (`startTelemetry()`/`adjustTelemRate()`).<br>\* GS: parse frames and print CSV over USB-CDC.<br>\* Verify rate throttles from 20 → 4 Hz when LoRa RSSI < -110 dBm. | Live altitude graph in a quick Python script follows bench-top barometer changes in real time. |

---

## Phase 6 — SD Logging on NAVC

| Goal                     | Tasks                                                                                                                                                                       | Done when                                                                      |
| ------------------------ | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------ |
| Sustained 100 Hz logging | \* NAVC: DMA-backed ring buffer → `SD.write()` in 4 kB blocks.<br>\* Implement `startSdRecording()` RPC (UART command from FC).<br>\* Benchmark `actualHz`, return via ACK. | 32 MiB log file shows no dropped packets or write overruns after 5-minute run. |

---

## Phase 7 — Ground-Station CLI

| Goal                            | Tasks                                                                                                                                                  | Done when                                                                         |
| ------------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------ | --------------------------------------------------------------------------------- |
| Operator-friendly command shell | \* `Cli.cpp`: auto-complete, help, command history.<br>\* Wrap existing LoRa send/receive.<br>\* Add `record` sub-command to dump telemetry to PC CSV. | Operator can `record --outfile flight.csv`, arm, and watch CSV grow in real time. |

---

## Phase 8 — Integration & Soak Test

| Goal                              | Tasks                                                                                                                                                                                                             | Done when                                                        |
| --------------------------------- | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------- |
| Full system 30-minute bench cycle | \* Script drives FSM: IDLE → TEST → IDLE → ARMED (5 min) → enterRecovery() → DISARM.<br>\* Log RSSI, packet counts, SD write stats.<br>\* Inject faults (pull LoRa antenna, yank SD card) and assert `<ALERT:…>`. | Zero crashes, all alerts emitted, state exits correctly to IDLE. |

---

## Phase 9 — Robustness & Error Paths

| Goal                      | Tasks                                                                                                                                                                                            | Done when                                                                                                 |
| ------------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------ | --------------------------------------------------------------------------------------------------------- |
| Harden against edge cases | \* Add CRC check on ASCII frames, watchdog resets, brown-out detection.<br>\* Implement buffering during LoRa outages; flush on reconnect.<br>\* Unit-test every public API with invalid inputs. | Static analysis (cppcheck + clang-tidy) reports no critical findings; 100 % branch coverage on CmdParser. |

---

## Phase 10 — Performance & Extras

| Goal                   | Tasks                                                                                                                                                                                                     | Done when                                                  |
| ---------------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------- |
| Polish & stretch goals | \* Upgrade to binary telemetry option.<br>\* Add AES-CTR encryption.<br>\* Optimize ISR latency (DMA, `__disable_irq` audit).<br>\* Continuous integration: GitHub Actions + unit + hardware-in-the-loop. | CI green, encrypted link demonstrated, ISR jitter < 20 µs. |

---

### How to Use This Roadmap

1. **One phase per pull-request.** Merge only when the “done” check passes on hardware.
2. **Tag releases**: `v0.1-fsm`, `v0.2-lora`, etc., so you can always revert.
3. **Document as you go**: update README tables and state diagrams after each phase.

Following these milestones keeps complexity contained, surfaces integration issues early, and provides a clear “definition of done” at every step.
