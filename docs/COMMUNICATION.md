# Brunito Software Architecture

## 1. Introduction

The Brunito project is a flight control system comprising three STM32F401CCU6-based modules: Navigation Controller (NAVC), Flight Controller (FC), and Ground Station (GS). Built using the Arduino framework via PlatformIO, the system manages autonomous flight, sensor data, telemetry, and ground communication. This document provides a comprehensive software architecture, detailing component interactions, state management, communication protocols, and key functions, with a strong emphasis on live telemetry and an optimized source layout. It addresses concerns about source structure simplicity while maintaining modularity, corrects logical inconsistencies (e.g., LoRa settings synchronization), and ensures all commands are accessible via both GS and debug interfaces.

## 2. System Overview

The Brunito system integrates three components:

- **NAVC**: Collects sensor data, performs data fusion, and logs to SD card.
- **FC**: Manages states, routes commands, and streams telemetry.
- **GS**: Provides a command-line interface (CLI) for operators.

**Communication Links:**
- **NAVC ↔ FC**: UART2 at 115200 bps for binary packets and commands.
- **FC ↔ GS**: LoRa at 915 MHz for ASCII-framed commands and telemetry.
- **FC ↔ Debug**: USB-CDC at 921600 bps for bench debugging (not used in flight).

The system uses Arduino’s cooperative loop without an RTOS, and a finite-state machine (FSM) governs behavior, with all states exiting to IDLE.

## 3. Component Descriptions

### 3.1 Navigation Controller (NAVC)
- **Hardware**: STM32F401CCU6 Blackpill, UBLOX MAX M10S GPS (PB6 TX, PB7 RX), DS3231 RTC (I2C 0x68), BMP280 barometer (I2C 0x77), BMI088 IMU (I2C 0x19/0x69), BMM150 magnetometer (I2C 0x13), SD card (SPI, CS PA4), SK6812 RGB LED (PC14).
- **Software**: Arduino framework with libraries: TinyGPSPlus, RTClib, Adafruit BMP280, Bolder Flight BMI088, Bosch BMM150, SD, Adafruit NeoPixel.
- **Responsibilities**: Sample sensors at 100 Hz, fuse data, log to SD, communicate via UART2.

### 3.2 Flight Controller (FC)
- **Hardware**: STM32F401CCU6, RFM95W LoRa (SPI: NSS PA4, DIO0 PA8, NRST PA9, DIO1 PA10), PWM servo (PB10, TIM2 CH3, 200Hz), buzzer (PB13).
- **Software**: Arduino framework with RadioLib.
- **Responsibilities**: Manage FSM, route commands, convert packets to ASCII, control actuators.

### 3.3 Ground Station (GS)
- **Hardware**: STM32F401CCU6, RFM95W LoRa (same pinout as FC).
- **Software**: Arduino framework with RadioLib.
- **Responsibilities**: Send commands, receive/display telemetry via USB-CDC (115200 bps).

## 4. Source Layout

The source layout is designed for modularity, scalability, and clarity, addressing the need for simplicity while supporting the project’s complexity. Each component has its own directory with a minimal number of files to keep the structure intuitive. Shared utilities are centralized in `/include/utils/` to avoid duplication.

```
/src
  navc/
    main.cpp        // Entry point for NAVC
    Sensors.cpp|h   // Sensor sampling and fusion
    Packet.cpp|h    // Binary packet encoding
  fc/
    main.cpp        // Entry point for FC
    CmdParser.cpp|h // Command parsing and routing
    State.cpp|h     // FSM and state-specific logic
  gs/
    main.cpp        // Entry point for GS
    Cli.cpp|h       // CLI and LoRa communication
/include
  config/
    lora.h        // LoRa configuration
    system.h      // System-wide constants
  utils/
    FrameCodec.h  // Telemetry encoding/decoding
    Heartbeat.h   // LED and status utilities
```

**Rationale**:
- **Component Directories**: Separate `navc/`, `fc/`, `gs/` align with PlatformIO’s multi-environment setup, using `build_src_filter` (e.g., `+<src/fc/*>`) to compile only relevant files per module.
- **Minimal Files**: Each component has 2–3 files, balancing modularity with simplicity. For example, FC’s `State.cpp` consolidates state logic, reducing file count from the previous four state-specific files.
- **Shared Utilities**: `FrameCodec.h` and `Heartbeat.h` provide reusable functions (e.g., telemetry formatting, LED patterns) across components, included as needed.
- **Flat Hierarchy**: Subdirectories are kept to a minimum to avoid complexity, ensuring new developers can navigate easily.

**Comparison to Single-File Approach**:
A single-file approach (`navc.cpp`, `fc.cpp`, `gs.cpp`) would consolidate code but risks creating large, unwieldy files (potentially thousands of lines each). This could hinder maintenance and collaboration. The current layout, with small, focused files, improves readability and supports incremental compilation, as only modified files are recompiled.

## 5. Communication Protocols

### 5.1 Message Framing
ASCII framing: `<TYPE:PAYLOAD>`
- **Delimiters**: `<`, `>`
- **Separator**: `:`
- **Types**:
  | Type  | Description                | Example                     |
  |-------|----------------------------|-----------------------------|
  | CMD   | Command/response           | `<CMD:DISARM>`              |
  | TEST  | Test output                | `<TEST:BMP280_OK>`          |
  | TELEM | Telemetry data             | `<TELEM:pkID,ts,lat,lon,…>` |
  | ALERT | Critical notifications     | `<ALERT:LOW_BATTERY>`       |
  | DEBUG | Debug info                 | `<DEBUG:INIT_SENSORS>`      |

### 5.2 UART2 (NAVC-FC)
- **Baud Rate**: 115200 bps
- **Format**: 50 Hz binary packets
- **Structure**:
  | Bytes | Field       | Type     | Scaling        | Description               |
  |-------|-------------|----------|----------------|---------------------------|
  | 0–1   | pkID        | uint16_t | None           | Packet ID                 |
  | 2–5   | timestamp   | uint32_t | milliseconds   | Time since start          |
  | 6–9   | alt         | int32_t  | meters * 100   | Altitude                  |
  | 10–11 | accel_x     | int16_t  | g * 1000       | Acceleration X            |
  | 12–13 | accel_y     | int16_t  | g * 1000       | Acceleration Y            |
  | 14–15 | accel_z     | int16_t  | g * 1000       | Acceleration Z            |
  | 16–17 | gyro_x      | int16_t  | dps * 100      | Gyroscope X               |
  | 18–19 | gyro_y      | int16_t  | dps * 100      | Gyroscope Y               |
  | 20–21 | gyro_z      | int16_t  | dps * 100      | Gyroscope Z               |
  | 22–23 | mag_x       | int16_t  | µT * 10        | Magnetometer X            |
  | 24–25 | mag_y       | int16_t  | µT * 10        | Magnetometer Y            |
  | 26–27 | mag_z       | int16_t  | µT * 10        | Magnetometer Z            |
  | 28–31 | lat         | int32_t  | degrees * 1e7  | Latitude                  |
  | 32–35 | lon         | int32_t  | degrees * 1e7  | Longitude                 |
  | 36–37 | crc16       | uint16_t | CCITT-F        | Checksum                  |

### 5.3 LoRa (FC-GS)
- **Config**: 915 MHz, SF7, BW 250 kHz, CR 4/5, Sync Word 0xAB, FC Addr 0xA2, GS Addr 0xA1.
- **Format**: ASCII frames.
- **Error Handling**: RadioLib ACK + 3 retries with backoff.

### 5.4 USB-CDC (FC-Debug)
- **Baud Rate**: 921600 bps
- **Format**: ASCII frames, bench-only.

## 6. Finite-State Machine

The FC hosts a four-state FSM, exiting to IDLE.

### 6.1 States and Behaviors
| State    | Entry Condition                     | Behavior                              | Allowed Commands         |
|----------|-------------------------------------|---------------------------------------|--------------------------|
| IDLE     | Power-on, DISARM                    | 1 Hz status, full commands            | All                      |
| TEST     | CMD:ENTER_TEST from IDLE            | 50 Hz test outputs, TEST/QUERY only   | TEST, QUERY, ARM, DISARM |
| ARMED    | CMD:ARM from IDLE/TEST              | 20 Hz telemetry, no TEST commands     | All except TEST          |
| RECOVERY | Auto (30 min + no motion) or CMD:ENTER_RECOVERY from ARMED | 1 Hz GPS-only, buzzer, CRITICAL only | DISARM, FIND_ME          |

- **No-Motion Threshold**: ±0.02 g.
- **Auto-RECOVERY Timer**: 30 minutes.

### 6.2 Transitions
```
[IDLE] -- CMD:ENTER_TEST --> [TEST]
[IDLE] -- CMD:ARM --> [ARMED]
[TEST] -- CMD:DISARM --> [IDLE]
[TEST] -- CMD:ARM --> [ARMED]
[ARMED] -- CMD:DISARM --> [IDLE]
[ARMED] -- auto or CMD:ENTER_RECOVERY --> [RECOVERY]
[RECOVERY] -- CMD:DISARM --> [IDLE]
```

## 7. Command Processing

Commands are prioritized:
1. **Critical**: DISARM.
2. **Control**: Servo, buzzer.
3. **Query**: Status.
4. **Test**: Lowest in ARMED.

**Flow**:
1. FC receives `<CMD:…>` via LoRa/USB-CDC.
2. `CmdParser.cpp` validates state.
3. Commands forwarded to NAVC or executed locally.
4. Responses sent to LoRa/USB-CDC.

## 8. Telemetry and Logging

### 8.1 Live Telemetry Streaming
Live telemetry enables real-time flight monitoring, active in ARMED (full sensors) and RECOVERY (GPS-only) states. It uses minimal ASCII frames for backend parsing.

#### Formats
- **ARMED**: `<TELEM:pkID,timestamp,alt,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,mag_x,mag_y,mag_z,lat,lon,sats>`
  | Field     | Type     | Scaling        | Description               |
  |-----------|----------|----------------|---------------------------|
  | pkID      | uint16_t | None           | Packet ID                 |
  | timestamp | uint32_t | milliseconds   | Time since start          |
  | alt       | int32_t  | meters * 100   | Altitude                  |
  | accel_x   | int16_t  | g * 1000       | Acceleration X            |
  | accel_y   | int16_t  | g * 1000       | Acceleration Y            |
  | accel_z   | int16_t  | g * 1000       | Acceleration Z            |
  | gyro_x    | int16_t  | dps * 100      | Gyroscope X               |
  | gyro_y    | int16_t  | dps * 100      | Gyroscope Y               |
  | gyro_z    | int16_t  | dps * 100      | Gyroscope Z               |
  | mag_x     | int16_t  | µT * 10        | Magnetometer X            |
  | mag_y     | int16_t  | µT * 10        | Magnetometer Y            |
  | mag_z     | int16_t  | µT * 10        | Magnetometer Z            |
  | lat       | int32_t  | degrees * 1e7  | Latitude                  |
  | lon       | int32_t  | degrees * 1e7  | Longitude                 |
  | sats      | uint8_t  | None           | GPS satellite count       |
- **RECOVERY**: `<TELEM:timestamp,lat,lon,alt,sats>`
  | Field     | Type     | Scaling        | Description               |
  |-----------|----------|----------------|---------------------------|
  | timestamp | uint32_t | milliseconds   | Time since start          |
  | lat       | int32_t  | degrees * 1e7  | Latitude                  |
  | lon       | int32_t  | degrees * 1e7  | Longitude                 |
  | alt       | int32_t  | meters * 100   | Altitude                  |
  | sats      | uint8_t  | None           | GPS satellite count       |

#### Encoding
- **NAVC**: Sends 50 Hz binary packets (38 bytes) to FC.
- **FC**: Converts to ASCII, formats as comma-separated integers, sends via LoRa/USB-CDC.
- **Example Code**:
  ```cpp
  String formatTelem(const uint8_t* packet, bool gpsOnly) {
      uint16_t pkID = *(uint16_t*)&packet[0];
      uint32_t timestamp = *(uint32_t*)&packet[2];
      int32_t alt = *(int32_t*)&packet[6];
      if (gpsOnly) {
          int32_t lat = *(int32_t*)&packet[28];
          int32_t lon = *(int32_t*)&packet[32];
          char buf[100];
          snprintf(buf, sizeof(buf), "<TELEM:%lu,%ld,%ld,%ld>", timestamp, lat, lon, alt);
          return String(buf);
      }
      // Extract other fields
      char buf[200];
      snprintf(buf, sizeof(buf), "<TELEM:%u,%lu,%ld,%d,%d,%d,%d,%d,%d,%d,%d,%d,%ld,%ld>",
               pkID, timestamp, alt, /* ... */, lat, lon);
      return String(buf);
  }
  ```

#### Decoding
- **Backend**: Split on commas, convert to integers, apply scaling (e.g., alt / 100.0).
- **Example Python**:  ```python
  def parse_telem(payload):
      fields = payload.strip('<TELEM:>').split(',')
      if len(fields) == 15:  # ARMED
          return {
              'pkID': int(fields[0]),
              'timestamp': int(fields[1]),
              'alt': int(fields[2]) / 100.0,
              'accel': [int(f) / 1000.0 for f in fields[3:6]],
              'gyro': [int(f) / 100.0 for f in fields[6:9]],
              'mag': [int(f) / 10.0 for f in fields[9:12]],
              'lat': int(fields[12]) / 1e7,
              'lon': int(fields[13]) / 1e7,
              'sats': int(fields[14])
          }
      elif len(fields) == 5:  # RECOVERY
          return {
              'timestamp': int(fields[0]),
              'lat': int(fields[1]) / 1e7,
              'lon': int(fields[2]) / 1e7,
              'alt': int(fields[3]) / 100.0,
              'sats': int(fields[4])
          }
      return None
  ```

#### Rate Control
- **ARMED**: Default 20 Hz, capped at ~4 Hz due to LoRa bandwidth (~683 bytes/s, ~160 bytes/frame).
- **RECOVERY**: 1 Hz (~50 bytes/frame).
- **Logic**: `startTelemetry(hz)` adjusts rate, reduces during critical commands.

#### Example Frames
- **ARMED**: `<TELEM:1234,567890123,1000,-980,0,100,50,0,0,-1234,5678,-9012,34567890,-12345670>`
  - Parsed: `{pkID: 1234, timestamp: 567890123, alt: 10.0, accel: [-0.98, 0.0, 0.1], ...}`
- **RECOVERY**: `<TELEM:567890123,34567890,-12345670,1000>`
  - Parsed: `{timestamp: 567890123, lat: 3.456789, lon: -1.234567, alt: 10.0}`

### 8.2 SD Card Logging
- **State**: ARMED
- **Rate**: Max sustainable (≤100 Hz, benchmarked by NAVC).
- **Format**: Binary packets.
- **Flow**: NAVC writes to `FLIGHT_YYMMDD_HHMM.BIN`; errors raise `<ALERT:SD_FAIL>`.

## 9. Function Specifications

| Function            | State       | Parameters                              | Description                                                                 | Return        |
|---------------------|-------------|----------------------------------------|-----------------------------------------------------------------------------|---------------|
| lora_settings()     | IDLE        | uint32_t freq, uint8_t sf, uint32_t bw, uint8_t cr, uint8_t node, uint8_t target | Updates LoRa settings on FC, then GS, with ACKs. | bool (success) |
| updateRTC()         | IDLE, TEST  | time_t unixTs                          | Updates NAVC RTC via RTClib.                                                | bool (ACK)    |
| arm()               | IDLE, TEST  | —                                      | Transitions to ARMED, starts telemetry.                                     | —             |
| enterTest()         | IDLE        | —                                      | Enters TEST, begins 50 Hz diagnostics.                                      | —             |
| exitTest()          | TEST        | —                                      | Exits to IDLE, stops test outputs.                                          | —             |
| startTelemetry()    | ARMED       | uint8_t hz = 20                        | Starts telemetry at specified Hz, capped by bandwidth.                      | —             |
| startSdRecording()  | ARMED       | uint8_t *actualHz = nullptr            | Starts SD logging, returns actual rate.                                     | bool (ACK)    |
| startFlightLog()    | ARMED       | —                                      | Calls startTelemetry() and startSdRecording().                              | bool (success) |
| enterRecovery()     | ARMED       | —                                      | Enters RECOVERY, activates findMe().                                        | —             |
| findMe()            | RECOVERY    | —                                      | Activates buzzer/LED for location.                                          | —             |
| sendGpsOnly()       | RECOVERY    | —                                      | Sends GPS-only TELEM at 1 Hz.                                               | —             |
| disarm()            | TEST, ARMED, RECOVERY | —                            | Transitions to IDLE, stops telemetry/logging.                               | —             |
| adjustTelemRate()   | ARMED       | uint8_t hz                             | Adjusts telemetry rate.                                                     | —             |

## 10. Integration and Testing

- **State Transitions**: Verify ARMED → RECOVERY (auto/manual).
- **Command Handling**: Test filtering under telemetry load.
- **Telemetry/Logging**: Measure SD rate, ensure no streaming interference.
- **Error Cases**: Simulate LoRa loss, SD failures, UART errors.
- **Debug Menu**: Validate USB-CDC menu in all states.

## 11. Future Enhancements

- **Binary Telemetry**: Increase data rate.
- **Error Correction**: Add LoRa CRC.
- **Buffering**: Store telemetry during outages.
- **Heartbeat**: Detect component failures.
- **Encryption**: Secure commands.

## 12. Logical Corrections

- **LoRa Settings**: `lora_settings()` uses current settings to avoid communication loss.
- **SD Logging**: NAVC benchmarks SD speed to prevent blocking.
- **State Exits**: Single-exit-to-IDLE rule enforced.
- **Telemetry**: Standardized ARMED/RECOVERY formats.