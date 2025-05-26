# Brunito Project

This repository contains code for the Brunito flight control system, which consists of three main components:
- **Navigation Controller (NAVC)** - Collects sensor data, performs data fusion, and logs to SD card
- **Flight Controller (FC)** - Manages states, routes commands, and streams telemetry  
- **Ground Station (GS)** - Provides a command-line interface for operators

## System Overview

The Brunito system uses STM32F401CCU6 Blackpill boards for all three modules, communicating via:
- **NAVC ↔ FC**: UART2 at 115200 bps for binary sensor packets
- **FC ↔ GS**: LoRa at 915 MHz for ASCII-framed commands and telemetry
- **Debug interfaces**: USB-CDC (NAVC: 921600 bps, FC: 921600 bps, GS: 115200 bps)

## Key Features

### NAVC (Navigation Controller)
- Sensor fusion at 100 Hz from multiple sensors:
  - BMI088 IMU (accelerometer + gyroscope)
  - BMM150 magnetometer
  - BMP280 barometer
  - UBLOX MAX-M10S GPS
  - DS3231 RTC
- SD card logging with hot-swap support
- Binary packet streaming to FC at 50 Hz
- SK6812 RGB LED status indicator

### FC (Flight Controller)
- Four-state finite state machine (IDLE, TEST, ARMED, RECOVERY)
- Telemetry rate adaptation based on link quality (1-20 Hz)
- Command routing between GS and NAVC
- PWM servo control and buzzer support
- Automatic recovery mode after 30 minutes of inactivity

### GS (Ground Station)
- Command-line interface for system control
- Real-time telemetry display
- Link quality monitoring
- Command acknowledgment with retry logic

## Build and Upload Instructions

The project uses PlatformIO for building and uploading firmware. You can build and upload each component independently:

### Upload a specific component

```bash
# Upload only the Navigation Controller
pio run -e navc -t upload

# Upload only the Flight Controller  
pio run -e fc -t upload

# Upload only the Ground Station
pio run -e gs -t upload
```

### Build without uploading

To build without uploading (for example, to check for compilation errors):

```bash
# Build only the Navigation Controller
pio run -e navc

# Build only the Flight Controller
pio run -e fc  

# Build only the Ground Station
pio run -e gs
```

## Communication Protocol

The system uses ASCII framing for all high-level communication:

```
<TYPE:PAYLOAD>
```

Where TYPE can be:
- `CMD` - Commands and responses
- `TELEM` - Telemetry data
- `TEST` - Test mode output
- `ALERT` - Critical notifications
- `DEBUG` - Debug information

## State Machine

The FC implements a finite state machine with the following transitions:

```
[IDLE] -- CMD:ENTER_TEST --> [TEST]
[IDLE] -- CMD:ARM --> [ARMED]  
[TEST] -- CMD:DISARM --> [IDLE]
[TEST] -- CMD:ARM --> [ARMED]
[ARMED] -- CMD:DISARM --> [IDLE]
[ARMED] -- auto or CMD:ENTER_RECOVERY --> [RECOVERY]
[RECOVERY] -- CMD:DISARM --> [IDLE]
```

## Data Logging

The NAVC module logs sensor data to SD card in CSV format:
- Automatic file creation with 8.3 naming (LOG00001.TXT)
- Hot-swap SD card support
- 10-second sensor stabilization before logging starts
- Orange LED blink on each write

## Dependencies

All dependencies are managed through PlatformIO's `lib_deps`:
- Adafruit NeoPixel
- Adafruit BMP280 Library
- RTClib
- TinyGPSPlus
- Bolder Flight Systems BMI088
- BMM150 Sensor API (Seeed Studio)
- RadioLib
- SD

## Version History

- v0.4 - Current version with SD logging, telemetry optimization, and improved error handling
- v0.3 - Added finite state machine and command processing
- v0.2 - Initial sensor integration
- v0.1 - Basic framework setup