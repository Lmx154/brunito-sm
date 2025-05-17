# Brunito Project

![Brunito Project](https://via.placeholder.com/800x200?text=Brunito+Flight+Control+System)

## 1. Overview

Brunito is a comprehensive flight control system designed for autonomous aerial vehicles. The system comprises three distinct STM32F401CCU6-based modules, each serving a specific purpose in the flight control ecosystem.

### System Components

- **Navigation Controller (NAVC)**: Collects sensor data, performs data fusion, and logs flight information to SD card
- **Flight Controller (FC)**: Manages system states, routes commands, and streams telemetry
- **Ground Station (GS)**: Provides a command-line interface for operators to monitor and control the flight system

## 2. Hardware Specifications

### Navigation Controller (NAVC)
- **MCU**: STM32F401CCU6 Blackpill
- **Sensors**:
  - UBLOX MAX M10S GPS (PB6 TX, PB7 RX)
  - DS3231 RTC (I2C 0x68)
  - BMP280 barometer (I2C 0x77)
  - BMI088 IMU (I2C 0x19/0x69)
  - BMM150 magnetometer (I2C 0x13)
- **Storage**: SD card (SPI, CS PA4)
- **Indicators**: SK6812 RGB LED (PC14)

### Flight Controller (FC)
- **MCU**: STM32F401CCU6 Blackpill
- **Communication**: RFM95W LoRa (SPI: NSS PA4, DIO0 PA8, NRST PA9, DIO1 PA10)
- **Actuators**: 
  - PWM servo (PB10, TIM2 CH3, 200Hz)
  - Buzzer (PB13)

### Ground Station (GS)
- **MCU**: STM32F401CCU6 Blackpill
- **Communication**: RFM95W LoRa (Same pinout as FC)
- **Interface**: USB-CDC for terminal access (115200 bps)

## 3. Communication Architecture

### Communication Links
- **NAVC ↔ FC**: UART2 at 115200 bps for binary packets and commands
- **FC ↔ GS**: LoRa at 915 MHz for ASCII-framed commands and telemetry
- **FC ↔ Debug**: USB-CDC at 921600 bps for bench debugging (not used in flight)

### LoRa Configuration
- **Frequency**: 915 MHz
- **Spreading Factor**: SF7
- **Bandwidth**: 250 kHz
- **Coding Rate**: 4/5
- **Sync Word**: 0xAB
- **Addressing**: FC 0xA2, GS 0xA1

## 4. Software Architecture

### State Machine
The system operates using a finite-state machine (FSM) with four primary states:

| State    | Description                              | Telemetry Rate |
|----------|------------------------------------------|----------------|
| IDLE     | Default state, full command access       | 1 Hz status    |
| TEST     | Diagnostics mode for pre-flight checks   | 50 Hz test     |
| ARMED    | Active flight mode with sensor sampling  | 20 Hz full     |
| RECOVERY | Power-saving mode with location services | 1 Hz GPS only  |

### Telemetry Formats
- **ARMED**: `<TELEM:pkID,timestamp,alt,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,mag_x,mag_y,mag_z,lat,lon>`
- **RECOVERY**: `<TELEM:timestamp,lat,lon,alt>`

## 5. Project Structure

```
└── lmx154-brunito-sm/
    ├── README.md           # Project documentation
    ├── COMMUNICATION.md    # Communication protocol details
    ├── HWSETUP.md          # Hardware configuration documentation
    ├── platformio.ini      # PlatformIO configuration
    ├── ROADMAP.md          # Project development roadmap
    ├── include/            # Header files
    │   ├── config/         # Configuration files
    │   ├── fc/             # Flight Controller headers
    │   ├── gs/             # Ground Station headers
    │   ├── navc/           # Navigation Controller headers
    │   └── utils/          # Shared utilities
    ├── src/                # Source files
    │   ├── fc/             # Flight Controller implementation
    │   ├── gs/             # Ground Station implementation
    │   ├── navc/           # Navigation Controller implementation
    │   └── utils/          # Shared utility implementation
    └── test/               # Test files
```

## 6. Build and Upload Instructions

The project uses PlatformIO for building and uploading firmware. You can selectively build and upload each component independently.

### Available Commands

#### Upload a specific component

```bash
# Upload only the Navigation Controller
pio run -e navc -t upload

# Upload only the Flight Controller
pio run -e fc -t upload

# Upload only the Ground Station
pio run -e gs -t upload
```

#### Upload the entire project

To build and upload all components:

```bash
pio run -e blackpill_f401cc -t upload
```

#### Build without uploading

To build without uploading (for example, to check for compilation errors):

```bash
# Build only the Navigation Controller
pio run -e navc

# Build only the Flight Controller
pio run -e fc

# Build only the Ground Station
pio run -e gs

# Build the entire project
pio run -e blackpill_f401cc
```

## 7. Libraries and Dependencies

The project relies on the following libraries, which are automatically managed through PlatformIO:

- `adafruit/Adafruit NeoPixel`
- `adafruit/Adafruit BMP280 Library`
- `adafruit/RTClib`
- `mikalhart/TinyGPSPlus`
- `bolderflight/Bolder Flight Systems BMI088`
- `https://github.com/BoschSensortec/BMM150-Sensor-API`
- `jgromes/RadioLib`
- `SPI`
- `SD`

## 8. License

This project is licensed under the MIT License - see the LICENSE file for details.

## 9. Contributors

- Project maintainer: [Your Name]
- Contributors: [List of contributors]

## 10. Documentation

For more detailed information, please refer to:

- [COMMUNICATION.md](COMMUNICATION.md) - Comprehensive overview of communication protocols
- [HWSETUP.md](HWSETUP.md) - Detailed hardware specifications and connections
- [ROADMAP.md](ROADMAP.md) - Future development plans