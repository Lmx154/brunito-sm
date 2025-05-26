# Brunito Hardware Setup Guide

## Overview

The Brunito project uses three STM32F401CCU6 Blackpill modules:

* **NAVC (Navigation & Autonomous Vehicle Computer)** - Sensor hub and data logger
* **FC (Flight Computer)** - State machine and telemetry router
* **GS (Ground Station)** - Operator interface

All modules are developed using PlatformIO with the Arduino framework.

## Hardware Specifications

### Common for all modules
- **MCU**: STM32F401CCU6 @ 84MHz
- **Board**: WeAct Studio Blackpill V3.0
- **Memory**: 256KB Flash, 64KB SRAM
- **Power**: USB or 3.3V/5V external
- **Upload**: DFU bootloader via USB

### NAVC (Navigation & Autonomous Vehicle Computer)

#### Connections

| Component | Interface | Pins | Address/Config |
|-----------|-----------|------|----------------|
| **UBLOX MAX-M10S GPS** | UART | TX: PB6, RX: PB7 | 9600 bps |
| **DS3231 RTC** | I2C | SCL: PB8, SDA: PB9 | 0x68 |
| **BMP280 Barometer** | I2C | SCL: PB8, SDA: PB9 | 0x77 |
| **BMI088 Accelerometer** | I2C | SCL: PB8, SDA: PB9 | 0x19 |
| **BMI088 Gyroscope** | I2C | SCL: PB8, SDA: PB9 | 0x69 |
| **BMM150 Magnetometer** | I2C | SCL: PB8, SDA: PB9 | 0x13 |
| **SD Card** | SPI | CS: PA4, MOSI: PA7, MISO: PA6, SCK: PA5 | - |
| **SK6812 RGB LED** | GPIO | Data: PC14 | WS2812B protocol |
| **Buzzer** | GPIO | PA0 | Active buzzer, shared with FC |
| **Onboard LED** | GPIO | PC13 | Active LOW |
| **FC UART** | Serial2 | TX: PA3, RX: PA2 | 115200 bps |
| **USB Debug** | USB CDC | - | 921600 bps |

#### Sensor Configuration

**I2C Bus**: 100 kHz (reduced for BMM150 compatibility)

**BMP280 Settings**:
- Mode: Normal
- Temperature oversampling: 2x
- Pressure oversampling: 16x
- Filter: 4x
- Standby: 1ms

**BMI088 Settings**:
- Accelerometer: ±6g range, 800Hz ODR
- Gyroscope: ±500°/s range, 400Hz ODR

**BMM150 Settings**:
- Mode: Normal
- Preset: High accuracy

### FC (Flight Computer)

#### Connections

| Component | Interface | Pins | Address/Config |
|-----------|-----------|------|----------------|
| **RFM95W LoRa** | SPI | NSS: PA4, MOSI: PA7, MISO: PA6, SCK: PA5 | - |
| **LoRa Control** | GPIO | DIO0: PA8, NRST: PA9, DIO1: PA10 | - |
| **Servo** | PWM | PB14 | TIM2 CH3, 200Hz |
| **Buzzer** | GPIO | PA0 | Active buzzer, shared with NAVC |
| **NAVC UART** | Serial2 | TX: PA3, RX: PA2 | 115200 bps |
| **Onboard LED** | GPIO | PC13 | Active LOW |
| **USB Debug** | USB CDC | - | 921600 bps |

#### LoRa Configuration

- **Frequency**: 915 MHz
- **Spreading Factor**: 7
- **Bandwidth**: 500 kHz
- **Coding Rate**: 4/5
- **Sync Word**: 0xAB
- **TX Power**: 20 dBm
- **Node Address**: 0xA2

**Note**: The buzzer on pin PA0 is shared between NAVC and FC modules for audio feedback.

### GS (Ground Station)

#### Connections

| Component