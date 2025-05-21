# B.R.U.N.O Project Details (Updated with PlatformIO Configuration)

## Overview

The **B.R.U.N.O** project comprises three STM32F401CCU6-based modules:

* **NAVC (Navigation & Autonomous Vehicle Computer)**
* **FC (Flight Computer)**
* **Ground Station**

Development is managed using **PlatformIO** with the Arduino framework. All libraries are defined in the `platformio.ini` file and are fetched through PlatformIO's dependency management system.

---

## PlatformIO Environment Configuration

**General Build Configuration:**

```ini
platform = ststm32
board = blackpill_f401cc
framework = arduino
upload_protocol = dfu
build_flags = \
  -DUSB_VID=0x0483 \
  -DUSB_PID=0x5740 \
  -DUSB_MANUFACTURER="STMicroelectronics" \
  -DUSB_PRODUCT="\"BLACKPILL_F401CC\"" \
  -DHAL_PCD_MODULE_ENABLED \
  -DUSBCON \
  -DUSBD_USE_CDC
```

**Libraries Used (PlatformIO `lib_deps`)**

* `adafruit/Adafruit NeoPixel`
* `adafruit/Adafruit BMP280 Library`
* `adafruit/RTClib`
* `mikalhart/TinyGPSPlus`
* `bolderflight/Bolder Flight Systems BMI088`
* `https://github.com/BoschSensortec/BMM150-Sensor-API`
* `jgromes/RadioLib`
* `SPI`
* `SD`

Each component (NAVC, FC, GS) uses a `build_src_filter` and `build_flags` for source separation and compile-time definitions.

---

## Hardware Details

### NAVC (Navigation & Autonomous Vehicle Computer)

* **MCU:** STM32F401CCU6 Blackpill
* **GPS:** UBLOX MAX M10S (UART @ 9600 bps)

  * Pins: PB6 (TX), PB7 (RX)
* **RTC:** DS3231 (I2C, Addr: 0x68)

  * Library: `RTClib`
* **Barometer:** BMP280 (I2C, Addr: 0x77)

  * Library: `Adafruit BMP280`
* **IMU:** BMI088 (I2C)

  * Accelerometer Addr: 0x19
  * Gyroscope Addr: 0x69
  * Library: `Bolder Flight Systems BMI088`
* **Magnetometer:** BMM150 (I2C, Addr: 0x13)

  * Library: Bosch BMM150 Sensor API
* **SD Card:** SPI

  * CS: PA4
* **UART to FC:** Serial2 (115200 bps)

  * TX: PA3, RX: PA2
* **LEDs:**

  * Onboard (PC13)
  * SK6812 MINI-E RGB (WS2812B compatible, PC14)
* **USB Debug:** Serial (USB CDC)

### FC (Flight Computer)

* **MCU:** STM32F401CCU6 Blackpill
* **LoRa Transceiver:** RFM95W 915MHz (SPI)

  * NSS: PA4
  * DIO0: PA8
  * NRST: PA9
  * DIO1: PA10
  * Library: `RadioLib`
  * Config:

    * Freq: 915 MHz
    * SF: 7
    * BW: 250 kHz
    * CR: 5
    * Sync Word: 0xAB
    * Node Addr: 0xA2
* **Servo Motor:** PWM (PB14)

  * Timer: TIM2, CH3, 200Hz
* **Buzzer:** Digital Output (PA0) - Passive buzzer
* **UART to NAVC:** Serial2 (115200 bps)

  * TX: PA3, RX: PA2
* **USB Debug:** Serial (USB CDC @ 921600 bps)

### Ground Station

* **MCU:** STM32F401CCU6 Blackpill
* **LoRa Transceiver:** RFM95W 915MHz (SPI)

  * NSS: PA4
  * DIO0: PA8
  * NRST: PA9
  * DIO1: PA10
  * Library: `RadioLib`
  * Config:

    * Freq: 915 MHz
    * SF: 7
    * BW: 250 kHz
    * CR: 5
    * Sync Word: 0xAB
    * Node Addr: 0xA1
    * Target Addr (FC): 0xA2
* **USB Debug Console:** Serial (USB CDC @ 115200 bps)

---

## Source File Mapping (from `platformio.ini`)

| Module         | Source File | Build Flag     |
| -------------- | ----------- | -------------- |
| NAVC           | `NAVC.cpp`  | `-DNAVC_BUILD` |
| FC             | `FC.cpp`    | `-DFC_BUILD`   |
| Ground Station | `GS.cpp`    | `-DGS_BUILD`   |

---

## Notes

* All modules utilize `Wire.h` for I2C, `SPI.h` for SPI, and `HardwareSerial.h` for UART.
* USB CDC is enabled through custom build flags.
* DFU upload is configured as the default protocol.
* Libraries are centrally managed and resolved automatically by PlatformIO.
* If the Bosch BMM150 library is modified or custom, maintain the local fork URL or git reference in `lib_deps`.

This documentation ensures coherence between hardware, firmware, and build environment as defined in `platformio.ini`.
