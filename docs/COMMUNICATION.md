# Brunito Communication Architecture

## 1. Overview

The Brunito flight control system uses a multi-layered communication architecture:

- **Physical Layer**: UART (NAVC-FC) and LoRa (FC-GS)
- **Data Layer**: Binary packets (NAVC-FC) and ASCII frames (FC-GS)
- **Application Layer**: Commands, telemetry, and status messages

## 2. Communication Links

### 2.1 NAVC ↔ FC (UART2)
- **Baud Rate**: 115200 bps
- **Format**: Binary packets at 50 Hz
- **Pins**: NAVC TX=PA3, RX=PA2 | FC RX=PA2, TX=PA3
- **Flow Control**: None
- **Error Detection**: CRC-16 on each packet

### 2.2 FC ↔ GS (LoRa)
- **Frequency**: 915 MHz
- **Configuration**:
  - Spreading Factor: 7
  - Bandwidth: 500 kHz (increased from 250 kHz)
  - Coding Rate: 4/5
  - Sync Word: 0xAB
  - TX Power: 20 dBm
  - Preamble: 6 symbols
- **Addressing**: FC=0xA2, GS=0xA1
- **Error Handling**: ACK with 2 retries, exponential backoff

### 2.3 Debug Interfaces (USB-CDC)
- **NAVC**: 921600 bps
- **FC**: 921600 bps  
- **GS**: 115200 bps

## 3. Message Formats

### 3.1 ASCII Frame Format
All high-level messages use ASCII framing:
```
<TYPE:PAYLOAD>
```

Message types:
| Type  | Description                | ACK Required | Example                          |
|-------|----------------------------|--------------|----------------------------------|
| CMD   | Commands/responses         | Yes          | `<CMD:ARM>`                      |
| TELEM | Telemetry data            | No           | `<datetime,alt,accel,...>`       |
| TEST  | Test mode output          | No           | `<TEST:BMP280_OK>`               |
| ALERT | Critical notifications    | No           | `<ALERT:LOW_BATTERY>`            |
| DEBUG | Debug information         | No           | `<DEBUG:INIT_SENSORS>`           |
| STATUS| Status updates            | No           | `<STATUS:IDLE,12345>`            |

### 3.2 Binary Packet Format (NAVC→FC)

46-byte structure sent at 50 Hz:

| Offset | Field       | Type     | Scaling          | Description               |
|--------|-------------|----------|------------------|---------------------------|
| 0-1    | packetId    | uint16_t | None             | Packet sequence number    |
| 2-5    | timestamp   | uint32_t | milliseconds     | Time since boot           |
| 6-9    | altitude    | int32_t  | cm (meters×100)  | Altitude in centimeters   |
| 10-11  | accelX      | int16_t  | mg (g×1000)      | Acceleration X            |
| 12-13  | accelY      | int16_t  | mg (g×1000)      | Acceleration Y            |
| 14-15  | accelZ      | int16_t  | mg (g×1000)      | Acceleration Z            |
| 16-17  | gyroX       | int16_t  | 0.01 dps         | Gyroscope X               |
| 18-19  | gyroY       | int16_t  | 0.01 dps         | Gyroscope Y               |
| 20-21  | gyroZ       | int16_t  | 0.01 dps         | Gyroscope Z               |
| 22-23  | magX        | int16_t  | 0.1 µT           | Magnetometer X            |
| 24-25  | magY        | int16_t  | 0.1 µT           | Magnetometer Y            |
| 26-27  | magZ        | int16_t  | 0.1 µT           | Magnetometer Z            |
| 28-31  | latitude    | int32_t  | degrees×1e7      | GPS Latitude              |
| 32-35  | longitude   | int32_t  | degrees×1e7      | GPS Longitude             |
| 36     | year        | uint8_t  | Last 2 digits    | RTC Year (YY)             |
| 37     | month       | uint8_t  | 1-12             | RTC Month                 |
| 38     | day         | uint8_t  | 1-31             | RTC Day                   |
| 39     | hour        | uint8_t  | 0-23             | RTC Hour                  |
| 40     | minute      | uint8_t  | 0-59             | RTC Minute                |
| 41     | second      | uint8_t  | 0-59             | RTC Second                |
| 42     | satellites  | uint8_t  | Count            | GPS satellites in view    |
| 43-44  | temperature | int16_t  | °C (whole)       | Temperature               |
| 44-45  | crc16       | uint16_t | CCITT-0xFFFF     | CRC-16 checksum           |

## 4. Telemetry Formats

### 4.1 ARMED State Telemetry (20 Hz max, adaptive)
```
<MM/DD/YYYY,HH:MM:SS,altitude,accelX,accelY,accelZ,gyroX,gyroY,gyroZ,magX,magY,magZ,lat,lon,sats,temp>
```

Example:
```
<05/26/2025,15:20:30,10.25,-980,0,100,50,0,0,-1234,5678,-9012,34567890,-12345670,8,22>
```

### 4.2 RECOVERY State Telemetry (1 Hz fixed)
```
<MM/DD/YYYY,HH:MM:SS,latitude,longitude,altitude,satellites,temperature>
```

Example:
```
<05/26/2025,15:20:30,34567890,-12345670,10.25,8,22>
```

## 5. Command Protocol

### 5.1 Command Format
```
<CMD:COMMAND_NAME[:parameters][:checksum]>
```

### 5.2 Command List

| Command              | Parameters           | States Allowed      | Description                    |
|---------------------|---------------------|---------------------|--------------------------------|
| ARM                 | None                | IDLE, TEST          | Transition to ARMED state      |
| DISARM              | None                | All                 | Return to IDLE state           |
| ENTER_TEST          | None                | IDLE                | Enter TEST mode                |
| ENTER_RECOVERY      | None                | ARMED               | Enter RECOVERY mode            |
| QUERY               | None                | All                 | Query current state            |
| CONTROL             | servo=0-180,buzzer=0/1 | All except TEST | Control actuators           |
| TEST                | None                | TEST                | Test buzzer on NAVC            |
| SERVO_TEST          | None                | TEST                | Test servo movement            |
| ALTITUDE_TEST       | threshold=cm        | TEST                | Test altitude-triggered actions|
| NAVC_RESET_STATS    | None                | All                 | Reset NAVC packet statistics   |
| LORA_RESET_STATS    | None                | All                 | Reset LoRa statistics          |
| LORA_STATS          | None                | All                 | Get LoRa link statistics       |

### 5.3 Acknowledgment Format
```
<CMD_ACK:STATUS[:details]>
```

Examples:
- `<CMD_ACK:OK:ARMED>` - Command successful, now in ARMED state
- `<CMD_ACK:ERR:DENIED>` - Command not allowed in current state

## 6. LoRa Packet Structure

### 6.1 Header Format (5 bytes)
| Byte | Field  | Description            |
|------|--------|------------------------|
| 0    | Type   | Message type (see 6.2) |
| 1    | Source | Source address         |
| 2    | Dest   | Destination address    |
| 3-4  | ID     | Packet ID (16-bit)     |

### 6.2 Message Types
| Value | Type      | ACK Required | Description                |
|-------|-----------|--------------|----------------------------|
| 0x01  | CMD       | Yes          | Commands                   |
| 0x02  | ACK       | No           | Acknowledgments            |
| 0x03  | TELEM     | No           | Telemetry (fire-and-forget)|
| 0x04  | SETTINGS  | Yes          | LoRa settings exchange     |
| 0x05  | STATUS    | No           | Status updates             |
| 0x06  | PING      | No           | Link quality test request  |
| 0x07  | PONG      | No           | Link quality test response |

## 7. Flow Control

### 7.1 Telemetry Rate Adaptation
The FC automatically adjusts telemetry rate based on link quality:

| RSSI (dBm) | Packet Loss | Queue Depth | Telemetry Rate |
|------------|-------------|-------------|----------------|
| > -80      | < 10%       | < 2         | 10 Hz (max)    |
| > -90      | < 10%       | < 2         | 8 Hz           |
| > -100     | < 20%       | < 3         | 5 Hz           |
| > -110     | < 30%       | < 4         | 2 Hz           |
| ≤ -110     | ≥ 30%       | ≥ 4         | 1 Hz           |

### 7.2 Queue Management
- Queue size: 12 packets
- Priority order: SETTINGS → CMD → TELEM → Others
- Critical commands (DISARM, ENTER_RECOVERY) get extra retries

## 8. Error Handling

### 8.1 UART Error Recovery
- CRC validation on every packet
- Packet sequence tracking for missed packet detection
- Automatic UART reinitialization after 3 seconds of no data

### 8.2 LoRa Error Recovery
- ACK timeout: 2000ms (adaptive based on RSSI)
- Max retries: 2 (5 for critical commands)
- Exponential backoff: 1.5× per retry
- Automatic statistics reset every hour

## 9. SD Card Logging

### 9.1 Log File Format
- Filename: LOG00001.TXT (8.3 format)
- Header: Metadata including start time and column names
- Data: CSV format matching telemetry structure
- Write rate: As fast as SD card allows (typically 50-100 Hz)

### 9.2 CSV Column Format
```
MM/DD/YYYY,HH:MM:SS,altitude,accelX,accelY,accelZ,gyroX,gyroY,gyroZ,magX,magY,magZ,lat,lon,sats,temp
```

Units:
- altitude: meters with 2 decimal places
- accel[X/Y/Z]: raw int16 values (mg)
- gyro[X/Y/Z]: raw int16 values (0.01 dps)
- mag[X/Y/Z]: raw int16 values (0.1 µT)
- lat/lon: raw int32 values (degrees×1e7)
- sats: satellite count
- temp: temperature in °C

## 10. Link Quality Monitoring

### 10.1 Metrics
- RSSI: Received Signal Strength Indicator (dBm)
- SNR: Signal-to-Noise Ratio (dB)
- Packet loss rate: Lost / (Sent + Lost) × 100%
- Connection timeout: 30 seconds

### 10.2 Status Reporting
Both FC and GS report link status every 30 seconds:
```
<FC_LINK:EXCELLENT,RSSI:-85,SNR:10.5,PKT_SENT:1234,PKT_RECV:1200,LOSS:2.8%,LAST_RX:00:00:05>
```

Link quality levels:
- EXCELLENT: RSSI > -90 dBm
- GOOD: RSSI > -100 dBm
- FAIR: RSSI > -110 dBm
- POOR: RSSI ≤ -110 dBm
- DOWN: No packets received within timeout