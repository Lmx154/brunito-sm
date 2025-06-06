# Brunito Flight System - Telemetry Data Sheet

## Flight Controller States

| State | Description | Telemetry Rate | Available Commands |
|-------|-------------|----------------|-------------------|
| **IDLE** | Ground idle, sensors inactive | No telemetry | `ARM`, `ENTER_TEST`, `DISARM`, `QUERY`, `NAVC_RESET_STATS` |
| **TEST** | Pre-flight diagnostics | 0.5Hz diagnostic | `ARM`, `DISARM`, `QUERY`, `TEST`, `SERVO_TEST`, `ALTITUDE_TEST`, `ENABLE_ALTITUDE_TEST`, `DISABLE_ALTITUDE_TEST`, `NAVC_RESET_STATS` |
| **ARMED** | Ready for flight, full sensors | 1-10Hz adaptive | `DISARM`, `ENTER_RECOVERY`, `QUERY`, `NAVC_RESET_STATS` |
| **RECOVERY** | Parachute deployed, GPS tracking | 1Hz GPS-only | `DISARM`, `QUERY`, `NAVC_RESET_STATS` |

## ARMED State Telemetry Format

**Output**: `<MM/DD/YYYY,HH:MM:SS,altitude,accelX,accelY,accelZ,gyroX,gyroY,gyroZ,magX,magY,magZ,latitude,longitude,satellites,temperature>`

**Example**: `<05/27/2025,11:43:46,0.95,-37,-967,-3,128,-27,204,6,-53,20,1,1,0,24>`

### Python Parsing Example
```python
def parse_armed_telemetry(line):
    data = line.strip('<>').split(',')
    return {
        'datetime': datetime.strptime(f"{data[0]},{data[1]}", "%m/%d/%Y,%H:%M:%S"),
        'altitude_m': float(data[2]),
        'accel_xyz_mg': [int(data[3]), int(data[4]), int(data[5])],
        'gyro_xyz_centidps': [int(data[6]), int(data[7]), int(data[8])],
        'mag_xyz_decisla': [int(data[9]), int(data[10]), int(data[11])],
        'gps_lat_1e7': int(data[12]),
        'gps_lon_1e7': int(data[13]),
        'gps_satellites': int(data[14]),
        'temperature_c': int(data[15])
    }
```

### Field Reference (16 fields)
| # | Field | Example | Units | Range | Conversion |
|---|-------|---------|-------|-------|------------|
| 1-2 | Date/Time | `05/27/2025,11:43:46` | MM/DD/YYYY,HH:MM:SS | - | Parse as datetime |
| 3 | Altitude | `0.95` | meters | -1000 to 50000 | Direct use |
| 4-6 | Acceleration | `-37,-967,-3` | mg | ±20000 | ÷1000 for g-force |
| 7-9 | Gyroscope | `128,-27,204` | 0.01°/s | ±200000 | ÷100 for °/s |
| 10-12 | Magnetometer | `6,-53,20` | 0.1µT | ±1000 | ÷10 for µT |
| 13-14 | GPS Position | `1,1` | degrees×10⁷ | ±1800000000 | ÷10⁷ for degrees |
| 15 | GPS Satellites | `0` | count | 0-12 | Direct use |
| 16 | Temperature | `24` | °C | -40 to 85 | Direct use |

## RECOVERY State Telemetry Format

**Output**: `<MM/DD/YYYY,HH:MM:SS,latitude,longitude,altitude,satellites,temperature>`

**Example**: `<05/27/2025,11:43:46,123456789,-456789012,125.50,8,23>`

### Python Parsing Example
```python
def parse_recovery_telemetry(line):
    data = line.strip('<>').split(',')
    return {
        'datetime': datetime.strptime(f"{data[0]},{data[1]}", "%m/%d/%Y,%H:%M:%S"),
        'gps_lat_deg': int(data[2]) / 10000000.0,  # 12.3456789°
        'gps_lon_deg': int(data[3]) / 10000000.0,  # -45.6789012°
        'altitude_m': float(data[4]),
        'gps_satellites': int(data[5]),
        'temperature_c': int(data[6])
    }
```

### Field Reference (7 fields)
| # | Field | Example | Units | Conversion |
|---|-------|---------|-------|------------|
| 1-2 | Date/Time | `05/27/2025,11:43:46` | MM/DD/YYYY,HH:MM:SS | Parse as datetime |
| 3-4 | GPS Position | `123456789,-456789012` | degrees×10⁷ | ÷10⁷ for degrees |
| 5 | Altitude | `125.50` | meters | Direct use |
| 6 | GPS Satellites | `8` | count | Direct use |
| 7 | Temperature | `23` | °C | Direct use |

## TEST State Telemetry Format

**Output**: `<TEST:ALT:125.50m,ACCEL:-0.037,0.967,-0.003>`

**Purpose**: Diagnostic sensor validation during pre-flight checks.

## Status Messages & Acknowledgments

### Command Responses
- **Success**: `<CMD_ACK:OK[:info]>` - e.g., `<CMD_ACK:OK:ARMED>`
- **Failure**: `<CMD_ACK:ERR[:reason]>` - e.g., `<CMD_ACK:ERR:DENIED>`

### Debug Messages
- **Format**: `DEBUG:<category>:<message>`
- **Examples**: `DEBUG:GPS:Fix acquired, satellites=8`

### Status Reports
- **Format**: `STATUS:<component>:<info>`
- **Examples**: `STATUS:BATTERY:7.4V (90%)`

## Command Availability Matrix

| Command | IDLE | TEST | ARMED | RECOVERY |
|---------|------|------|-------|----------|
| `<CMD:ARM>` | ✓ | ✓ | - | - |
| `<CMD:DISARM>` | ✓ | ✓ | ✓ | ✓ |
| `<CMD:ENTER_TEST>` | ✓ | - | - | - |
| `<CMD:ENTER_RECOVERY>` | - | - | ✓ | - |
| `<CMD:QUERY>` | ✓ | ✓ | ✓ | ✓ |
| `<CMD:NAVC_RESET_STATS>` | ✓ | ✓ | ✓ | ✓ |
| `<CMD:TEST>` | - | ✓ | - | - |
| `<CMD:SERVO_TEST>` | - | ✓ | - | - |
| `<CMD:ALTITUDE_TEST>` | - | ✓ | - | - |
| `<CMD:ENABLE_ALTITUDE_TEST>` | - | ✓ | - | - |
| `<CMD:DISABLE_ALTITUDE_TEST>` | - | ✓ | - | - |

## Command Usage Instructions

### Basic Commands (No Parameters)
Most commands require no parameters and follow the simple format:
```
<CMD:COMMAND_NAME>
```

**Examples:**
- `<CMD:ARM>` - Transition to ARMED state
- `<CMD:DISARM>` - Transition to IDLE state
- `<CMD:QUERY>` - Query current state
- `<CMD:TEST>` - Send buzzer test to NAVC
- `<CMD:SERVO_TEST>` - Test servo movement (0° → 90° → 0°)

### Commands with Parameters

#### ALTITUDE_TEST (One-time altitude threshold test)
**Format:** `<CMD:ALTITUDE_TEST:threshold=VALUE>`

**Parameters:**
- `threshold` - Altitude threshold in centimeters (1-30000)

**Examples:**
```
<CMD:ALTITUDE_TEST:threshold=200>     # Test at 2.0 meters
<CMD:ALTITUDE_TEST:threshold=1000>    # Test at 10.0 meters
<CMD:ALTITUDE_TEST:threshold=500>     # Test at 5.0 meters
```

**Behavior:**
- Compares current altitude against threshold
- If altitude ≥ threshold: Activates buzzer (500ms) and moves servo (0° → 90° → 0°)
- If altitude < threshold: Reports "ALTITUDE_BELOW_THRESHOLD"
- Default threshold: 200cm (2.0m) if parameter omitted

#### ENABLE_ALTITUDE_TEST (Background altitude monitoring)
**Format:** `<CMD:ENABLE_ALTITUDE_TEST:threshold=VALUE>`

**Parameters:**
- `threshold` - Altitude threshold in centimeters (minimum 1, no upper limit)

**Examples:**
```
<CMD:ENABLE_ALTITUDE_TEST:threshold=300>    # Monitor at 3.0 meters
<CMD:ENABLE_ALTITUDE_TEST:threshold=1500>   # Monitor at 15.0 meters
<CMD:ENABLE_ALTITUDE_TEST:threshold=100>    # Monitor at 1.0 meter
```

**Behavior:**
- Enables continuous background altitude monitoring
- Triggers buzzer and servo when altitude threshold is reached
- Remains active until explicitly disabled
- Default threshold: 200cm (2.0m) if parameter omitted

#### DISABLE_ALTITUDE_TEST (Disable background monitoring)
**Format:** `<CMD:DISABLE_ALTITUDE_TEST>`

**Parameters:** None

**Example:**
```
<CMD:DISABLE_ALTITUDE_TEST>
```

**Behavior:**
- Disables background altitude monitoring
- Stops all altitude-triggered actions

#### CONTROL (Device control - RECOGNIZED BUT NOT IMPLEMENTED)
**Format:** `<CMD:CONTROL:param1=value1,param2=value2>`

**Note:** This command is parsed and acknowledged but has no functional implementation in the current firmware.

**Theoretical Parameters:**
- `servo` - Servo position in degrees (0-180)
- `buzzer` - Buzzer state (0=off, 1=on)

**Examples:**
```
<CMD:CONTROL:servo=45>              # Set servo to 45 degrees
<CMD:CONTROL:buzzer=1>              # Turn buzzer on
<CMD:CONTROL:servo=90,buzzer=1>     # Set servo and buzzer
```

### Command Format Rules

1. **Encapsulation:** All commands must be enclosed in `< >` brackets
2. **Prefix:** All commands start with `CMD:`
3. **Parameters:** Separated by colons `:` from command name
4. **Key-Value Pairs:** Use `=` to assign values: `key=value`
5. **Multiple Parameters:** Separate with commas: `param1=value1,param2=value2`
6. **Case Sensitivity:** Command names are case-sensitive
7. **No Spaces:** Avoid spaces in command strings

### Parameter Validation

#### ALTITUDE_TEST Parameters
- **threshold:** Must be 1-30000 centimeters (0.01m to 300m)
- Invalid values will result in `<CMD_ACK:ERR:INVALID_PARAMS>`

#### ENABLE_ALTITUDE_TEST Parameters  
- **threshold:** Must be ≥1 centimeter (no upper limit for background monitoring)
- Invalid values will result in `<CMD_ACK:ERR:INVALID_PARAMS>`

#### CONTROL Parameters (if implemented)
- **servo:** Must be 0-180 degrees
- **buzzer:** Must be 0 (off) or 1 (on)
- Invalid values will result in `<CMD_ACK:ERR:INVALID_PARAMS>`

### Response Examples

**Successful Commands:**
```
Command: <CMD:ALTITUDE_TEST:threshold=500>
Response: <CMD_ACK:OK:ALTITUDE_TEST_SUCCESS>

Command: <CMD:ENABLE_ALTITUDE_TEST:threshold=1000>
Response: <CMD_ACK:OK:ALTITUDE_TEST_ENABLED>

Command: <CMD:ARM>
Response: <CMD_ACK:OK:ARMED>

Command: <CMD:DISARM>
Response: <CMD_ACK:OK:IDLE>

Command: <CMD:QUERY>
Response: <CMD_ACK:OK:ARMED> (or current state)
```

**Failed Commands:**
```
Command: <CMD:ALTITUDE_TEST:threshold=50000>
Response: <CMD_ACK:ERR:INVALID_PARAMS>

Command: <CMD:ENABLE_ALTITUDE_TEST:threshold=0>
Response: <CMD_ACK:ERR:INVALID_PARAMS>

Command: <CMD:ARM> (when already armed)
Response: <CMD_ACK:ERR:ALREADY_ARMED>

Command: <CMD:ENTER_TEST> (when not in IDLE)
Response: <CMD_ACK:ERR:MUST_BE_IDLE>

Command: <CMD:ENTER_RECOVERY> (when not in ARMED)
Response: <CMD_ACK:ERR:MUST_BE_ARMED>

Command: Invalid format
Response: <CMD_ACK:ERR:INVALID_FORMAT>

Command: Parse error
Response: <CMD_ACK:ERR:PARSE_ERROR>

Command: Checksum error
Response: <CMD_ACK:ERR:CHECKSUM_ERROR>

Command: General failure
Response: <CMD_ACK:ERR:DENIED>
```

**Complete Success Response List:**
- `<CMD_ACK:OK:IDLE>` - DISARM command
- `<CMD_ACK:OK:ARMED>` - ARM command
- `<CMD_ACK:OK:TEST>` - ENTER_TEST command
- `<CMD_ACK:OK:RECOVERY>` - ENTER_RECOVERY command
- `<CMD_ACK:OK:TEST_SUCCESS>` - TEST command
- `<CMD_ACK:OK:[STATE]>` - QUERY command (returns current state)
- `<CMD_ACK:OK:CONTROL_APPLIED>` - CONTROL command (recognized but not implemented)
- `<CMD_ACK:OK:NAVC_STATS_RESET>` - NAVC_RESET_STATS command
- `<CMD_ACK:OK:SERVO_TEST_SUCCESS>` - SERVO_TEST command
- `<CMD_ACK:OK:ALTITUDE_TEST_SUCCESS>` - ALTITUDE_TEST command
- `<CMD_ACK:OK:ALTITUDE_TEST_ENABLED>` - ENABLE_ALTITUDE_TEST command
- `<CMD_ACK:OK:ALTITUDE_TEST_DISABLED>` - DISABLE_ALTITUDE_TEST command

## Data Types & Scaling Reference

| Sensor | Raw Type | Scaling Factor | Physical Unit | Pre-Filter |
|--------|----------|----------------|---------------|------------|
| **Accelerometer** | int16 | ÷1000 | g-force | EMA α=0.2 |
| **Gyroscope** | int16 | ÷100 | degrees/sec | EMA α=0.2 |
| **Magnetometer** | int16 | ÷10 | microTesla | EMA α=0.1 |
| **Barometer** | float | 1.0 | meters | EMA α=0.05 |
| **GPS Coordinates** | int32 | ÷10⁷ | degrees | No filter |
| **Temperature** | int16 | 1.0 | °Celsius | No filter |
| **Timestamp** | uint32 | 1.0 | milliseconds | No filter |

**Note**: All IMU sensors (accelerometer, gyroscope, magnetometer) and barometric altitude are pre-filtered with Exponential Moving Average (EMA) filters to reduce noise. GPS and temperature data are unfiltered.

## Link Quality & Telemetry Rate Adaptation

### RSSI-Based Rate Control
- **Strong Signal** (RSSI > -70dBm): 10Hz telemetry rate
- **Good Signal** (-70 to -85dBm): 5Hz telemetry rate  
- **Weak Signal** (-85 to -100dBm): 2Hz telemetry rate
- **Poor Signal** (< -100dBm): 1Hz telemetry rate

### Data Validation Guidelines

#### GPS Validity Check
```python
def is_gps_valid(lat_1e7, lon_1e7, satellites):
    return (satellites >= 4 and 
            abs(lat_1e7) > 100000 and  # > 0.01 degrees
            abs(lon_1e7) > 100000)
```

#### IMU Range Validation
- **Acceleration**: Valid range ±20g (±20000mg)
- **Gyroscope**: Valid range ±2000°/s (±200000 centidegrees/s)
- **Magnetometer**: Earth's field ~25-65µT (250-650 decisla)

#### Connection Status
- **Telemetry Timeout**: >2 seconds indicates communication loss
- **Invalid GPS**: Coordinates near 0.0000001° indicate no GPS fix
- **Sensor Fault**: All magnetometer values = 0 indicates sensor failure

## Communication Specifications

### Ground Station USB Interface
- **Baud Rate**: 921600
- **Data Format**: ASCII, newline-terminated
- **Flow Control**: None
- **Timeout**: 2 seconds for telemetry, 5 seconds for commands

### LoRa RF Link (FC ↔ GS)
- **Frequency**: 915.0MHz (configurable)
- **Modulation**: LoRa (SF7, BW500kHz, CR4/5)
- **Max Payload**: 128 bytes
- **Power**: 20dBm default (0-20dBm configurable)
- **Sync Word**: 0xAB
- **Preamble Length**: 6 symbols

### UART Link (NAVC ↔ FC)
- **Baud Rate**: 115200
- **Data Format**: Binary packets with CRC16
- **Packet Rate**: 50Hz sensor data stream
- **Packet Size**: 44 bytes + 4 bytes framing = 48 bytes total
