# Brunito Telemetry Communication Protocol

## Overview

This document provides a comprehensive guide to the telemetry communication protocol used in the Brunito project. The system consists of three main modules:

- **NAVC (Navigation Controller)**: Handles sensor fusion and data collection
- **FC (Flight Controller)**: Manages flight operations and state transitions
- **GS (Ground Station)**: Acts as a bridge between the flight system and ground computer

The Ground Station (GS) is the primary interface for computer-based telemetry parsing and command transmission, as it's the only module directly connected to the computer via USB.

## System Architecture

```
[NAVC] --UART--> [FC] --LoRa--> [GS] --USB--> [Computer]
```

### Communication Links
- **NAVC ↔ FC**: UART (115200 baud) - Binary sensor packets at 50Hz
- **FC ↔ GS**: LoRa (433MHz) - ASCII telemetry packets at 1-10Hz adaptive rate
- **GS ↔ Computer**: USB CDC (921600 baud) - ASCII telemetry and commands

## Data Structures

### 1. NAVC Sensor Packet (Binary - 46 bytes)

The core sensor data structure transmitted from NAVC to FC:

```cpp
struct SensorPacket {
    uint32_t timestamp;        // Milliseconds since boot (4 bytes)
    
    // Accelerometer data (12 bytes) - Pre-filtered with EMA α=0.2
    float accel_x;            // m/s² (4 bytes)
    float accel_y;            // m/s² (4 bytes)
    float accel_z;            // m/s² (4 bytes)
    
    // Gyroscope data (12 bytes) - Pre-filtered with EMA α=0.2
    float gyro_x;             // rad/s (4 bytes)
    float gyro_y;             // rad/s (4 bytes)
    float gyro_z;             // rad/s (4 bytes)
    
    // Magnetometer data (12 bytes) - Pre-filtered with EMA α=0.1
    float mag_x;              // µT (4 bytes)
    float mag_y;              // µT (4 bytes)
    float mag_z;              // µT (4 bytes)
    
    // Barometric data (4 bytes) - Pre-filtered with EMA α=0.05
    float pressure;           // hPa (4 bytes)
    
    // CRC validation (2 bytes)
    uint16_t crc16;           // CRC16-CCITT checksum (2 bytes)
};
```

**Total Size**: 46 bytes  
**Transmission Rate**: 50Hz (NAVC → FC)  
**Validation**: CRC16-CCITT with polynomial 0x1021, initial value 0xFFFF

#### 📊 **Important: Pre-Applied Low-Pass Filtering**

**All sensor data in the telemetry stream has been pre-processed with Exponential Moving Average (EMA) low-pass filters** to reduce MEMS sensor noise and improve data quality for real-time flight control applications.

**Filter Parameters**:
- **Accelerometer & Gyroscope**: α = 0.2 (moderate smoothing, preserves responsiveness for flight control)
- **Magnetometer**: α = 0.1 (higher smoothing due to EMI sensitivity)  
- **Barometer**: α = 0.05 (heavy smoothing for stable altitude readings)

**EMA Formula**: `filtered_value = α × raw_value + (1-α) × previous_filtered_value`

**Implications for Software Development**:
- Data is already noise-reduced and suitable for direct use in flight algorithms
- No additional low-pass filtering is typically needed in ground station software
- Higher-frequency noise components (>5-10Hz) have been significantly attenuated
- Original sensor noise characteristics are not present in the telemetry data
- Filter introduces minimal phase lag while preserving signal dynamics important for flight control

### 2. LoRa Packet Structure (FC ↔ GS)

LoRa packets use ASCII format with structured headers:

```
[PACKET_TYPE:DATA_PAYLOAD]
```

**Maximum LoRa Payload**: 255 bytes  
**Transmission Rate**: Adaptive 1-10Hz based on flight state

## Ground Station (GS) Protocol

### USB Communication Format

The GS communicates with the computer via USB CDC at 921600 baud using ASCII protocols.

### Telemetry Output Formats

#### 1. ARMED State Telemetry (Full Sensor Data)
**Format**: Comma-separated values enclosed in angle brackets
```
<MM/DD/YYYY,HH:MM:SS,altitude,accelX,accelY,accelZ,gyroX,gyroY,gyroZ,magX,magY,magZ,latitude,longitude,satellites,temperature>
```

**Real Example**:
```
<05/27/2025,11:43:46,0.95,-37,-967,-3,128,-27,204,6,-53,20,1,1,0,24>
```

**Field-by-Field Breakdown**:
| Position | Field | Example Value | Units/Scaling | Description |
|----------|-------|---------------|---------------|-------------|
| 1 | Date | `05/27/2025` | MM/DD/YYYY | Current date from RTC |
| 2 | Time | `11:43:46` | HH:MM:SS | Current time from RTC |
| 3 | Altitude | `0.95` | meters | Barometric altitude (raw value ÷ 100) |
| 4 | AccelX | `-37` | mg | X-axis acceleration (milli-g) |
| 5 | AccelY | `-967` | mg | Y-axis acceleration (milli-g) |
| 6 | AccelZ | `-3` | mg | Z-axis acceleration (milli-g) |
| 7 | GyroX | `128` | 0.01°/s | X-axis rotation rate (centidegrees/sec) |
| 8 | GyroY | `-27` | 0.01°/s | Y-axis rotation rate (centidegrees/sec) |
| 9 | GyroZ | `204` | 0.01°/s | Z-axis rotation rate (centidegrees/sec) |
| 10 | MagX | `6` | 0.1µT | X-axis magnetic field (decisla) |
| 11 | MagY | `-53` | 0.1µT | Y-axis magnetic field (decisla) |
| 12 | MagZ | `20` | 0.1µT | Z-axis magnetic field (decisla) |
| 13 | Latitude | `1` | degrees×10⁷ | GPS latitude (divide by 10,000,000) |
| 14 | Longitude | `1` | degrees×10⁷ | GPS longitude (divide by 10,000,000) |
| 15 | Satellites | `0` | count | Number of GPS satellites in view |
| 16 | Temperature | `24` | °C | Temperature in degrees Celsius |

**Value Conversions for Application Use**:
- **Altitude**: `0.95` meters (raw value already converted)
- **Acceleration**: X=-37mg, Y=-967mg, Z=-3mg (1000mg = 1g = 9.81m/s²)
- **Gyroscope**: X=1.28°/s, Y=-0.27°/s, Z=2.04°/s (divide by 100)
- **Magnetometer**: X=0.6µT, Y=-5.3µT, Z=2.0µT (divide by 10)
- **GPS**: Lat=0.0000001°, Lon=0.0000001° (divide by 10,000,000)

#### 2. RECOVERY State Telemetry (GPS-Only Data)
**Format**: Reduced data set for recovery tracking
```
<MM/DD/YYYY,HH:MM:SS,latitude,longitude,altitude,satellites,temperature>
```

**Example**:
```
<05/27/2025,11:43:46,123456789,-456789012,125.50,8,23>
```

**Field-by-Field Breakdown**:
| Position | Field | Example Value | Units/Scaling | Description |
|----------|-------|---------------|---------------|-------------|
| 1 | Date | `05/27/2025` | MM/DD/YYYY | Current date from RTC |
| 2 | Time | `11:43:46` | HH:MM:SS | Current time from RTC |
| 3 | Latitude | `123456789` | degrees×10⁷ | GPS latitude (12.3456789°) |
| 4 | Longitude | `-456789012` | degrees×10⁷ | GPS longitude (-45.6789012°) |
| 5 | Altitude | `125.50` | meters | Barometric altitude |
| 6 | Satellites | `8` | count | Number of GPS satellites |
| 7 | Temperature | `23` | °C | Temperature in degrees Celsius |

#### 3. Test Mode Telemetry
```
<TEST:ALT:125.50m,ACCEL:-0.037,0.967,-0.003>
```

**Purpose**: Diagnostic output for sensor validation

### Status Messages

#### Debug Messages
```
DEBUG:<category>:<message>
```

**Examples**:
```
DEBUG:LORA:Packet received, RSSI=-67dBm
DEBUG:GPS:Fix acquired, satellites=8
DEBUG:STATE:Transition from IDLE to ARMED
DEBUG:BATTERY:Voltage=7.2V, Level=85%
```

#### Error Messages
```
ERROR:<category>:<error_description>
```

**Examples**:
```
ERROR:LORA:Communication timeout
ERROR:GPS:Signal lost
ERROR:SENSOR:IMU calibration failed
```

#### System Status
```
STATUS:<component>:<status_info>
```

**Examples**:
```
STATUS:GS:Ready, LoRa connected
STATUS:BATTERY:7.4V (90%)
STATUS:GPS:8 satellites, HDOP=1.2
```

### Command Protocol

#### Command Format
Commands sent to the GS use bracketed format:
```
<CMD:COMMAND_NAME[:PARAMETERS]>
```

#### Available Commands

##### 1. System Commands
```
<CMD:PING>                    // Test connectivity
<CMD:RESET>                   // Reset GS system
<CMD:STATUS>                  // Request status report
<CMD:VERSION>                 // Get firmware version
```

##### 2. LoRa Commands
```
<CMD:LORA_FREQ:433000000>     // Set LoRa frequency (Hz)
<CMD:LORA_POWER:14>           // Set TX power (0-20 dBm)
<CMD:LORA_BW:125000>          // Set bandwidth (Hz)
<CMD:LORA_SF:7>               // Set spreading factor (6-12)
```

##### 3. Flight Commands (Relayed to FC)
```
<CMD:ARM>                     // Arm the flight system
<CMD:DISARM>                  // Disarm the flight system
<CMD:ABORT>                   // Emergency abort
<CMD:RECOVERY>                // Force recovery mode
<CMD:CALIBRATE>               // Start sensor calibration
```

##### 4. Data Commands
```
<CMD:START_LOG>               // Start data logging
<CMD:STOP_LOG>                // Stop data logging
<CMD:CLEAR_LOG>               // Clear log files
<CMD:DOWNLOAD_LOG>            // Download log data
```

#### Command Responses

##### Success Responses
```
ACK:<command_name>[:additional_info]
```

**Examples**:
```
ACK:PING:GS_Ready
ACK:ARM:Flight_system_armed
ACK:LORA_FREQ:Set_to_433000000Hz
```

##### Error Responses
```
NAK:<command_name>:<error_reason>
```

**Examples**:
```
NAK:ARM:System_not_ready
NAK:LORA_FREQ:Invalid_frequency
NAK:UNKNOWN_COMMAND:Command_not_recognized
```

## Flight Controller (FC) Protocol

### State Management

The FC manages the following states:
- `IDLE`: Ground idle state
- `ARMED`: Ready for flight
- `ASCENT`: Active flight upward
- `DESCENT`: Controlled descent
- `RECOVERY`: Recovery mode (parachute deployed)
- `LANDED`: Safe landing detected

### Telemetry Generation

#### ARMED State Telemetry (10Hz)
```cpp
// Full telemetry packet
sprintf(buffer, "TELEM:T=%lu,LAT=%.6f,LON=%.6f,ALT=%.1f,VEL=%.1f,STATE=%s",
        timestamp, latitude, longitude, altitude, velocity, state_name);
```

#### RECOVERY State Telemetry (1Hz)
```cpp
// GPS-only telemetry for recovery
sprintf(buffer, "RECOVERY:T=%lu,LAT=%.6f,LON=%.6f,ALT=%.1f",
        timestamp, latitude, longitude, altitude);
```

### UART Communication (FC ↔ NAVC)

#### Commands to NAVC
```
<START_TELEMETRY>             // Enable sensor data streaming
<STOP_TELEMETRY>              // Disable sensor data streaming
<PING>                        // Test NAVC connectivity
<RESET_STATS>                 // Reset packet statistics
<USB_DEBUG_ON/OFF>            // Control debug output
<ECHO:message>                // Echo test command
```

#### NAVC Responses
```
<PONG>                        // Response to PING
<ACK:command_name>            // Command acknowledged
<NAK:error_reason>            // Command failed
```

## Navigation Controller (NAVC) Protocol

### Sensor Data Streaming

#### Binary Packet Format
- **Protocol**: Custom binary with CRC16 validation
- **Rate**: 50Hz continuous when telemetry enabled
- **Transport**: UART (115200 baud) with hardware flow control
- **Data Processing**: All sensor values are pre-filtered with EMA low-pass filters before transmission

#### Frame Structure
```
[START_BYTE][LENGTH][SENSOR_PACKET][CRC16][END_BYTE]
```

### USB Debug Output

The NAVC provides extensive debug information via USB CDC:

```
<DEBUG:category:message>
```

#### Debug Categories
- `SENSOR_INIT`: Sensor initialization status
- `I2C_ERROR`: I2C communication issues
- `PACKET_STATS`: Transmission statistics
- `SD_LOGGING`: SD card logging status
- `COMMAND_RECEIVED`: Incoming commands
- `RESPONSE_SENT`: Outgoing responses

## Data Parsing Guidelines

### Complete Field Reference for Applications

When building applications to interface with the Brunito flight computer, you'll primarily work with CSV telemetry strings. Here's everything you need to know:

⚠️ **Critical Note**: All sensor data (accelerometer, gyroscope, magnetometer, barometer) has been **pre-filtered with EMA low-pass filters** at the NAVC level. The values in telemetry packets are already noise-reduced and do not require additional filtering for most applications.

#### Primary Data Format
**Input String**: `<05/27/2025,11:43:46,0.95,-37,-967,-3,128,-27,204,6,-53,20,1,1,0,24>`

#### Field Extraction (Zero-indexed after removing < and >)
```python
def parse_brunito_telemetry(data_line):
    # Remove brackets and split by comma
    clean_data = data_line.strip('<>')
    fields = clean_data.split(',')
    
    # Parse datetime (fields 0-1)
    date_str = fields[0]  # MM/DD/YYYY
    time_str = fields[1]  # HH:MM:SS
    datetime_obj = datetime.strptime(f"{date_str},{time_str}", "%m/%d/%Y,%H:%M:%S")
    
    # Parse sensor data (fields 2-15)
    parsed_data = {
        'timestamp': datetime_obj,
        'altitude_m': float(fields[2]),                    # Already in meters
        'accel_x_mps2': int(fields[3]) * 0.001 * 9.81,   # mg to m/s²
        'accel_y_mps2': int(fields[4]) * 0.001 * 9.81,   # mg to m/s²
        'accel_z_mps2': int(fields[5]) * 0.001 * 9.81,   # mg to m/s²
        'gyro_x_dps': int(fields[6]) / 100.0,             # 0.01°/s to °/s
        'gyro_y_dps': int(fields[7]) / 100.0,             # 0.01°/s to °/s
        'gyro_z_dps': int(fields[8]) / 100.0,             # 0.01°/s to °/s
        'mag_x_ut': int(fields[9]) / 10.0,                # 0.1µT to µT
        'mag_y_ut': int(fields[10]) / 10.0,               # 0.1µT to µT
        'mag_z_ut': int(fields[11]) / 10.0,               # 0.1µT to µT
        'latitude_deg': int(fields[12]) / 10000000.0,     # 1e-7 degrees to degrees
        'longitude_deg': int(fields[13]) / 10000000.0,    # 1e-7 degrees to degrees
        'gps_satellites': int(fields[14]),                # Satellite count
        'temperature_c': int(fields[15])                  # Temperature in °C
    }
    
    return parsed_data
```

#### Expected Value Ranges for Validation
| Parameter | Minimum | Maximum | Invalid Value | Notes |
|-----------|---------|---------|---------------|-------|
| Altitude | -1000m | +50000m | > 50000 | Barometric altitude (EMA filtered, α=0.05) |
| Acceleration | -20g | +20g | > 200 m/s² | Total acceleration (EMA filtered, α=0.2) |
| Gyroscope | -2000°/s | +2000°/s | > 2000°/s | Angular rates (EMA filtered, α=0.2) |
| Magnetometer | -100µT | +100µT | All zeros | Earth's field ~25-65µT (EMA filtered, α=0.1) |
| Latitude | -90° | +90° | 0.0000000° | GPS coordinate |
| Longitude | -180° | +180° | 0.0000000° | GPS coordinate |
| Satellites | 0 | 12 | >12 | GPS satellites in view |
| Temperature | -40°C | +85°C | <-50°C or >100°C | Operating range |

**Filter Effects on Data Quality**:
- Sensor noise and high-frequency vibrations are significantly reduced
- Rapid transients are smoothed but major motion events are preserved
- Data appears more stable compared to raw MEMS sensor output
- No additional filtering required for visualization or basic flight analysis

#### Data Quality Indicators
```python
def assess_data_quality(parsed_data):
    quality = {
        'gps_valid': parsed_data['gps_satellites'] >= 4 and 
                    parsed_data['latitude_deg'] != 0.0 and 
                    parsed_data['longitude_deg'] != 0.0,
        'imu_valid': abs(parsed_data['accel_x_mps2']) < 200 and
                    abs(parsed_data['accel_y_mps2']) < 200 and
                    abs(parsed_data['accel_z_mps2']) < 200,
        'mag_valid': not (parsed_data['mag_x_ut'] == 0 and 
                         parsed_data['mag_y_ut'] == 0 and 
                         parsed_data['mag_z_ut'] == 0),
        'temp_valid': -40 <= parsed_data['temperature_c'] <= 85
    }
    return quality
```

#### Real-Time Data Rates
- **ARMED State**: 1-10Hz adaptive (typical 5-8Hz)
- **RECOVERY State**: 1Hz fixed
- **TEST State**: 0.5Hz

#### Missing Data Handling
- **GPS coordinates = 0**: No GPS fix available
- **Magnetometer all zeros**: Sensor not initialized or failed
- **Temperature extremes**: Sensor fault or environmental limits
- **Excessive acceleration**: Possible sensor saturation

### Flight State Detection
```python
def detect_flight_phase(altitude_history, accel_history):
    """
    Detect flight phase based on telemetry data
    """
    current_alt = altitude_history[-1]
    alt_rate = (altitude_history[-1] - altitude_history[-5]) / 5.0  # m/s over 5 samples
    accel_mag = sqrt(sum(a**2 for a in accel_history[-1]))
    
    if alt_rate > 10:
        return "ASCENT"
    elif alt_rate < -5 and current_alt > 100:
        return "DESCENT"
    elif accel_mag < 2 and current_alt < 50:
        return "RECOVERY"
    else:
        return "ARMED"
```

## Error Handling and Validation

### CRC16 Validation (Binary Packets)
```cpp
uint16_t calculateCrc16(const uint8_t* data, size_t length) {
    uint16_t crc = 0xFFFF;
    
    for (size_t i = 0; i < length; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    
    return crc;
}
```

### Timeout Handling
- **Command Response Timeout**: 5 seconds
- **Telemetry Timeout**: 2 seconds (indicates communication loss)
- **LoRa Packet Timeout**: 10 seconds (adaptive retry)

### Data Validation
- Latitude: -90.0 to +90.0 degrees
- Longitude: -180.0 to +180.0 degrees
- Altitude: -1000 to +50000 meters
- Velocity: 0 to 500 m/s
- Timestamp: Monotonically increasing

## Example Implementation

### Complete Python Application for Brunito Telemetry

```python
import serial
import time
import csv
from datetime import datetime
from math import sqrt

class BrunitoTelemetryParser:
    def __init__(self, port, baudrate=921600):
        self.serial = serial.Serial(port, baudrate, timeout=1)
        self.data_callbacks = []
        self.debug_callbacks = []
        self.last_telemetry = None
        
    def add_data_callback(self, callback):
        """Add callback for processed telemetry data"""
        self.data_callbacks.append(callback)
        
    def add_debug_callback(self, callback):
        """Add callback for debug messages"""
        self.debug_callbacks.append(callback)
        
    def parse_telemetry_line(self, line):
        """Parse a complete telemetry line from GS output"""
        line = line.strip()
        
        # Handle telemetry data (most common)
        if line.startswith('<') and line.endswith('>'):
            return self.parse_sensor_data(line)
        
        # Handle debug/status messages
        elif line.startswith('<DEBUG:') or line.startswith('<STATUS:'):
            for callback in self.debug_callbacks:
                callback(line)
            return None
            
        return None
    
    def parse_sensor_data(self, data_line):
        """Parse the main sensor telemetry format"""
        try:
            # Remove brackets and split
            clean_data = data_line.strip('<>')
            fields = clean_data.split(',')
            
            if len(fields) == 16:  # ARMED state format
                parsed = self.parse_armed_telemetry(fields)
            elif len(fields) == 7:   # RECOVERY state format  
                parsed = self.parse_recovery_telemetry(fields)
            else:
                return None
                
            # Validate data quality
            parsed['quality'] = self.assess_data_quality(parsed)
            
            # Call all registered callbacks
            for callback in self.data_callbacks:
                callback(parsed)
                
            self.last_telemetry = parsed
            return parsed
            
        except (ValueError, IndexError) as e:
            print(f"Parse error: {e} for line: {data_line}")
            return None
    
    def parse_armed_telemetry(self, fields):
        """Parse full ARMED state telemetry"""
        # Parse datetime
        date_str = fields[0]  # MM/DD/YYYY
        time_str = fields[1]  # HH:MM:SS
        timestamp = datetime.strptime(f"{date_str},{time_str}", "%m/%d/%Y,%H:%M:%S")
        
        return {
            'mode': 'ARMED',
            'timestamp': timestamp,
            'altitude_m': float(fields[2]),
            'accel_x_g': int(fields[3]) / 1000.0,         # mg to g
            'accel_y_g': int(fields[4]) / 1000.0,
            'accel_z_g': int(fields[5]) / 1000.0,
            'gyro_x_dps': int(fields[6]) / 100.0,         # centidegrees/s to degrees/s
            'gyro_y_dps': int(fields[7]) / 100.0,
            'gyro_z_dps': int(fields[8]) / 100.0,
            'mag_x_ut': int(fields[9]) / 10.0,            # 0.1µT to µT
            'mag_y_ut': int(fields[10]) / 10.0,
            'mag_z_ut': int(fields[11]) / 10.0,
            'latitude_deg': int(fields[12]) / 10000000.0, # 1e-7 degrees to degrees
            'longitude_deg': int(fields[13]) / 10000000.0,
            'gps_satellites': int(fields[14]),
            'temperature_c': int(fields[15]),
            # Computed values
            'accel_magnitude_g': sqrt(sum(float(fields[i])**2 for i in [3,4,5])) / 1000.0,
            'gyro_magnitude_dps': sqrt(sum(int(fields[i])**2 for i in [6,7,8])) / 100.0
        }
    
    def parse_recovery_telemetry(self, fields):
        """Parse RECOVERY state GPS-only telemetry"""
        date_str = fields[0]
        time_str = fields[1] 
        timestamp = datetime.strptime(f"{date_str},{time_str}", "%m/%d/%Y,%H:%M:%S")
        
        return {
            'mode': 'RECOVERY',
            'timestamp': timestamp,
            'latitude_deg': int(fields[2]) / 10000000.0,
            'longitude_deg': int(fields[3]) / 10000000.0,
            'altitude_m': float(fields[4]),
            'gps_satellites': int(fields[5]),
            'temperature_c': int(fields[6])
        }
    
    def assess_data_quality(self, data):
        """Assess quality of telemetry data"""
        quality = {
            'gps_valid': False,
            'imu_valid': False,
            'mag_valid': False,
            'temp_valid': False
        }
        
        # GPS validation (if present)
        if 'latitude_deg' in data and 'longitude_deg' in data:
            quality['gps_valid'] = (
                data['gps_satellites'] >= 4 and
                abs(data['latitude_deg']) > 0.0001 and
                abs(data['longitude_deg']) > 0.0001 and
                -90 <= data['latitude_deg'] <= 90 and
                -180 <= data['longitude_deg'] <= 180
            )
        
        # IMU validation (if present)
        if 'accel_magnitude_g' in data:
            quality['imu_valid'] = (
                0.5 <= data['accel_magnitude_g'] <= 20 and  # Reasonable acceleration range
                abs(data['gyro_magnitude_dps']) <= 2000     # Reasonable rotation rate
            )
        
        # Magnetometer validation (if present)
        if 'mag_x_ut' in data:
            mag_magnitude = sqrt(data['mag_x_ut']**2 + data['mag_y_ut']**2 + data['mag_z_ut']**2)
            quality['mag_valid'] = 20 <= mag_magnitude <= 80  # Earth's magnetic field range
        
        # Temperature validation
        quality['temp_valid'] = -40 <= data['temperature_c'] <= 85
        
        return quality
    
    def send_command(self, command, parameters=None):
        """Send command to flight computer via Ground Station"""
        if parameters:
            cmd_str = f"<CMD:{command}:{parameters}>\n"
        else:
            cmd_str = f"<CMD:{command}>\n"
        
        self.serial.write(cmd_str.encode())
        
        # Wait for response (5 second timeout)
        start_time = time.time()
        while time.time() - start_time < 5:
            if self.serial.in_waiting:
                response = self.serial.readline().decode().strip()
                if response.startswith("ACK:"):
                    return True, response
                elif response.startswith("NAK:"):
                    return False, response
        
        return False, "TIMEOUT"
    
    def start_data_logging(self, filename=None):
        """Start logging telemetry to CSV file"""
        if filename is None:
            filename = f"brunito_telemetry_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        
        self.log_file = open(filename, 'w', newline='')
        self.csv_writer = csv.writer(self.log_file)
        
        # Write header
        self.csv_writer.writerow([
            'timestamp', 'mode', 'altitude_m', 'latitude_deg', 'longitude_deg',
            'accel_x_g', 'accel_y_g', 'accel_z_g', 'accel_mag_g',
            'gyro_x_dps', 'gyro_y_dps', 'gyro_z_dps', 'gyro_mag_dps',
            'mag_x_ut', 'mag_y_ut', 'mag_z_ut', 
            'gps_satellites', 'temperature_c',
            'gps_valid', 'imu_valid', 'mag_valid', 'temp_valid'
        ])
        
        def log_callback(data):
            row = [
                data['timestamp'].isoformat(),
                data['mode'],
                data.get('altitude_m', ''),
                data.get('latitude_deg', ''),
                data.get('longitude_deg', ''),
                data.get('accel_x_g', ''),
                data.get('accel_y_g', ''),
                data.get('accel_z_g', ''),
                data.get('accel_magnitude_g', ''),
                data.get('gyro_x_dps', ''),
                data.get('gyro_y_dps', ''),
                data.get('gyro_z_dps', ''),
                data.get('gyro_magnitude_dps', ''),
                data.get('mag_x_ut', ''),
                data.get('mag_y_ut', ''),
                data.get('mag_z_ut', ''),
                data.get('gps_satellites', ''),
                data.get('temperature_c', ''),
                data['quality']['gps_valid'],
                data['quality']['imu_valid'],
                data['quality']['mag_valid'],
                data['quality']['temp_valid']
            ]
            self.csv_writer.writerow(row)
            self.log_file.flush()
        
        self.add_data_callback(log_callback)
        return filename
    
    def run(self):
        """Main parsing loop"""
        print("Brunito Telemetry Parser started...")
        while True:
            try:
                if self.serial.in_waiting:
                    line = self.serial.readline().decode('utf-8', errors='ignore')
                    self.parse_telemetry_line(line)
                time.sleep(0.01)  # Small delay to prevent excessive CPU usage
            except KeyboardInterrupt:
                print("Stopping telemetry parser...")
                break
            except Exception as e:
                print(f"Parse error: {e}")

# Usage Example
if __name__ == "__main__":
    # Create parser instance
    parser = BrunitoTelemetryParser('COM3')  # Adjust port as needed
    
    # Add callback for real-time data display
    def display_telemetry(data):
        if data['mode'] == 'ARMED':
            print(f"ALT: {data['altitude_m']:6.2f}m  "
                  f"GPS: {data['latitude_deg']:8.5f},{data['longitude_deg']:8.5f}  "
                  f"ACCEL: {data['accel_magnitude_g']:5.2f}g  "
                  f"SATS: {data['gps_satellites']}")
        elif data['mode'] == 'RECOVERY':
            print(f"RECOVERY - GPS: {data['latitude_deg']:8.5f},{data['longitude_deg']:8.5f}  "
                  f"ALT: {data['altitude_m']:6.2f}m")
    
    def display_debug(message):
        print(f"DEBUG: {message}")
    
    parser.add_data_callback(display_telemetry)
    parser.add_debug_callback(display_debug)
    
    # Start logging to file
    log_filename = parser.start_data_logging()
    print(f"Logging to: {log_filename}")
    
    # Send ARM command (optional)
    success, response = parser.send_command('ARM')
    print(f"ARM command: {'SUCCESS' if success else 'FAILED'} - {response}")
    
    # Start main loop
    parser.run()
```

## Troubleshooting

### Common Issues

1. **No Telemetry Data**
   - Check USB connection to GS
   - Verify LoRa communication between FC and GS
   - Ensure FC is in ARMED state for full telemetry

2. **Invalid GPS Coordinates**
   - GPS may not have fix (wait for satellite acquisition)
   - Check for GPS antenna connection
   - Verify GPS module power supply

3. **Command Not Acknowledged**
   - Check command syntax (must be exact)
   - Verify LoRa communication range
   - Ensure FC is in appropriate state for command

4. **High Packet Loss**
   - Check LoRa signal strength (RSSI values)
   - Verify antenna connections
   - Consider reducing transmission rate

### Debug Commands
```
<CMD:STATUS>          // Get system status
<CMD:PING>            // Test connectivity
<CMD:VERSION>         // Get firmware versions
```

## Revision History

| Version | Date | Changes |
|---------|------|---------|
| 1.0 | 2025-05-27 | Initial documentation |
| 1.1 | 2025-01-27 | Updated to reflect EMA low-pass filtering implementation on all MEMS sensors |

---

*This document covers the complete telemetry communication protocol for the Brunito project. For hardware setup information, see HWSETUP.md. For command reference, see COMMANDS.md.*
