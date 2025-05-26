Looking at the NAVC codebase and the SD card implementation instructions, I'll help you create a refined version of these instructions that fits the existing codebase structure and fixes any logic errors.

Here's the updated SD card implementation guide for the NAVC:

# NAVC SD Card Logging - Implementation Instructions (Revised)

## Prerequisites
Based on the existing NAVC codebase:
- The NAVC already has SD card support in `platformio.ini` (SD library included)
- RTC is already initialized and working (DS3231)
- NeoPixel LED is already configured and working
- Serial2 is used for FC communication, so SD card will use SPI pins

## Phase 1: Foundation Setup (Non-Disruptive)

### Step 1.1: Add SD Card Pin Definition and Data Structures
**Location**: Add to `include/navc/Sensors.h` or create new `include/navc/SDLogger.h`

```cpp
// SD Card configuration
#define SD_CS_PIN PA4  // Choose appropriate CS pin for your hardware
#define PACKET_BUFFER_SIZE 20
#define SD_FILENAME_MAX_LEN 50
#define LED_BLINK_DURATION_MS 50
#define SD_POLL_INTERVAL_MS 1000  // Increased from 100ms to reduce overhead

// Packet buffer for SD logging
typedef struct {
    char data[250];  // Sized for full telemetry packet string
    uint32_t timestamp_ms;
} PacketEntry;

typedef struct {
    PacketEntry packets[PACKET_BUFFER_SIZE];
    volatile uint8_t head;
    volatile uint8_t tail;
    volatile uint8_t count;
} PacketBuffer;
```

### Step 1.2: Create SDLogger Class
**Location**: Create new files `include/navc/SDLogger.h` and `src/navc/SDLogger.cpp`

```cpp
// include/navc/SDLogger.h
#ifndef SDLOGGER_H
#define SDLOGGER_H

#include <Arduino.h>
#include <SD.h>
#include <RTClib.h>

class SDLogger {
private:
    PacketBuffer packet_buffer;
    bool sd_card_present;
    bool logging_active;
    char current_log_filename[SD_FILENAME_MAX_LEN];
    uint32_t last_sd_poll_ms;
    uint32_t packets_logged;
    File log_file;
    
    // RTC reference from SensorManager
    RTC_DS3231& rtc;
    
public:
    SDLogger(RTC_DS3231& rtc_ref);
    bool begin();
    void update();
    bool addPacket(const char* packet_data);
    bool isLogging() const { return logging_active; }
    uint32_t getPacketsLogged() const { return packets_logged; }
    
private:
    void checkSDCard();
    bool initializeSDCard();
    void generateLogFilename(char* filename, size_t size);
    bool openLogFile();
    void closeLogFile();
    void processWrites();
};

#endif
```

### Step 1.3: Update NAVC.cpp to Include SDLogger
**Location**: `src/navc/NAVC.cpp`

Add after other includes:
```cpp
#include "../include/navc/SDLogger.h"

// Add global SDLogger instance after SensorManager
SDLogger* sdLogger = nullptr;  // Initialize as nullptr for safety
```

In `setup()`, after sensor initialization:
```cpp
// Initialize SD card logger
sdLogger = new SDLogger(sensorManager.rtc);
if (sdLogger->begin()) {
    Serial.println("<DEBUG:SD_LOGGER_INITIALIZED>");
} else {
    Serial.println("<DEBUG:SD_LOGGER_INIT_FAILED>");
}
```

**STOP HERE - Test Phase 1**
*Verify: Code compiles, no functionality changes, device boots normally*

---

## Phase 2: SD Card Basic Operations

### Step 2.1: Implement SDLogger Basic Functions
**Location**: `src/navc/SDLogger.cpp`

```cpp
#include "../include/navc/SDLogger.h"

SDLogger::SDLogger(RTC_DS3231& rtc_ref) : 
    rtc(rtc_ref),
    sd_card_present(false),
    logging_active(false),
    last_sd_poll_ms(0),
    packets_logged(0) {
    
    // Initialize packet buffer
    memset(&packet_buffer, 0, sizeof(packet_buffer));
    memset(current_log_filename, 0, sizeof(current_log_filename));
}

bool SDLogger::begin() {
    // Initialize SD card pins
    pinMode(SD_CS_PIN, OUTPUT);
    digitalWrite(SD_CS_PIN, HIGH);
    
    // Try to initialize SD card
    if (SD.begin(SD_CS_PIN)) {
        sd_card_present = true;
        Serial.println("<DEBUG:SD_CARD_FOUND>");
        return true;
    }
    
    Serial.println("<DEBUG:SD_CARD_NOT_FOUND>");
    return false;
}

void SDLogger::update() {
    checkSDCard();
    
    if (logging_active) {
        processWrites();
    }
}

void SDLogger::checkSDCard() {
    uint32_t current_time = millis();
    
    if (current_time - last_sd_poll_ms < SD_POLL_INTERVAL_MS) {
        return;
    }
    
    last_sd_poll_ms = current_time;
    
    // Check if card is still present by trying to open root
    File root = SD.open("/");
    bool card_detected = (root ? true : false);
    if (root) root.close();
    
    if (card_detected != sd_card_present) {
        sd_card_present = card_detected;
        
        if (card_detected) {
            Serial.println("<DEBUG:SD_CARD_INSERTED>");
            if (initializeSDCard()) {
                openLogFile();
            }
        } else {
            Serial.println("<DEBUG:SD_CARD_REMOVED>");
            closeLogFile();
        }
    }
}

bool SDLogger::initializeSDCard() {
    // Re-initialize SD card
    if (!SD.begin(SD_CS_PIN)) {
        return false;
    }
    
    Serial.println("<DEBUG:SD_CARD_INITIALIZED>");
    return true;
}

void SDLogger::generateLogFilename(char* filename, size_t size) {
    DateTime now = rtc.now();
    
    // Format: MM_DD_YYYY_FLIGHT_DATA_N.txt
    char date_str[12];
    snprintf(date_str, sizeof(date_str), "%02d_%02d_%04d", 
             now.month(), now.day(), now.year());
    
    // Find next available file number
    int file_number = 1;
    do {
        snprintf(filename, size, "%s_FLIGHT_DATA_%d.txt", 
                date_str, file_number);
        file_number++;
    } while (SD.exists(filename) && file_number < 100);
}
```

### Step 2.2: Add File Operations
**Location**: Continue in `src/navc/SDLogger.cpp`

```cpp
bool SDLogger::openLogFile() {
    if (!sd_card_present) {
        return false;
    }
    
    generateLogFilename(current_log_filename, sizeof(current_log_filename));
    
    log_file = SD.open(current_log_filename, FILE_WRITE);
    if (log_file) {
        // Write header
        log_file.println("# NAVC Flight Data Log");
        log_file.print("# Started: ");
        
        DateTime now = rtc.now();
        char timestamp[25];
        snprintf(timestamp, sizeof(timestamp), "%02d/%02d/%04d %02d:%02d:%02d",
                now.month(), now.day(), now.year(),
                now.hour(), now.minute(), now.second());
        log_file.println(timestamp);
        log_file.println("# Format: MM/DD/YYYY,HH:MM:SS,altitude,accelX,accelY,accelZ,gyroX,gyroY,gyroZ,magX,magY,magZ,lat,lon,sats,temp");
        log_file.flush();
        
        logging_active = true;
        packets_logged = 0;
        
        Serial.print("<DEBUG:LOG_FILE_OPENED:");
        Serial.print(current_log_filename);
        Serial.println(">");
        
        return true;
    }
    
    return false;
}

void SDLogger::closeLogFile() {
    if (log_file) {
        log_file.close();
        logging_active = false;
        
        Serial.print("<DEBUG:LOG_FILE_CLOSED:PACKETS=");
        Serial.print(packets_logged);
        Serial.println(">");
    }
}
```

**STOP HERE - Test Phase 2**
*Verify: SD card detection works, files are created with correct names*

---

## Phase 3: Packet Buffering System

### Step 3.1: Implement Circular Buffer
**Location**: Add to `src/navc/SDLogger.cpp`

```cpp
bool SDLogger::addPacket(const char* packet_data) {
    // Use volatile access for thread safety
    uint8_t next_head = (packet_buffer.head + 1) % PACKET_BUFFER_SIZE;
    
    if (next_head == packet_buffer.tail) {
        // Buffer full
        return false;
    }
    
    // Copy packet data
    strncpy(packet_buffer.packets[packet_buffer.head].data, 
            packet_data, 
            sizeof(packet_buffer.packets[packet_buffer.head].data) - 1);
    packet_buffer.packets[packet_buffer.head].data[sizeof(packet_buffer.packets[packet_buffer.head].data) - 1] = '\0';
    packet_buffer.packets[packet_buffer.head].timestamp_ms = millis();
    
    // Update head
    packet_buffer.head = next_head;
    packet_buffer.count++;
    
    return true;
}

void SDLogger::processWrites() {
    if (!logging_active || packet_buffer.count == 0) {
        return;
    }
    
    // Process one packet per call to avoid blocking
    if (packet_buffer.tail != packet_buffer.head) {
        // Get packet from buffer
        char* packet_data = packet_buffer.packets[packet_buffer.tail].data;
        
        // Write to SD card
        if (log_file) {
            log_file.println(packet_data);
            log_file.flush();  // Ensure data is written
            packets_logged++;
            
            // Trigger LED blink here (in Phase 6)
        }
        
        // Update tail
        packet_buffer.tail = (packet_buffer.tail + 1) % PACKET_BUFFER_SIZE;
        packet_buffer.count--;
    }
}
```

### Step 3.2: Integrate with NAVC Telemetry
**Location**: Modify `src/navc/NAVC.cpp`

In the main loop, after `if (sensorManager.isPacketReady())`:
```cpp
// Get the latest packet
SensorPacket packet = sensorManager.getPacket();

// Calculate and set CRC
packet.crc16 = calculateCrc16(
    (const uint8_t*)&packet, 
    sizeof(SensorPacket) - sizeof(uint16_t)
);

// Format packet for SD logging (if enabled)
if (sdLogger && sdLogger->isLogging()) {
    // Format telemetry in CSV format
    char logBuffer[250];
    char timestamp[25];
    
    // Create timestamp from RTC data in packet
    snprintf(timestamp, sizeof(timestamp), "%02d/%02d/20%02d,%02d:%02d:%02d",
             packet.month, packet.day, packet.year,
             packet.hour, packet.minute, packet.second);
    
    // Format as CSV: timestamp,altitude,accelX,accelY,accelZ,gyroX,gyroY,gyroZ,magX,magY,magZ,lat,lon,sats,temp
    snprintf(logBuffer, sizeof(logBuffer), 
             "%s,%ld,%d,%d,%d,%d,%d,%d,%d,%d,%d,%ld,%ld,%d,%d",
             timestamp,
             packet.altitude,
             packet.accelX, packet.accelY, packet.accelZ,
             packet.gyroX, packet.gyroY, packet.gyroZ,
             packet.magX, packet.magY, packet.magZ,
             packet.latitude, packet.longitude,
             packet.satellites,
             packet.temperature);
    
    sdLogger->addPacket(logBuffer);
}

// Continue with existing telemetry code...
```

Also add to main loop:
```cpp
// Update SD logger
if (sdLogger) {
    sdLogger->update();
}
```

**STOP HERE - Test Phase 3**
*Verify: Packets are buffered, no impact on telemetry timing*

---

## Phase 4: Non-Blocking Write System

The write system is already implemented in Phase 3's `processWrites()` function, which processes one packet per call to avoid blocking.

### Step 4.1: Add Write Performance Monitoring
**Location**: Add to `src/navc/SDLogger.cpp`

```cpp
void SDLogger::processWrites() {
    if (!logging_active || packet_buffer.count == 0) {
        return;
    }
    
    // Track write timing
    uint32_t write_start = micros();
    
    // Process one packet per call to avoid blocking
    if (packet_buffer.tail != packet_buffer.head) {
        // Get packet from buffer
        char* packet_data = packet_buffer.packets[packet_buffer.tail].data;
        
        // Write to SD card
        if (log_file) {
            log_file.println(packet_data);
            log_file.flush();  // Ensure data is written
            packets_logged++;
            
            // Update tail
            packet_buffer.tail = (packet_buffer.tail + 1) % PACKET_BUFFER_SIZE;
            packet_buffer.count--;
            
            // Check write time
            uint32_t write_duration = micros() - write_start;
            if (write_duration > 5000) {  // More than 5ms
                Serial.print("<DEBUG:SLOW_SD_WRITE:");
                Serial.print(write_duration);
                Serial.println("us>");
            }
        }
    }
}
```

**STOP HERE - Test Phase 4**
*Verify: Data is written to files, no timing impact*

---

## Phase 5: Hot-Swap Handling

Already implemented in Phase 2's `checkSDCard()` function. The system handles insertion/removal gracefully.

### Step 5.1: Add Buffer Management on Card Changes
**Location**: Update `src/navc/SDLogger.cpp`

In `checkSDCard()`, when card is removed:
```cpp
if (!card_detected && sd_card_present) {
    Serial.println("<DEBUG:SD_CARD_REMOVED>");
    closeLogFile();
    
    // Clear buffer on card removal
    packet_buffer.head = 0;
    packet_buffer.tail = 0;
    packet_buffer.count = 0;
}
```

**STOP HERE - Test Phase 5**
*Verify: Hot-swap works correctly*

---

## Phase 6: Visual Feedback

### Step 6.1: Add LED Feedback
**Location**: Modify `src/navc/SDLogger.cpp`

Add member variables to SDLogger class:
```cpp
private:
    uint32_t led_blink_start_ms;
    bool led_blink_active;
    SensorManager* sensor_manager;  // For LED access
```

Update constructor and begin():
```cpp
SDLogger::SDLogger(RTC_DS3231& rtc_ref, SensorManager* sm) : 
    rtc(rtc_ref),
    sensor_manager(sm),
    // ... other initializations
```

In `processWrites()`, after successful write:
```cpp
// Trigger LED blink
led_blink_start_ms = millis();
led_blink_active = true;
if (sensor_manager) {
    sensor_manager->setStatusLED(50, 20, 0);  // Dim orange
}
```

Add LED update method:
```cpp
void SDLogger::updateLED() {
    if (led_blink_active) {
        uint32_t current_time = millis();
        if (current_time - led_blink_start_ms >= LED_BLINK_DURATION_MS) {
            // Turn off LED
            if (sensor_manager) {
                sensor_manager->setStatusLED(0, 0, 0);
            }
            led_blink_active = false;
        }
    }
}
```

Call `updateLED()` from `update()` method.

**STOP HERE - Test Phase 6**
*Verify: Orange LED blinks on writes*

---

## Phase 7: Final Integration

### Step 7.1: Add Status Reporting
**Location**: Add to `src/navc/NAVC.cpp`

```cpp
// In reportStatus() or similar periodic function:
if (sdLogger) {
    static uint32_t last_sd_report = 0;
    uint32_t current_time = millis();
    
    if (current_time - last_sd_report >= 10000) {  // Every 10 seconds
        last_sd_report = current_time;
        
        if (sdLogger->isLogging()) {
            Serial.print("<DEBUG:SD_LOGGING:ACTIVE,PACKETS=");
            Serial.print(sdLogger->getPacketsLogged());
            Serial.println(">");
        } else {
            Serial.println("<DEBUG:SD_LOGGING:INACTIVE>");
        }
    }
}
```

**Final Testing**
*Perform comprehensive system testing as outlined in original Phase 7*

## Key Improvements Made:

1. **Better Integration**: Created a proper SDLogger class that integrates cleanly with the existing NAVC architecture
2. **Fixed Buffer Safety**: Used volatile variables and proper circular buffer implementation
3. **Proper Error Handling**: SD card operations are wrapped with proper error checking
4. **Non-Blocking Design**: One packet per write cycle ensures no timing disruption
5. **Clean Hot-Swap**: Card removal/insertion is handled gracefully with buffer management
6. **Reuses Existing Components**: Uses the existing RTC and LED infrastructure
7. **Proper CSV Format**: Matches the telemetry format expected by ground station

This implementation maintains the existing NAVC functionality while adding robust SD card logging capabilities.