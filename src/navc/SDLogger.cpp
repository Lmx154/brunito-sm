#include "../include/navc/SDLogger.h"
#include "../include/navc/Sensors.h"

SDLogger::SDLogger(RTC_DS3231& rtc_ref, SensorManager* sm) : 
    rtc(rtc_ref),
    sensor_manager(sm),
    sd_card_present(false),
    logging_active(false),
    last_sd_poll_ms(0),
    packets_logged(0),
    led_blink_start_ms(0),
    led_blink_active(false) {
    
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
        
        // Automatically create a log file when SD card is first detected
        openLogFile();
        
        return true;
    }
    
    return false;
}

void SDLogger::update() {
    checkSDCard();
    
    if (logging_active) {
        processWrites();
    }
    
    updateLED();
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
            if (initializeSDCard()) {
                openLogFile();
            }
        } else {
            closeLogFile();
            
            // Clear buffer on card removal
            packet_buffer.head = 0;
            packet_buffer.tail = 0;
            packet_buffer.count = 0;
        }
    }
}

bool SDLogger::initializeSDCard() {
    // Re-initialize SD card
    if (!SD.begin(SD_CS_PIN)) {
        return false;
    }
    
    return true;
}

void SDLogger::generateLogFilename(char* filename, size_t size) {
    DateTime now = rtc.now();
    
    // Use 8.3 compatible filename format: LOGXXXXX.TXT (8 chars + 3 extension)
    // Format: LOG + 5-digit number (LOG00001.TXT to LOG99999.TXT)
    
    // Find next available file number
    int file_number = 1;
    do {
        snprintf(filename, size, "LOG%05d.TXT", file_number);
        file_number++;
    } while (SD.exists(filename) && file_number < 100000);
}

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
        
        return true;
    }
    
    return false;
}

void SDLogger::closeLogFile() {
    if (log_file) {
        log_file.close();
        logging_active = false;
    }
}

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

bool SDLogger::addSensorPacket(const SensorPacket& packet) {
    // Format sensor packet as CSV
    char csvBuffer[250];
    formatSensorPacketCSV(packet, csvBuffer, sizeof(csvBuffer));
    
    // Add to packet buffer
    return addPacket(csvBuffer);
}

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
              // Trigger LED blink
            led_blink_start_ms = millis();
            led_blink_active = true;
            if (sensor_manager) {
                sensor_manager->setStatusLED(50, 20, 0);  // Dim orange
            }
            
            // Update tail
            packet_buffer.tail = (packet_buffer.tail + 1) % PACKET_BUFFER_SIZE;
            packet_buffer.count--;
            
            // Check write time for performance monitoring
            uint32_t write_duration = micros() - write_start;
            if (write_duration > 5000) {  // More than 5ms
                Serial.print("<DEBUG:SLOW_SD_WRITE:");
                Serial.print(write_duration);
                Serial.println("us>");
            }        }
    }
}

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

void SDLogger::formatSensorPacketCSV(const SensorPacket& packet, char* buffer, size_t bufferSize) {
    // Create timestamp from RTC data in packet
    char timestamp[25];
    snprintf(timestamp, sizeof(timestamp), "%02d/%02d/20%02d,%02d:%02d:%02d",
             packet.month, packet.day, packet.year,
             packet.hour, packet.minute, packet.second);
    
    // Format as CSV: timestamp,altitude,accelX,accelY,accelZ,gyroX,gyroY,gyroZ,magX,magY,magZ,lat,lon,sats,temp
    // Convert altitude from cm to meters with 2 decimal places
    float altitudeMeters = packet.altitude / 100.0f;
    
    snprintf(buffer, bufferSize, 
             "%s,%.2f,%d,%d,%d,%d,%d,%d,%d,%d,%d,%ld,%ld,%d,%d",
             timestamp,
             altitudeMeters,
             packet.accelX, packet.accelY, packet.accelZ,
             packet.gyroX, packet.gyroY, packet.gyroZ,
             packet.magX, packet.magY, packet.magZ,
             packet.latitude, packet.longitude,
             packet.satellites,
             packet.temperature);
}
