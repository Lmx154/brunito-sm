#include "../include/navc/SDLogger.h"
#include "../include/navc/Sensors.h"

SDLogger::SDLogger(RTC_DS3231& rtc_ref, SensorManager* sm) : 
    rtc(rtc_ref),
    sensor_manager(sm),
    sd_card_present(false),
    logging_active(false),
    sensors_ready(false),
    sensor_ready_time_ms(0),
    last_sd_poll_ms(0),
    last_card_change_ms(0),
    packets_logged(0),
    led_blink_start_ms(0),
    led_blink_active(false),
    led_status_update_ms(0) {
    
    // Initialize packet buffer
    memset(&packet_buffer, 0, sizeof(packet_buffer));
    memset(current_log_filename, 0, sizeof(current_log_filename));
}

bool SDLogger::begin() {
    // Initialize SD card pins
    pinMode(SD_CS_PIN, OUTPUT);
    digitalWrite(SD_CS_PIN, HIGH);    // Try to initialize SD card
    if (SD.begin(SD_CS_PIN)) {
        sd_card_present = true;
        
        // Do NOT automatically create a log file here
        // Wait for sensors to be ready and stabilized
        Serial.println("<DEBUG:SD_CARD_FOUND>");
        
        return true;
    }
      Serial.println("<DEBUG:SD_CARD_NOT_FOUND>");
    return false;
}

void SDLogger::setSensorsReady() {
    if (!sensors_ready) {
        sensors_ready = true;
        sensor_ready_time_ms = millis();
        Serial.println("<DEBUG:SD_LOGGER_SENSORS_READY>");
        Serial.print("<DEBUG:SD_LOGGER_STABILIZATION_WAIT:");
        Serial.print(SENSOR_STABILIZATION_DELAY_MS / 1000);
        Serial.println("s>");
    }
}

void SDLogger::update() {
    checkSDCard();
    
    // Check if we should start logging (after sensors are stable)
    if (sensors_ready && sd_card_present && !logging_active) {
        uint32_t current_time = millis();
        if (current_time - sensor_ready_time_ms >= SENSOR_STABILIZATION_DELAY_MS) {
            // Sensors have been ready and stable for the required time
            if (openLogFile()) {
                Serial.println("<DEBUG:SD_LOGGING_STARTED_AFTER_STABILIZATION>");
            }
        }
    }
    
    if (logging_active) {
        processWrites();
    }
    
    updateLED();
    updateStatusLED();
}

void SDLogger::checkSDCard() {
    uint32_t current_time = millis();
    
    // Use faster polling for 10 seconds after a card change is detected
    uint32_t poll_interval = SD_POLL_INTERVAL_MS;
    if (current_time - last_card_change_ms < 10000) {
        poll_interval = SD_POLL_INTERVAL_FAST_MS;
    }
    
    if (current_time - last_sd_poll_ms < poll_interval) {
        return;
    }
    
    last_sd_poll_ms = current_time;
      // More robust card detection - try to perform an actual read operation
    bool card_detected = false;
    
    // Method 1: Try to re-initialize SD card (most reliable for detecting removal)
    if (SD.begin(SD_CS_PIN)) {
        // Method 2: Try to read the actual volume info or create a small test operation
        File testFile = SD.open("/");
        if (testFile) {
            testFile.close();
            
            // Method 3: If we're currently logging, test if we can still write to our log file
            if (logging_active && log_file) {
                // Check if the log file is still valid by trying to get its position
                size_t pos = log_file.position();
                if (pos != (size_t)-1) {
                    card_detected = true;
                } else {
                    // File position failed, card likely removed
                    Serial.println("<DEBUG:SD_CARD_FILE_POSITION_FAILED>");
                }
            } else {
                // Not currently logging, so basic directory access is sufficient
                card_detected = true;
            }
        }
    }
  if (card_detected != sd_card_present) {
        sd_card_present = card_detected;
        last_card_change_ms = current_time;  // Record when change occurred for fast polling
        
        if (card_detected) {
            Serial.println("<DEBUG:SD_CARD_INSERTED>");
            if (initializeSDCard()) {
                Serial.println("<DEBUG:SD_CARD_REINITIALIZED>");
                // Do NOT automatically start logging - wait for sensors to be ready and stable
                // The main update() loop will handle starting logging when conditions are met
            } else {
                Serial.println("<DEBUG:SD_CARD_REINIT_FAILED>");
            }} else {
            Serial.println("<DEBUG:SD_CARD_REMOVED>");
            closeLogFile();
            
            // Report packets lost due to card removal (before clearing buffer)
            if (packet_buffer.count > 0) {
                Serial.print("<DEBUG:SD_PACKETS_LOST:");
                Serial.print(packet_buffer.count);
                Serial.println(">");
            }
            
            // Clear buffer on card removal to prevent data loss/corruption
            packet_buffer.head = 0;
            packet_buffer.tail = 0;
            packet_buffer.count = 0;
            
            Serial.println("<DEBUG:SD_BUFFER_CLEARED>");
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
  }
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

void SDLogger::updateStatusLED() {
    // Only update status LED if no blink is active and we have sensor manager access
    if (!led_blink_active && sensor_manager) {
        uint32_t current_time = millis();
        
        // Update status LED every 2 seconds
        if (current_time - led_status_update_ms >= 2000) {
            led_status_update_ms = current_time;
            
            if (!sd_card_present) {
                // No SD card - slow red pulse
                static bool red_state = false;
                red_state = !red_state;
                if (red_state) {
                    sensor_manager->setStatusLED(20, 0, 0);  // Dim red
                } else {
                    sensor_manager->setStatusLED(0, 0, 0);   // Off
                }
            } else if (!logging_active) {
                // SD card present but not logging - slow blue pulse
                static bool blue_state = false;
                blue_state = !blue_state;
                if (blue_state) {
                    sensor_manager->setStatusLED(0, 0, 20);  // Dim blue
                } else {
                    sensor_manager->setStatusLED(0, 0, 0);   // Off
                }
            } else {
                // Actively logging - solid green
                sensor_manager->setStatusLED(0, 20, 0);     // Dim green
            }
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
    // Manual conversion to avoid float formatting issues on STM32
    char altStr[16];
    
    // Convert altitude from cm to meters with 2 decimal places using integer math
    int32_t altitudeCm = packet.altitude;
    int32_t altitudeWhole = altitudeCm / 100;  // Whole meters
    int32_t altitudeFrac = abs(altitudeCm % 100);  // Fractional part (always positive)
    
    // Format altitude manually to avoid float conversion issues
    if (altitudeCm < 0 && altitudeWhole == 0) {
        // Special case for negative values between -0.99 and 0
        snprintf(altStr, sizeof(altStr), "-0.%02ld", (long)altitudeFrac);
    } else {
        snprintf(altStr, sizeof(altStr), "%ld.%02ld", (long)altitudeWhole, (long)altitudeFrac);
    }
  
    
    snprintf(buffer, bufferSize, 
             "%s,%s,%d,%d,%d,%d,%d,%d,%d,%d,%d,%ld,%ld,%d,%d",
             timestamp,
             altStr,  // Use the manually formatted altitude string
             packet.accelX, packet.accelY, packet.accelZ,
             packet.gyroX, packet.gyroY, packet.gyroZ,
             packet.magX, packet.magY, packet.magZ,
             packet.latitude, packet.longitude,
             packet.satellites,
             packet.temperature);
}