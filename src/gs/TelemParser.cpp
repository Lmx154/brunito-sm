#include "../include/gs/TelemParser.h"
#include <Arduino.h>

TelemParser::TelemParser() : headerSent(false) {
}

void TelemParser::reset() {
    headerSent = false;
}

void TelemParser::processTelemetryFrame(const char* frame) {
    // Check if this is a telemetry frame
    if (strncmp(frame, "<TELEM:", 7) != 0) {
        return; // Not a telemetry frame
    }
    
    // Variables to store parsed data
    uint16_t packetId;
    uint32_t timestamp;
    int32_t alt, lat, lon;
    int16_t accelX, accelY, accelZ;
    int16_t gyroX, gyroY, gyroZ;
    int16_t magX, magY, magZ;
    
    // Try to parse as standard telemetry frame
    if (parseTelemetryFrame(frame, packetId, timestamp, alt, accelX, accelY, accelZ, 
                           gyroX, gyroY, gyroZ, magX, magY, magZ, lat, lon)) {
        
        // If header hasn't been sent yet, send it
        if (!headerSent) {
            Serial.println("# Telemetry Data CSV");
            Serial.println("packet_id,timestamp_ms,altitude_cm,accel_x_mg,accel_y_mg,accel_z_mg,gyro_x_01dps,gyro_y_01dps,gyro_z_01dps,mag_x_01uT,mag_y_01uT,mag_z_01uT,latitude_1e7,longitude_1e7");
            headerSent = true;
        }
        
        // Format and print CSV data
        char csvBuffer[128];
        snprintf(csvBuffer, sizeof(csvBuffer), 
                 "%u,%lu,%ld,%d,%d,%d,%d,%d,%d,%d,%d,%d,%ld,%ld",
                 packetId, timestamp, alt, accelX, accelY, accelZ,
                 gyroX, gyroY, gyroZ, magX, magY, magZ, lat, lon);
        
        Serial.println(csvBuffer);
        return;
    }
    
    // Try to parse as recovery telemetry frame
    if (parseRecoveryFrame(frame, timestamp, lat, lon, alt)) {
        // If header hasn't been sent yet, send it
        if (!headerSent) {
            Serial.println("# Recovery Telemetry Data CSV");
            Serial.println("timestamp_ms,latitude_1e7,longitude_1e7,altitude_cm");
            headerSent = true;
        }
        
        // Format and print CSV data
        char csvBuffer[128];
        snprintf(csvBuffer, sizeof(csvBuffer), "%lu,%ld,%ld,%ld", timestamp, lat, lon, alt);
        
        Serial.println(csvBuffer);
    }
}

bool TelemParser::parseTelemetryFrame(const char* frame, uint16_t& packetId, uint32_t& timestamp,
                                    int32_t& alt, int16_t& accelX, int16_t& accelY, int16_t& accelZ,
                                    int16_t& gyroX, int16_t& gyroY, int16_t& gyroZ,
                                    int16_t& magX, int16_t& magY, int16_t& magZ,
                                    int32_t& lat, int32_t& lon) {
    
    // Format: <TELEM:packetId,timestamp,alt,accelX,accelY,accelZ,gyroX,gyroY,gyroZ,magX,magY,magZ,lat,lon>
    
    // Count commas to check if this is a full telemetry frame
    int commaCount = 0;
    for (int i = 0; frame[i] != '\0' && frame[i] != '>'; i++) {
        if (frame[i] == ',') commaCount++;
    }
    
    // Full telemetry has 13 commas (14 fields)
    if (commaCount != 13) {
        return false;
    }
    
    // Parse the frame
    int result = sscanf(frame, "<TELEM:%hu,%lu,%ld,%hd,%hd,%hd,%hd,%hd,%hd,%hd,%hd,%hd,%ld,%ld>",
                       &packetId, &timestamp, &alt,
                       &accelX, &accelY, &accelZ,
                       &gyroX, &gyroY, &gyroZ,
                       &magX, &magY, &magZ,
                       &lat, &lon);
    
    return (result == 14); // All 14 fields must be successfully parsed
}

bool TelemParser::parseRecoveryFrame(const char* frame, uint32_t& timestamp, 
                                   int32_t& lat, int32_t& lon, int32_t& alt) {
    
    // Format: <TELEM:timestamp,lat,lon,alt>
    
    // Count commas to check if this is a recovery telemetry frame
    int commaCount = 0;
    for (int i = 0; frame[i] != '\0' && frame[i] != '>'; i++) {
        if (frame[i] == ',') commaCount++;
    }
    
    // Recovery telemetry has 3 commas (4 fields)
    if (commaCount != 3) {
        return false;
    }
    
    // Parse the frame
    int result = sscanf(frame, "<TELEM:%lu,%ld,%ld,%ld>",
                       &timestamp, &lat, &lon, &alt);
    
    return (result == 4); // All 4 fields must be successfully parsed
}
