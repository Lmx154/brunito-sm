/**
 * FrameCodec.h - Message framing and encoding utilities
 * 
 * This utility handles the ASCII framing protocol used by the Brunito system.
 * Format: <TYPE:PAYLOAD>
 */

#ifndef FRAME_CODEC_H
#define FRAME_CODEC_H

#include <Arduino.h>

class FrameCodec {
private:
    static const int MAX_FRAME_LENGTH = 128;
    char frameBuffer[MAX_FRAME_LENGTH];
    
    // CRC16-CCITT calculation 
    static uint16_t calculateCRC16(const uint8_t* data, size_t length) {
        uint16_t crc = 0xFFFF;
        
        for (size_t i = 0; i < length; i++) {
            crc ^= (uint16_t)data[i] << 8;
            
            for (int j = 0; j < 8; j++) {
                if (crc & 0x8000) {
                    crc = (crc << 1) ^ 0x1021;
                } else {
                    crc = crc << 1;
                }
            }
        }
        
        return crc;
    }
    
public:
    // Format a debug message
    static void formatDebug(char* buffer, size_t bufferSize, const char* message) {
        snprintf(buffer, bufferSize, "<DEBUG:%s>", message);
    }
    
    // Format a status message
    static void formatStatus(char* buffer, size_t bufferSize, const char* state, unsigned long timestamp) {
        snprintf(buffer, bufferSize, "<STATUS:%s,%lu>", state, timestamp);
    }
    
    // Format a command acknowledgment
    static void formatCmdAck(char* buffer, size_t bufferSize, bool success, const char* message = nullptr) {
        if (success) {
            if (message) {
                snprintf(buffer, bufferSize, "<CMD_ACK:OK:%s>", message);
            } else {
                snprintf(buffer, bufferSize, "<CMD_ACK:OK>");
            }
        } else {
            if (message) {
                snprintf(buffer, bufferSize, "<CMD_ACK:ERR:%s>", message);
            } else {
                snprintf(buffer, bufferSize, "<CMD_ACK:ERR>");
            }
        }
    }    // Format a telemetry message in ARMED state
    static void formatArmedTelemetry(char* buffer, size_t bufferSize, 
                                   uint16_t pkId, uint8_t year, uint8_t month, uint8_t day, 
                                   uint8_t hour, uint8_t minute, uint8_t second,
                                   int32_t alt, int16_t accelX, int16_t accelY, int16_t accelZ,
                                   int16_t gyroX, int16_t gyroY, int16_t gyroZ,
                                   int16_t magX, int16_t magY, int16_t magZ,
                                   int32_t lat, int32_t lon, uint8_t sats, int16_t temp) {
        char datetime[22]; // MM/DD/YYYY,HH:MM:SS
        snprintf(datetime, sizeof(datetime), "%02u/%02u/20%02u,%02u:%02u:%02u", 
                 month, day, year, hour, minute, second);        snprintf(buffer, bufferSize, "<%s,%ld,%d,%d,%d,%d,%d,%d,%d,%d,%d,%ld,%ld,%u,%d>",
                datetime, alt, accelX, accelY, accelZ, gyroX, gyroY, gyroZ,
                magX, magY, magZ, lat, lon, sats, temp);
    }
      // Format a telemetry message in RECOVERY state (GPS only)
    static void formatRecoveryTelemetry(char* buffer, size_t bufferSize,
                                     uint8_t year, uint8_t month, uint8_t day, 
                                     uint8_t hour, uint8_t minute, uint8_t second,
                                     int32_t lat, int32_t lon, int32_t alt, 
                                     uint8_t sats, int16_t temp) {
        char datetime[22]; // MM/DD/YYYY,HH:MM:SS
        snprintf(datetime, sizeof(datetime), "%02u/%02u/20%02u,%02u:%02u:%02u", 
                 month, day, year, hour, minute, second);
        snprintf(buffer, bufferSize, "<%s,%ld,%ld,%ld,%u,%d>", datetime, lat, lon, alt, sats, temp);
    }
      // Calculate CRC16 for binary packet
    static uint16_t calculatePacketCRC(const uint8_t* packet, size_t length) {
        return calculateCRC16(packet, length - 2);  // Exclude the CRC bytes
    }
      // Verify CRC16 in a binary packet
    static bool verifyPacketCRC(const uint8_t* packet, size_t length) {
        uint16_t calculatedCRC = calculateCRC16(packet, length - 2);
        uint16_t packetCRC = (packet[length - 2] << 8) | packet[length - 1];
        return calculatedCRC == packetCRC;
    }
    
    // Helper for NAVC sensor packet CRC calculation
    static uint16_t calculateSensorPacketCRC(const uint8_t* packet, size_t length) {
        // CRC16-CCITT (0xFFFF) for sensor packet (excluding the CRC field)
        return calculateCRC16(packet, length - 2);
    }
};

#endif // FRAME_CODEC_H
