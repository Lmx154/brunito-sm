#include "../include/fc/UartManager.h"

UartManager::UartManager() : 
    receiveBufferIndex(0),
    packetSize(sizeof(SensorPacket)),
    packetReady(false),
    packetsReceived(0),
    packetsDropped(0),
    crcErrors(0),
    lastPacketTime(0)
{
    // Initialize buffer
    memset(receiveBuffer, 0, sizeof(receiveBuffer));
    memset(&latestPacket, 0, sizeof(latestPacket));
}

UartPacketStatus UartManager::processUartData() {
    // Read available bytes from UART
    while (Serial2.available() > 0) {
        // Check if buffer has space
        if (receiveBufferIndex >= sizeof(receiveBuffer)) {
            // Buffer overflow, reset and increment error counter
            receiveBufferIndex = 0;
            packetsDropped++;
            return PACKET_ERROR;
        }
        
        // Read one byte and add to buffer
        receiveBuffer[receiveBufferIndex++] = Serial2.read();
        
        // Check if we have a complete packet
        if (receiveBufferIndex >= packetSize) {
            // Copy data to packet structure
            memcpy(&latestPacket, receiveBuffer, packetSize);
            
            // Reset buffer index for next packet
            receiveBufferIndex = 0;
            
            // Verify packet CRC
            if (verifyCrc16(latestPacket)) {
                // Valid packet received
                packetsReceived++;
                lastPacketTime = millis();
                packetReady = true;
                return PACKET_COMPLETE;
            } else {
                // CRC error
                crcErrors++;
                return PACKET_ERROR;
            }
        }
    }
    
    // No complete packet yet
    return PACKET_NONE;
}

bool UartManager::isPacketReady() const {
    return packetReady;
}

const SensorPacket& UartManager::getLatestPacket() const {
    return latestPacket;
}

void UartManager::markPacketProcessed() {
    packetReady = false;
}

uint32_t UartManager::getPacketsReceived() const {
    return packetsReceived;
}

uint32_t UartManager::getPacketsDropped() const {
    return packetsDropped;
}

uint32_t UartManager::getCrcErrors() const {
    return crcErrors;
}

float UartManager::getPacketRate() const {
    // Calculate packets per second
    unsigned long runtime = millis();
    if (runtime == 0) return 0.0f;
    
    return (float)packetsReceived / (runtime / 1000.0f);
}

unsigned long UartManager::getTimeSinceLastPacket() const {
    if (lastPacketTime == 0) return 0;
    return millis() - lastPacketTime;
}

void UartManager::resetStats() {
    packetsReceived = 0;
    packetsDropped = 0;
    crcErrors = 0;
}

bool UartManager::sendCommand(const char* cmd) {
    // Send command to NAVC over Serial2
    Serial2.println(cmd);
    return true;
}

bool UartManager::verifyCrc16(const SensorPacket& packet) {
    // Calculate CRC-16 for packet data (excluding the CRC itself)
    uint16_t calculatedCrc = calculateCrc16(
        reinterpret_cast<const uint8_t*>(&packet),
        sizeof(SensorPacket) - sizeof(uint16_t)
    );
    
    // Compare with the CRC in the packet
    return calculatedCrc == packet.crc16;
}

uint16_t UartManager::calculateCrc16(const uint8_t* data, size_t length) {
    // CRC16-CCITT (0xFFFF) implementation
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
