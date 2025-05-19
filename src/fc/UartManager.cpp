#include "../include/fc/UartManager.h"
#include "../include/utils/FrameCodec.h"

UartManager::UartManager() : 
    receiveBufferIndex(0),
    examinedBufferIndex(0),
    examinedBufferSize(0),
    packetSize(sizeof(SensorPacket)),
    packetReady(false),
    packetsReceived(0),
    packetsDropped(0),
    crcErrors(0),
    missedPackets(0),
    lastPacketId(0),
    firstPacket(true),
    lastPacketTime(0)
{
    // Initialize buffers
    memset(receiveBuffer, 0, sizeof(receiveBuffer));
    memset(examinedBuffer, 0, sizeof(examinedBuffer));
    memset(&latestPacket, 0, sizeof(latestPacket));
}

UartPacketStatus UartManager::processUartData() {
    // First, process any bytes that were examined but not consumed yet
    while (examinedBufferIndex < examinedBufferSize) {
        // Check if buffer has space
        if (receiveBufferIndex >= sizeof(receiveBuffer)) {
            // Buffer overflow, reset and increment error counter
            receiveBufferIndex = 0;
            packetsDropped++;
            return PACKET_ERROR;
        }
        
        // Process one examined byte
        uint8_t currentByte = examinedBuffer[examinedBufferIndex++];
        receiveBuffer[receiveBufferIndex++] = currentByte;
    }
    
    // If we've processed all examined bytes, reset those indices
    if (examinedBufferIndex >= examinedBufferSize) {
        examinedBufferIndex = 0;
        examinedBufferSize = 0;
    }
    
    // Then, read available bytes from UART
    while (Serial2.available() > 0) {
        // Check if buffer has space
        if (receiveBufferIndex >= sizeof(receiveBuffer)) {
            // Buffer overflow, reset and increment error counter
            receiveBufferIndex = 0;
            packetsDropped++;
            return PACKET_ERROR;
        }
        
        // Read one byte and add to buffer
        uint8_t currentByte = Serial2.read();
        receiveBuffer[receiveBufferIndex++] = currentByte;
        
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
                
                // Track packet IDs to detect missed packets
                if (firstPacket) {
                    // Initialize with first packet
                    firstPacket = false;
                    lastPacketId = latestPacket.packetId;
                } else {                    // Calculate expected vs actual difference in packet IDs
                    uint16_t expectedDiff = 1;
                    uint16_t actualDiff = (latestPacket.packetId - lastPacketId) & 0xFFFF; // Handle wrap-around
                    
                    // Check for missed packets with a more robust approach
                    // Only count if difference is reasonable (less than 100) to avoid false positives
                    // from packet ID resets or other unexpected conditions
                    if (actualDiff > expectedDiff && actualDiff < 100) {
                        missedPackets += (actualDiff - expectedDiff);
                        
                        // Removed packet loss debug logging as it's causing more problems than it solves
                        // No need to log missed packets anymore
                    }
                    
                    lastPacketId = latestPacket.packetId;
                }
                  // No longer logging every successfully received packet to reduce log verbosity
                
                return PACKET_COMPLETE;
            } else {
                // CRC error - attempt resynchronization
                crcErrors++;
                // Shift buffer by 1 byte to try to resynchronize
                memmove(receiveBuffer, receiveBuffer + 1, sizeof(receiveBuffer) - 1);
                receiveBufferIndex = packetSize - 1;
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

uint32_t UartManager::getMissedPackets() const {
    return missedPackets;
}

float UartManager::getPacketLossRate() const {
    uint32_t totalExpected = packetsReceived + missedPackets;
    if (totalExpected == 0) return 0.0f;
    return (float)missedPackets / (float)totalExpected * 100.0f;
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
    missedPackets = 0;
    firstPacket = true;
}

bool UartManager::sendCommand(const char* cmd) {
    // Send command to NAVC over Serial2
    Serial2.println(cmd);
    return true;
}

void UartManager::storeExaminedByte(uint8_t byte) {
    // Store the examined byte only if there's space
    if (examinedBufferSize < sizeof(examinedBuffer)) {
        examinedBuffer[examinedBufferSize++] = byte;
    }
}

bool UartManager::verifyCrc16(const SensorPacket& packet) {
    // Calculate CRC-16 for packet data using FrameCodec to ensure consistency
    // with the NAVC side calculation
    uint16_t calculatedCrc = FrameCodec::calculateSensorPacketCRC(
        reinterpret_cast<const uint8_t*>(&packet),
        sizeof(SensorPacket)
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
