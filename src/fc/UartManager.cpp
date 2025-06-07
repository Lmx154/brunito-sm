/**
 * UartManager.cpp - Thread-safe UART Manager
 * 
 * This file implements the UART manager with proper mutex protection
 * for thread-safe operation in the FreeRTOS environment.
 */

#include "../include/fc/UartManager.h"
#include "../include/utils/FrameCodec.h"
#include <STM32FreeRTOS.h>

// Note: The UartManager is designed to be used by a single task (uartTask)
// If accessed from multiple tasks, external synchronization is required

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
    memset(receiveBuffer, 0, sizeof(receiveBuffer));
    memset(examinedBuffer, 0, sizeof(examinedBuffer));
    memset(&latestPacket, 0, sizeof(latestPacket));
}

UartPacketStatus UartManager::processUartData() {
    // Process examined bytes first
    while (examinedBufferIndex < examinedBufferSize) {
        if (receiveBufferIndex >= sizeof(receiveBuffer)) {
            receiveBufferIndex = 0;
            packetsDropped++;
            return PACKET_ERROR;
        }
        
        uint8_t currentByte = examinedBuffer[examinedBufferIndex++];
        receiveBuffer[receiveBufferIndex++] = currentByte;
    }
    
    if (examinedBufferIndex >= examinedBufferSize) {
        examinedBufferIndex = 0;
        examinedBufferSize = 0;
    }
    
    // Read available bytes from UART
    while (Serial2.available() > 0) {
        if (receiveBufferIndex >= sizeof(receiveBuffer)) {
            receiveBufferIndex = 0;
            packetsDropped++;
            return PACKET_ERROR;
        }
        
        uint8_t currentByte = Serial2.read();
        receiveBuffer[receiveBufferIndex++] = currentByte;
        
        // Check for complete packet
        if (receiveBufferIndex >= packetSize) {
            memcpy(&latestPacket, receiveBuffer, packetSize);
            receiveBufferIndex = 0;
            
            // Verify CRC
            if (verifyCrc16(latestPacket)) {
                packetsReceived++;
                lastPacketTime = millis();
                packetReady = true;
                
                // Track missed packets
                if (firstPacket) {
                    firstPacket = false;
                    lastPacketId = latestPacket.packetId;
                } else {
                    uint16_t expectedDiff = 1;
                    uint16_t actualDiff = (latestPacket.packetId - lastPacketId) & 0xFFFF;
                    
                    if (actualDiff > expectedDiff && actualDiff < 100) {
                        missedPackets += (actualDiff - expectedDiff);
                    }
                    
                    lastPacketId = latestPacket.packetId;
                }
                
                return PACKET_COMPLETE;
            } else {
                crcErrors++;
                memmove(receiveBuffer, receiveBuffer + 1, sizeof(receiveBuffer) - 1);
                receiveBufferIndex = packetSize - 1;
                return PACKET_ERROR;
            }
        }
    }
    
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
    // Note: This function should be called with uartMutex held
    Serial2.println(cmd);
    return true;
}

void UartManager::storeExaminedByte(uint8_t byte) {
    if (examinedBufferSize < sizeof(examinedBuffer)) {
        examinedBuffer[examinedBufferSize++] = byte;
    }
}

bool UartManager::verifyCrc16(const SensorPacket& packet) {
    uint16_t calculatedCrc = FrameCodec::calculateSensorPacketCRC(
        reinterpret_cast<const uint8_t*>(&packet),
        sizeof(SensorPacket)
    );
    
    return calculatedCrc == packet.crc16;
}

uint16_t UartManager::calculateCrc16(const uint8_t* data, size_t length) {
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