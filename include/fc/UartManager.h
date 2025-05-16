#ifndef UART_MANAGER_H
#define UART_MANAGER_H

#include <Arduino.h>
#include "../include/navc/Sensors.h" // To share the SensorPacket structure

// Status codes for packet reception
enum UartPacketStatus {
    PACKET_NONE,
    PACKET_COMPLETE,
    PACKET_ERROR
};

class UartManager {
private:
    // Buffer for received data
    uint8_t receiveBuffer[128];
    size_t receiveBufferIndex;
    
    // Size of expected packet
    const size_t packetSize;
    
    // Latest received packet
    SensorPacket latestPacket;
    bool packetReady;
    
    // Statistics
    uint32_t packetsReceived;
    uint32_t packetsDropped;
    uint32_t crcErrors;
    unsigned long lastPacketTime;
    
    // Verify packet CRC
    bool verifyCrc16(const SensorPacket& packet);
    uint16_t calculateCrc16(const uint8_t* data, size_t length);
    
public:
    UartManager();
    
    // Process received UART data
    UartPacketStatus processUartData();
    
    // Check if a complete packet is ready
    bool isPacketReady() const;
    
    // Get the latest complete packet
    const SensorPacket& getLatestPacket() const;
    
    // Mark the packet as processed
    void markPacketProcessed();
    
    // Get statistics
    uint32_t getPacketsReceived() const;
    uint32_t getPacketsDropped() const;
    uint32_t getCrcErrors() const;
    float getPacketRate() const;
    unsigned long getTimeSinceLastPacket() const;
    
    // Reset statistics
    void resetStats();
    
    // Send commands to NAVC
    bool sendCommand(const char* cmd);
};

#endif // UART_MANAGER_H
