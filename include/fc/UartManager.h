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
    
    // Buffer for examined data that needs to be processed
    uint8_t examinedBuffer[16];
    size_t examinedBufferIndex;
    size_t examinedBufferSize;
    
    // Size of expected packet
    const size_t packetSize;
    
    // Latest received packet
    SensorPacket latestPacket;
    bool packetReady;
      // Statistics
    uint32_t packetsReceived;
    uint32_t packetsDropped;
    uint32_t crcErrors;
    uint32_t missedPackets;  // Tracks packets missed due to non-sequential IDs
    uint16_t lastPacketId;   // Last successfully received packet ID
    bool firstPacket;        // Flag for first packet initialization
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
    void markPacketProcessed();    // Get statistics
    uint32_t getPacketsReceived() const;
    uint32_t getPacketsDropped() const;
    uint32_t getCrcErrors() const;
    uint32_t getMissedPackets() const;
    float getPacketLossRate() const;
    float getPacketRate() const;
    unsigned long getTimeSinceLastPacket() const;
    
    // Reset statistics
    void resetStats();
    
    // Send commands to NAVC
    bool sendCommand(const char* cmd);
    
    // Store a byte examined from Serial for later processing
    void storeExaminedByte(uint8_t byte);
};

#endif // UART_MANAGER_H
