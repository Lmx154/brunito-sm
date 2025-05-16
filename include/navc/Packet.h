#ifndef PACKET_H
#define PACKET_H

#include <Arduino.h>
#include "../include/navc/Sensors.h"

// UART packet handling
class PacketManager {
private:
    // Ring buffer for outgoing packets
    static const int PACKET_QUEUE_SIZE = 10;
    SensorPacket packetQueue[PACKET_QUEUE_SIZE];
    volatile uint8_t queueHead;
    volatile uint8_t queueTail;
    
    // Statistics
    uint32_t packetsSent;
    uint32_t packetsDropped;
    
    // Helper methods
    bool queueIsFull() const;
    bool queueIsEmpty() const;
    
public:
    PacketManager();
    
    // Add a packet to the outgoing queue
    bool enqueuePacket(const SensorPacket& packet);
    
    // Send all queued packets
    void sendQueuedPackets();
    
    // Get statistics
    uint32_t getPacketsSent() const;
    uint32_t getPacketsDropped() const;
    float getPacketLossRate() const;
    
    // Reset statistics
    void resetStats();
};

#endif // PACKET_H
