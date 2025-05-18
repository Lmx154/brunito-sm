#include "../include/navc/Packet.h"
#include <Arduino.h>
#include <HardwareSerial.h>

PacketManager::PacketManager() : 
    queueHead(0),
    queueTail(0),
    packetsSent(0),
    packetsDropped(0),
    lastTransmitTime(0)
{
}

bool PacketManager::enqueuePacket(const SensorPacket& packet) {
    // Check if queue is full
    if (queueIsFull()) {
        packetsDropped++;
        return false;
    }
    
    // Add packet to the queue
    packetQueue[queueTail] = packet;
    queueTail = (queueTail + 1) % PACKET_QUEUE_SIZE;
    
    return true;
}

void PacketManager::sendQueuedPackets() {
    // Apply rate limiting to avoid overwhelming the FC (max 25Hz instead of 50Hz)
    // This helps prevent buffer overruns in the FC
    unsigned long currentTime = millis();
    if (currentTime - lastTransmitTime < 40) { // 40ms = 25Hz max
        return;
    }
    
    // Limit number of packets sent per call to avoid blocking too long
    // This prevents FC freezing by ensuring we don't spend too much time
    // transmitting in one go
    int packetsSentThisCall = 0;
    const int MAX_PACKETS_PER_CALL = 2; // Maximum 2 packets per call
    
    // Send queued packets over UART with limits
    while (!queueIsEmpty() && packetsSentThisCall < MAX_PACKETS_PER_CALL) {
        // Get next packet from queue
        SensorPacket& packet = packetQueue[queueHead];
        
        // Send packet over Serial2 (UART to FC)
        size_t bytesWritten = Serial2.write(reinterpret_cast<const uint8_t*>(&packet), sizeof(SensorPacket));
        
        // Update statistics only if successful
        if (bytesWritten == sizeof(SensorPacket)) {
            packetsSent++;
            packetsSentThisCall++;
        } else {
            packetsDropped++;
        }
        
        // Move head forward
        queueHead = (queueHead + 1) % PACKET_QUEUE_SIZE;
        
        // Apply full buffer flush after each packet to ensure reliable transmission
        Serial2.flush();
    }
    
    // Only update last transmit time if we actually sent something
    if (packetsSentThisCall > 0) {
        lastTransmitTime = currentTime;
    }
}

bool PacketManager::queueIsFull() const {
    return ((queueTail + 1) % PACKET_QUEUE_SIZE) == queueHead;
}

bool PacketManager::queueIsEmpty() const {
    return queueHead == queueTail;
}

uint32_t PacketManager::getPacketsSent() const {
    return packetsSent;
}

uint32_t PacketManager::getPacketsDropped() const {
    return packetsDropped;
}

float PacketManager::getPacketLossRate() const {
    if (packetsSent + packetsDropped == 0) {
        return 0.0f;
    }
    
    return (float)packetsDropped / (float)(packetsSent + packetsDropped) * 100.0f;
}

uint8_t PacketManager::getQueueSize() const {
    if (queueHead <= queueTail) {
        return queueTail - queueHead;
    } else {
        return PACKET_QUEUE_SIZE - (queueHead - queueTail);
    }
}

void PacketManager::resetStats() {
    packetsSent = 0;
    packetsDropped = 0;
}
