#include "../include/navc/Packet.h"
#include "../include/utils/FrameCodec.h"
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
    // Apply even more conservative rate limiting to ensure stable flow
    // This prevents buffer overruns and CRC errors in the FC
    unsigned long currentTime = millis();
    if (currentTime - lastTransmitTime < 50) { // 50ms = 20Hz max
        return;
    }
    
    // Check if there's enough data in the queue to make sending worthwhile
    // This helps prevent sending partial or corrupted packets
    if (queueIsEmpty()) {
        return;
    }
    
    // Only send one packet per call to reduce UART transfer issues
    // This avoids potential timing issues that could lead to packet corruption
    const int MAX_PACKETS_PER_CALL = 1; // Maximum 1 packet per call
    int packetsSentThisCall = 0;
    
    // Send queued packets over UART with strict limits
    while (!queueIsEmpty() && packetsSentThisCall < MAX_PACKETS_PER_CALL) {
        // Get next packet from queue
        SensorPacket& packet = packetQueue[queueHead];
        
        // Recompute CRC right before sending to ensure integrity
        packet.crc16 = FrameCodec::calculateSensorPacketCRC(
            reinterpret_cast<const uint8_t*>(&packet), 
            sizeof(SensorPacket)
        );
        
        // Send packet over Serial2 (UART to FC) with complete flush
        Serial2.write(reinterpret_cast<const uint8_t*>(&packet), sizeof(SensorPacket));
        Serial2.flush(); // Ensure reliable transmission by waiting for completion
        
        // Update statistics and timing
        packetsSent++;
        packetsSentThisCall++;
        lastTransmitTime = currentTime;
        
        // Move head forward
        queueHead = (queueHead + 1) % PACKET_QUEUE_SIZE;
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
