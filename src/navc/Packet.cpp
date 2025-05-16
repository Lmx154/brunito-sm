#include "../include/navc/Packet.h"
#include <Arduino.h>
#include <HardwareSerial.h>

PacketManager::PacketManager() : 
    queueHead(0),
    queueTail(0),
    packetsSent(0),
    packetsDropped(0)
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
    // Send all queued packets over UART
    while (!queueIsEmpty()) {
        // Get next packet from queue
        SensorPacket& packet = packetQueue[queueHead];
        
        // Send packet over Serial2 (UART to FC)
        Serial2.write(reinterpret_cast<const uint8_t*>(&packet), sizeof(SensorPacket));
        
        // Update statistics
        packetsSent++;
        
        // Move head forward
        queueHead = (queueHead + 1) % PACKET_QUEUE_SIZE;
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

void PacketManager::resetStats() {
    packetsSent = 0;
    packetsDropped = 0;
}
