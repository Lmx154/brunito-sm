/**
 * LoraManager.h - LoRa communication handler
 * 
 * This file defines the LoraManager class which handles LoRa
 * communication for both FC and GS modules.
 */

#ifndef LORA_MANAGER_H
#define LORA_MANAGER_H

#include <Arduino.h>
#include <RadioLib.h>
#include "../config/lora.h"

// Forward declaration
class LoraManager;

// Default interrupt handler for RadioLib
extern volatile bool loraTransmittedFlag;
extern volatile bool loraReceivedFlag;

// Interrupt handler prototypes
void setLoraTransmittedFlag(void);
void setLoraReceivedFlag(void);

// Packet structure
typedef struct {
    uint8_t type;
    uint8_t source;
    uint8_t dest;
    uint16_t id;
    uint8_t data[LORA_MAX_PACKET_SIZE - 6]; // 6 bytes for header (type, source, dest, id)
    uint8_t len;
} LoraPacket;

class LoraManager {
private:
    // RadioLib module
    Module* module;
    RFM95 radio;
    
    // Module identity (FC or GS)
    uint8_t address;
    uint8_t targetAddress;
    
    // Packet tracking
    uint16_t txPacketId;
    uint16_t rxPacketId;
    
    // Queue for outgoing messages with retries
    static const int QUEUE_SIZE = 5;
    LoraPacket outQueue[QUEUE_SIZE];
    uint32_t queueRetryTime[QUEUE_SIZE];
    uint8_t queueRetries[QUEUE_SIZE];
    bool queueActive[QUEUE_SIZE];
      // Status
    bool initialized;
    int16_t rssi;
    float snr;
    
    // Statistics for link quality monitoring
    uint16_t packetsSent;
    uint16_t packetsReceived;
    uint16_t packetsLost;
    uint32_t lastStatsResetTime;
    uint32_t lastReceivedTime;   // Timestamp of last received packet from target
    uint32_t connectionTimeout;  // Time in ms after which connection is considered lost
      // Methods for packet handling
    bool encodePacket(LoraPacket* packet, uint8_t* buffer, size_t* size);
    bool decodePacket(uint8_t* buffer, size_t size, LoraPacket* packet);
    int findFreeQueueSlot();
    void processIncomingPacket(LoraPacket* packet);
    void sendAck(uint16_t packetId);
    void sendAckImmediate(uint16_t packetId); // Sends ACK immediately without queuing
    
public:
    LoraManager();
    
    // Initialization
    bool begin(uint8_t address, uint8_t targetAddress);
    bool sendSettings();
    bool setSettings(LoraSettings settings);
    
    // Transmission
    bool sendPacket(uint8_t type, const char* data, size_t len);
    bool sendCommand(const char* cmd);
    
    // Reception
    void checkReceived();
    void checkQueue();
      // Config and status
    int16_t getLastRssi() const { return rssi; }
    float getLastSnr() const { return snr; }
    bool isInitialized() const { return initialized; }
    
    // Connection status
    bool isConnected() const { return (millis() - lastReceivedTime) < connectionTimeout; }
    void setConnectionTimeout(uint32_t timeoutMs) { connectionTimeout = timeoutMs; }
    uint32_t getTimeSinceLastRx() const { return millis() - lastReceivedTime; }
    
    // Get link quality statistics
    uint16_t getPacketsSent() const { return packetsSent; }
    uint16_t getPacketsReceived() const { return packetsReceived; }
    uint16_t getPacketsLost() const { return packetsLost; }
    float getPacketLossRate() const { 
        return (packetsSent > 0) ? (float)packetsLost / (float)packetsSent : 0.0f; 
    }
    void resetStats() { packetsSent = packetsReceived = packetsLost = 0; lastStatsResetTime = millis(); }
    uint32_t getStatsDuration() const { return millis() - lastStatsResetTime; }
    
    // Callback for received data
    void (*onPacketReceived)(LoraPacket* packet) = nullptr;
};

#endif // LORA_MANAGER_H
