/**
 * LoraManager.cpp - LoRa communication implementation
 * 
 * This file implements the LoraManager class which handles LoRa
 * communication for both FC and GS modules.
 */

#include "../include/utils/LoraManager.h"
#include "../include/utils/FrameCodec.h"

// Global flags for RadioLib interrupt handling
volatile bool loraTransmittedFlag = false;
volatile bool loraReceivedFlag = false;

// Interrupt handlers
#if defined(ESP8266) || defined(ESP32)
void ICACHE_RAM_ATTR setLoraTransmittedFlag(void) {
#else
void setLoraTransmittedFlag(void) {
#endif
    loraTransmittedFlag = true;
}

#if defined(ESP8266) || defined(ESP32)
void ICACHE_RAM_ATTR setLoraReceivedFlag(void) {
#else
void setLoraReceivedFlag(void) {
#endif
    loraReceivedFlag = true;
}

LoraManager::LoraManager() : 
    module(new Module(LORA_NSS, LORA_DIO0, LORA_NRST, LORA_DIO1)),
    radio(module),
    txPacketId(0),
    rxPacketId(0),
    initialized(false),
    rssi(0),
    snr(0),    packetsSent(0),
    packetsReceived(0),
    packetsLost(0),
    lastStatsResetTime(0),
    lastReceivedTime(0),
    connectionTimeout(15000) // Default timeout of 15 seconds (1.5x the STATUS_PERIOD_MS)
{
    // Initialize queue
    for (int i = 0; i < QUEUE_SIZE; i++) {
        queueActive[i] = false;
        queueRetryTime[i] = 0;
        queueRetries[i] = 0;
    }
}

bool LoraManager::begin(uint8_t addr, uint8_t targetAddr) {
    // Store addresses
    address = addr;
    targetAddress = targetAddr;
    
    // Initialize radio
    int state = radio.begin(
        LORA_FREQUENCY,
        LORA_BANDWIDTH,
        LORA_SPREADING_FACTOR,
        LORA_CODING_RATE,
        LORA_SYNC_WORD,
        LORA_TX_POWER,
        LORA_PREAMBLE_LEN
    );
    
    if (state != RADIOLIB_ERR_NONE) {
        char buffer[64];
        snprintf(buffer, sizeof(buffer), "Failed to init LoRa: %d", state);
        Serial.println(buffer);
        return false;
    }
      // Set interrupt handlers for RadioLib 7.1.2
    radio.setDio0Action(setLoraReceivedFlag, RISING);
    
    // Start listening
    state = radio.startReceive();
    if (state != RADIOLIB_ERR_NONE) {
        char buffer[64];
        snprintf(buffer, sizeof(buffer), "Failed to start receive: %d", state);
        Serial.println(buffer);
        return false;
    }
    
    initialized = true;
    
    // Send settings packet for handshake (if we're FC)
    if (address == LORA_FC_ADDR) {
        return sendSettings();
    }
    
    return true;
}

bool LoraManager::sendSettings() {
    // Only FC sends settings
    if (address != LORA_FC_ADDR) {
        return false;
    }
    
    // Create settings packet
    LoraSettings settings;
    settings.spreadingFactor = LORA_SPREADING_FACTOR;
    settings.bandwidth = LORA_BANDWIDTH;
    settings.codingRate = LORA_CODING_RATE;
    settings.syncWord = LORA_SYNC_WORD;
    settings.txPower = LORA_TX_POWER;
    
    // Send settings
    uint8_t buffer[sizeof(settings)];
    memcpy(buffer, &settings, sizeof(settings));
    
    return sendPacket(LORA_TYPE_SETTINGS, (const char*)buffer, sizeof(settings));
}

bool LoraManager::setSettings(LoraSettings settings) {
    // Apply new LoRa settings
    int state = radio.setSpreadingFactor(settings.spreadingFactor);
    if (state != RADIOLIB_ERR_NONE) return false;
    
    state = radio.setBandwidth(settings.bandwidth);
    if (state != RADIOLIB_ERR_NONE) return false;
    
    state = radio.setCodingRate(settings.codingRate);
    if (state != RADIOLIB_ERR_NONE) return false;
    
    state = radio.setSyncWord(settings.syncWord);
    if (state != RADIOLIB_ERR_NONE) return false;
    
    state = radio.setOutputPower(settings.txPower);
    if (state != RADIOLIB_ERR_NONE) return false;
    
    // Restart receiver with new settings
    state = radio.startReceive();
    if (state != RADIOLIB_ERR_NONE) return false;
    
    return true;
}

bool LoraManager::sendPacket(uint8_t type, const char* data, size_t len) {
    // Find a free queue slot
    int slot = findFreeQueueSlot();
    if (slot < 0) {
        return false; // Queue full
    }
    
    // Prepare packet
    LoraPacket packet;
    packet.type = type;
    packet.source = address;
    packet.dest = targetAddress;
    packet.id = txPacketId++;
    
    // Copy data, ensuring we don't overflow
    size_t copyLen = min(len, sizeof(packet.data));
    memcpy(packet.data, data, copyLen);
    packet.len = copyLen;    // Add to queue
    memcpy(&outQueue[slot], &packet, sizeof(packet));
    queueActive[slot] = true;
    // Initialize retry counter to 0
    queueRetries[slot] = 0;
    queueRetryTime[slot] = millis(); // Send immediately
    
    return true;
}

bool LoraManager::sendCommand(const char* cmd) {
    // Send command via LoRa
    bool result = sendPacket(LORA_TYPE_CMD, cmd, strlen(cmd));
    
    // For critical commands like DISARM, immediately process the queue to send them with highest priority
    if (result && strstr(cmd, "DISARM") != NULL) {
        // Give a slight delay to ensure proper radio state
        delay(10);
        checkQueue();
    }
    
    return result;
}

void LoraManager::checkReceived() {
    if (!initialized) return;
    
    // Check if a packet was received
    if (loraReceivedFlag) {
        loraReceivedFlag = false;
        
        // Read packet data
        uint8_t buffer[LORA_MAX_PACKET_SIZE];
        size_t len = 0;
        
        // For RadioLib 7.1.2 compatibility
        int state = radio.readData(buffer, LORA_MAX_PACKET_SIZE);
        
        // Get the length of the packet
        len = radio.getPacketLength();
        
        // Get packet info (in RadioLib 7.1.2)
        rssi = radio.getRSSI();
        snr = radio.getSNR();
          // Process packet if received successfully
        if (state == RADIOLIB_ERR_NONE) {
            // Update statistics (only for non-ACK packets)
            packetsReceived++;
            
            LoraPacket packet;
            if (decodePacket(buffer, len, &packet)) {
                processIncomingPacket(&packet);
            }
        }
        
        // Restart receiver
        radio.startReceive();
    }
}

void LoraManager::checkQueue() {
    if (!initialized) return;
    
    uint32_t now = millis();
    
    // Check for packets in queue that need to be sent
    for (int i = 0; i < QUEUE_SIZE; i++) {
        if (queueActive[i] && now >= queueRetryTime[i]) {
            // Encode packet
            uint8_t buffer[LORA_MAX_PACKET_SIZE];
            size_t size;
            
            if (encodePacket(&outQueue[i], buffer, &size)) {
                // Cancel receive mode and send packet
                radio.standby();
                
                // In RadioLib 7.1.2, transmit takes an array and length
                int state = radio.transmit(buffer, size);
                
                // Update statistics
                if (outQueue[i].type != LORA_TYPE_ACK) { // Don't count ACKs in the statistics
                    packetsSent++;
                }
                
                // Log transmission attempt
                char logBuffer[64];
                snprintf(logBuffer, sizeof(logBuffer), "LoRa TX: type=%d, id=%d, retry=%d", 
                        outQueue[i].type, outQueue[i].id, queueRetries[i]);
                Serial.println(logBuffer);
                  // Handle transmission result
                if (state == RADIOLIB_ERR_NONE) {
                    // Different handling based on packet type
                    if (outQueue[i].type == LORA_TYPE_ACK || outQueue[i].type == LORA_TYPE_STATUS) {
                        // No need to wait for ACKs on ACK packets or STATUS messages, remove from queue
                        queueActive[i] = false;
                    } else {
                        // For all other packet types, handle retries
                        int maxRetries = LORA_MAX_RETRIES;
                        
                        // If this is a DISARM command, try even harder (more retries)
                        if (outQueue[i].type == LORA_TYPE_CMD && 
                            outQueue[i].len >= 11 && // Length check to avoid buffer overruns
                            memcmp(outQueue[i].data, "<CMD:DISARM", 11) == 0) {
                            maxRetries = LORA_MAX_RETRIES + 2; // Two extra tries for safety critical commands
                        }
                        
                        // If max retries reached, remove from queue
                        if (queueRetries[i] >= maxRetries) {
                            queueActive[i] = false;
                            Serial.println("LoRa TX: Max retries reached, dropping packet");
                            
                            // Update packet loss statistics
                            packetsLost++;
                        } else {
                            // Set next retry time with exponential backoff
                            // First transmit was already done, increment retry counter for next attempt
                            queueRetries[i]++;
                            queueRetryTime[i] = now + LORA_RETRY_BASE_MS * (1 << queueRetries[i]);
                        }
                    }
                } else {
                    // Transmission failed, retry immediately on next check
                    queueRetryTime[i] = now;
                    Serial.print("LoRa TX failed: ");
                    Serial.println(state);
                }
                
                // Restart receiver
                radio.startReceive();
            }
        }
    }
}

void LoraManager::processIncomingPacket(LoraPacket* packet) {
    // Make sure the packet is for us
    if (packet->dest != address && packet->dest != 0xFF) {
        return;
    }
    
    // Log packet info
    char logBuffer[64];
    snprintf(logBuffer, sizeof(logBuffer), "LoRa RX: type=%d, id=%d, source=0x%02X", 
            packet->type, packet->id, packet->source);
    Serial.println(logBuffer);
    
    // Update last received time if packet came from our target
    if (packet->source == targetAddress) {
        lastReceivedTime = millis();
    }
      // Declare variable before the switch statement to fix the "crosses initialization" error
    bool foundMatch = false;
    
    // Handle based on packet type
    switch (packet->type) {        case LORA_TYPE_CMD:
            // Send ACK for command FIRST before processing to ensure prompt acknowledgment
            sendAckImmediate(packet->id);
            
            // Then forward command to callback for processing
            if (onPacketReceived) {
                onPacketReceived(packet);
            }
            break;
            
        case LORA_TYPE_ACK:
            // Find matching packet in queue and remove it
            foundMatch = false; // Reset the variable
            for (int i = 0; i < QUEUE_SIZE; i++) {
                if (queueActive[i] && outQueue[i].id == packet->id) {
                    queueActive[i] = false;
                    Serial.println("LoRa ACK received, packet removed from queue");
                    foundMatch = true;
                    break;
                }
            }
            
            // If no match was found, this could be a duplicate or delayed ACK
            if (!foundMatch) {
                Serial.println("LoRa RX: ACK for unknown packet ID");
            }
            break;
          case LORA_TYPE_TELEM:
            // Forward telemetry to callback
            if (onPacketReceived) {
                onPacketReceived(packet);
            }
            
            // No ACK needed for telemetry
            break;
              case LORA_TYPE_STATUS:
            // Forward to callback, but never ACK
            if (onPacketReceived) {
                onPacketReceived(packet);
            }
            break;   // continue with function
        
        case LORA_TYPE_SETTINGS:
            // Apply LoRa settings from packet
            if (packet->len >= sizeof(LoraSettings)) {
                LoraSettings settings;
                memcpy(&settings, packet->data, sizeof(settings));
                setSettings(settings);
                
                Serial.println("LoRa settings updated from remote");
                
                // Send ACK for settings packet
                sendAck(packet->id);
            }
            break;
    }
}

void LoraManager::sendAck(uint16_t packetId) {
    // Create ACK packet (no need to queue, sent immediately)
    LoraPacket ackPacket;
    ackPacket.type = LORA_TYPE_ACK;
    ackPacket.source = address;
    ackPacket.dest = targetAddress;
    ackPacket.id = packetId; // Use same ID as the packet we're acknowledging
    ackPacket.len = 0;
    
    // Find a free queue slot
    int slot = findFreeQueueSlot();
    if (slot >= 0) {
        // Add to queue with high priority
        memcpy(&outQueue[slot], &ackPacket, sizeof(ackPacket));
        queueActive[slot] = true;
        queueRetries[slot] = 0;
        queueRetryTime[slot] = millis(); // Send immediately
        
        // Mark ACK as high priority - will be sent first
        // This improves reliability by ensuring ACKs are processed quickly
        for (int i = 0; i < QUEUE_SIZE; i++) {
            if (queueActive[i] && outQueue[i].type != LORA_TYPE_ACK) {
                // Push back other messages in the queue
                queueRetryTime[i] += 100; // Add 100ms delay to other packets
            }
        }
        
        // For improved reliability, immediately try to send the ACK
        // This ensures faster acknowledgment without waiting for the next checkQueue call
        checkQueue();
    }
}

void LoraManager::sendAckImmediate(uint16_t packetId) {
    // Create ACK packet
    LoraPacket ackPacket;
    ackPacket.type = LORA_TYPE_ACK;
    ackPacket.source = address;
    ackPacket.dest = targetAddress;
    ackPacket.id = packetId; // Use same ID as the packet we're acknowledging
    ackPacket.len = 0;
    
    // Encode the packet
    uint8_t buffer[LORA_MAX_PACKET_SIZE];
    size_t size;
    
    if (encodePacket(&ackPacket, buffer, &size)) {
        // Log the immediate ACK
        Serial.println("LoRa TX: Sending immediate ACK");
        
        // Cancel receive mode and send ACK packet directly
        radio.standby();
        
        // In RadioLib 7.1.2, transmit takes an array and length
        int state = radio.transmit(buffer, size);
        
        if (state == RADIOLIB_ERR_NONE) {
            Serial.println("LoRa TX: Immediate ACK sent successfully");
        } else {
            Serial.print("LoRa TX: Failed to send immediate ACK, error code ");
            Serial.println(state);
            
            // Add to regular queue as a fallback
            sendAck(packetId);
        }
        
        // Restart receiver
        radio.startReceive();
    } else {
        // If encoding failed, fall back to regular queued ACK
        sendAck(packetId);
    }
}

bool LoraManager::encodePacket(LoraPacket* packet, uint8_t* buffer, size_t* size) {
    if (!packet || !buffer || !size) {
        return false;
    }
    
    // Header
    buffer[0] = packet->type;
    buffer[1] = packet->source;
    buffer[2] = packet->dest;
    buffer[3] = packet->id & 0xFF;
    buffer[4] = (packet->id >> 8) & 0xFF;
    
    // Data payload
    for (uint8_t i = 0; i < packet->len; i++) {
        buffer[5 + i] = packet->data[i];
    }
    
    // Set output size
    *size = 5 + packet->len;
    
    return true;
}

bool LoraManager::decodePacket(uint8_t* buffer, size_t size, LoraPacket* packet) {
    if (!buffer || !packet || size < 5) {
        return false;
    }
    
    // Header
    packet->type = buffer[0];
    packet->source = buffer[1];
    packet->dest = buffer[2];
    packet->id = buffer[3] | (buffer[4] << 8);
    
    // Data payload
    size_t dataLen = size - 5;
    if (dataLen > sizeof(packet->data)) {
        dataLen = sizeof(packet->data);
    }
    
    for (size_t i = 0; i < dataLen; i++) {
        packet->data[i] = buffer[5 + i];
    }
    
    packet->len = dataLen;
    
    return true;
}

int LoraManager::findFreeQueueSlot() {
    for (int i = 0; i < QUEUE_SIZE; i++) {
        if (!queueActive[i]) {
            return i;
        }
    }
    return -1;
}