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
    totalBytesSent(0),
    totalBytesReceived(0),
    lastReceivedTime(millis()), // Initialize to current time to avoid showing link down on startup
    connectionTimeout(30000),   // Default timeout of 30 seconds
    pingEnabled(false),         // Ping disabled by default
    lastPingTime(0),
    lastPongTime(0),
    pingInterval(20000)         // Ping every 20 seconds
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
    packet.len = copyLen;

    // Add to queue
    memcpy(&outQueue[slot], &packet, sizeof(packet));
    queueActive[slot] = true;

    // Set retry behavior based on packet type
    // No acknowledgments and no retries for:
    // - STATUS packets (status information)
    // - PING/PONG packets (link quality test)
    // - TELEM packets (telemetry data)
    if (type == LORA_TYPE_STATUS || type == LORA_TYPE_PING || 
        type == LORA_TYPE_PONG || type == LORA_TYPE_TELEM) {
        queueRetries[slot] = 0;  // No retries for these packet types
    } else {
        queueRetries[slot] = LORA_MAX_RETRIES;
    }
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
        if (state == RADIOLIB_ERR_NONE) {            // Update statistics (only for non-ACK packets)
            packetsReceived++;
            totalBytesReceived += len; // Track total bytes received
            
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
    
    // First priority: Process telemetry packets first as they are time-sensitive
    // Look for telemetry packets first to prioritize them
    for (int i = 0; i < QUEUE_SIZE; i++) {
        if (queueActive[i] && now >= queueRetryTime[i] && outQueue[i].type == LORA_TYPE_TELEM) {
            // Encode packet
            uint8_t buffer[LORA_MAX_PACKET_SIZE];
            size_t size;
            
            if (encodePacket(&outQueue[i], buffer, &size)) {
                // Cancel receive mode and send packet
                radio.standby();
                
                // In RadioLib 7.1.2, transmit takes an array and length
                int state = radio.transmit(buffer, size);
                
                // Update statistics
                packetsSent++;
                
                // Log transmission (but more concise for telemetry to reduce overhead)
                char logBuffer[64];
                snprintf(logBuffer, sizeof(logBuffer), "LoRa TX: type=%d, id=%d, retry=%d", 
                        outQueue[i].type, outQueue[i].id, queueRetries[i]);
                Serial.println(logBuffer);
                
                if (state == RADIOLIB_ERR_NONE) {
                    // Track bytes and free queue slot immediately (fire-and-forget)
                    totalBytesSent += outQueue[i].len + 6;
                    queueActive[i] = false;
                    
                    // Restart receiver and return immediately to process more telemetry
                    radio.startReceive();
                    return; // Process one telemetry packet then check for reception
                } else {
                    // On failure, just remove telemetry packet (it's outdated anyway)
                    queueActive[i] = false;
                    radio.startReceive();
                    return;
                }
            }
        }
    }
    
    // Second priority: Process other packets
    for (int i = 0; i < QUEUE_SIZE; i++) {
        if (queueActive[i] && now >= queueRetryTime[i] && outQueue[i].type != LORA_TYPE_TELEM) {
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
                
                // Log transmission attempt (but skip detailed logging for ping/pong packets)
                if (outQueue[i].type != LORA_TYPE_PING && outQueue[i].type != LORA_TYPE_PONG) {
                    char logBuffer[64];
                    snprintf(logBuffer, sizeof(logBuffer), "LoRa TX: type=%d, id=%d, retry=%d", 
                            outQueue[i].type, outQueue[i].id, queueRetries[i]);
                    Serial.println(logBuffer);
                }// Handle transmission result
                if (state == RADIOLIB_ERR_NONE) {
                    // Track bytes sent for bandwidth monitoring (add header size + data)
                    totalBytesSent += outQueue[i].len + 6; // 6 bytes for header
                      // Different handling based on packet type
                    if (outQueue[i].type == LORA_TYPE_ACK || outQueue[i].type == LORA_TYPE_STATUS || 
                        outQueue[i].type == LORA_TYPE_PING || outQueue[i].type == LORA_TYPE_PONG ||
                        outQueue[i].type == LORA_TYPE_TELEM) {
                        // No need to wait for ACKs on ACK packets, STATUS messages, telemetry data, or ping/pong
                        // Remove from queue immediately (fire-and-forget)
                        queueActive[i] = false;
                    } else {
                        // For all other packet types, handle retries
                        int maxRetries = LORA_MAX_RETRIES;
                        
                        // If this is a DISARM command, try even harder (more retries)
                        if (outQueue[i].type == LORA_TYPE_CMD && 
                            outQueue[i].len >= 11 && // Length check to avoid buffer overruns
                            memcmp(outQueue[i].data, "<CMD:DISARM", 11) == 0) {
                            maxRetries = LORA_MAX_RETRIES + 2; // Two extra tries for safety critical commands
                        }                        // If max retries reached, but add a grace period for ACKs to arrive
                        if (queueRetries[i] >= maxRetries) {
                            // If this is the exact retry when we hit max retries, set a grace period
                            if (queueRetries[i] == maxRetries) {
                                // Adaptive timeout based on link quality
                                uint32_t ackTimeout = LORA_ACK_TIMEOUT_MS;
                                
                                #if LORA_ADAPTIVE_TIMEOUT
                                // Adjust timeout based on RSSI value
                                if (rssi < -110) {
                                    // Very poor connection - use longer timeout
                                    ackTimeout = LORA_ACK_TIMEOUT_MS * 2;
                                } else if (rssi < -100) {
                                    // Poor connection - increase timeout
                                    ackTimeout = LORA_ACK_TIMEOUT_MS * 3 / 2;
                                }
                                
                                // Also adjust based on number of pending packets
                                uint8_t pendingCount = getPendingPacketCount();
                                if (pendingCount > QUEUE_SIZE / 2) {
                                    // Queue is getting full, increase timeout to reduce congestion
                                    ackTimeout += (pendingCount * 100); // Add 100ms per pending packet
                                }
                                #endif
                                
                                // Add grace period for ACK to arrive
                                queueRetryTime[i] = now + ackTimeout;
                                
                                // Only log for packets other than ping/pong
                                if (outQueue[i].type != LORA_TYPE_PING && outQueue[i].type != LORA_TYPE_PONG) {
                                    char logBuffer[64];
                                    snprintf(logBuffer, sizeof(logBuffer), 
                                           "LoRa TX: Max retries reached, waiting for ACK (timeout: %lu ms)", 
                                           ackTimeout);
                                    Serial.println(logBuffer);
                                }
                                
                                // Increment retry counter beyond max to indicate we're in grace period
                                queueRetries[i]++;
                            }
                            // If we're past the grace period, now actually drop the packet
                            else if (now >= queueRetryTime[i]) {
                                queueActive[i] = false;
                                
                                // Only log packet drops for packets other than ping/pong
                                if (outQueue[i].type != LORA_TYPE_PING && outQueue[i].type != LORA_TYPE_PONG) {
                                    Serial.println("LoRa TX: No ACK received, dropping packet");
                                    
                                    // Update packet loss statistics (don't count ping/pong packets)
                                    packetsLost++;
                                }
                            }                        } else if (queueRetries[i] < maxRetries) {
                            // Set next retry time with exponential backoff
                            // First transmit was already done, increment retry counter for next attempt
                            queueRetries[i]++;
                            
                            // Calculate adaptive backoff time based on link quality and packet type
                            uint32_t backoffMs = LORA_RETRY_BASE_MS;
                            
                            // Adjust for RSSI
                            if (rssi < -110) {
                                // Very poor connection - use longer backoff
                                backoffMs *= 2;
                            } else if (rssi < -100) {
                                // Poor connection - increase backoff
                                backoffMs = (backoffMs * 3) / 2;
                            }
                            
                            // Add jitter to avoid collisions when multiple devices retry at the same time
                            backoffMs += random(0, 100);
                            
                            // For telemetry packets, reduce retries to avoid overwhelming the connection
                            if (outQueue[i].type == LORA_TYPE_TELEM) {
                                // Use reduced backoff for telemetry to avoid queueing too many packets
                                backoffMs /= 2;                            } else if (outQueue[i].type == LORA_TYPE_CMD) {
                                // Critical commands get priority with faster retries
                                backoffMs *= 0.75;
                                
                                // For DISARM (safety critical), retry even faster
                                if (outQueue[i].len >= 11 && memcmp(outQueue[i].data, "<CMD:DISARM", 11) == 0) {
                                    backoffMs = LORA_RETRY_BASE_MS / 2;
                                }
                            }
                            
                            // Apply exponential backoff based on retry count
                            for (uint8_t j = 1; j < queueRetries[i]; j++) {
                                backoffMs = (backoffMs * 3) / 2;  // 1.5× multiplier each retry
                            }
                            
                            // Check queue congestion and adjust if needed
                            uint8_t pendingCount = getPendingPacketCount();
                            if (pendingCount > QUEUE_SIZE / 2) {
                                // Add extra backoff proportional to queue congestion
                                backoffMs += (pendingCount * 50);
                            }
                            
                            // Set the retry time
                            queueRetryTime[i] = now + backoffMs;
                        }
                    }                } else {
                    // Transmission failed, retry immediately on next check
                    queueRetryTime[i] = now;
                    
                    // Only log failures for packets other than ping/pong to reduce noise
                    if (outQueue[i].type != LORA_TYPE_PING && outQueue[i].type != LORA_TYPE_PONG) {
                        Serial.print("LoRa TX failed: ");
                        Serial.println(state);
                    }
                }
                
                // Restart receiver
                radio.startReceive();
            }
        }
    }
}

// Improved connected check that considers RSSI, ping responses, and packet timestamps
bool LoraManager::isConnected() const {
    // Always check if we've received any packets within the timeout period
    uint32_t timeSinceLastRx = millis() - lastReceivedTime;
    
    if (timeSinceLastRx < connectionTimeout) {
        // If we've received any packet recently, we're definitely connected
        return true;
    }
    
    if (pingEnabled) {
        // If ping is enabled, check if we've received a pong recently
        if (lastPongTime > 0 && (millis() - lastPongTime) < connectionTimeout) {
            return true;
        }
    }
    
    // If we have decent signal strength, we might still be connected despite no recent packets
    if (rssi > -110 && packetsReceived > 0) {
        return true;
    }
    
    // No evidence of connection
    return false;
}

// Send a ping packet to check link status
void LoraManager::sendPing() {
    // Only send ping if enabled
    if (!pingEnabled || !initialized) {
        return;
    }
    
    // Don't send pings too frequently
    if ((millis() - lastPingTime) < pingInterval) {
        return;
    }
    
    // Update ping time
    lastPingTime = millis();
    
    // Create a basic ping packet with timestamp
    char pingData[16];
    snprintf(pingData, sizeof(pingData), "%lu", millis());
    
    // Find a free queue slot (don't block important messages with pings)
    int slot = findFreeQueueSlot();
    if (slot < 0) {
        return; // Skip ping if queue is full
    }
    
    // Prepare packet manually
    LoraPacket packet;
    packet.type = LORA_TYPE_PING;
    packet.source = address;
    packet.dest = targetAddress;
    packet.id = txPacketId++;
    
    // Copy data 
    size_t copyLen = min(strlen(pingData), sizeof(packet.data));
    memcpy(packet.data, pingData, copyLen);
    packet.len = copyLen;
      // Add to queue with no retries
    memcpy(&outQueue[slot], &packet, sizeof(packet));
    queueActive[slot] = true;
    queueRetries[slot] = 0; // No retries for ping
    queueRetryTime[slot] = millis() + 500; // Small delay to avoid crowding the queue
}

void LoraManager::processIncomingPacket(LoraPacket* packet) {
    // Make sure the packet is for us
    if (packet->dest != address && packet->dest != 0xFF) {
        return;
    }
      // Log packet info (but skip ping/pong packets to reduce log noise)
    if (packet->type != LORA_TYPE_PING && packet->type != LORA_TYPE_PONG) {
        char logBuffer[64];
        snprintf(logBuffer, sizeof(logBuffer), "LoRa RX: type=%d, id=%d, source=0x%02X", 
                packet->type, packet->id, packet->source);
        Serial.println(logBuffer);
    }
    
    // Update last received time if packet came from our target
    if (packet->source == targetAddress) {
        lastReceivedTime = millis();
    }
      // Declare variable before the switch statement to fix the "crosses initialization" error
    bool foundMatch = false;
    
    // Handle based on packet type
    switch (packet->type) {case LORA_TYPE_CMD:
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
            
            // First try for exact ID match
            for (int i = 0; i < QUEUE_SIZE; i++) {
                if (queueActive[i] && outQueue[i].id == packet->id) {
                    queueActive[i] = false;
                    Serial.println("LoRa ACK received, packet removed from queue");
                    foundMatch = true;
                    break;
                }
            }
            
            // Extra check - if command was DISARM, always prioritize to ensure safety
            if (!foundMatch && packet->id > 0) {
                for (int i = 0; i < QUEUE_SIZE; i++) {
                    // Look for any active DISARM command that needs acknowledgment
                    if (queueActive[i] && outQueue[i].type == LORA_TYPE_CMD && 
                        outQueue[i].len >= 11 &&
                        memcmp(outQueue[i].data, "<CMD:DISARM", 11) == 0) {
                        
                        // Found a DISARM command waiting for ACK
                        queueActive[i] = false;
                        char logBuffer[96];
                        snprintf(logBuffer, sizeof(logBuffer), "LoRa DISARM ACK matched (expected ID:%d, got:%d)", 
                                outQueue[i].id, packet->id);
                        Serial.println(logBuffer);
                        foundMatch = true;
                        break;
                    }
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
            
            // Explicitly no ACK for telemetry - fire-and-forget
            // This comment clarifies that we deliberately don't send ACKs for telemetry
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
            
        case LORA_TYPE_PING:
            // Respond with a pong packet immediately, no ACK needed for pings
            {
                // Create pong packet with same payload (echo)
                char pongData[LORA_MAX_PACKET_SIZE];
                if (packet->len < sizeof(pongData)) {
                    memcpy(pongData, packet->data, packet->len);
                    pongData[packet->len] = '\0';
                    sendPacket(LORA_TYPE_PONG, pongData, packet->len);
                }
            }
            break;
            
        case LORA_TYPE_PONG:
            // Record that we received a pong response
            lastPongTime = millis();
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

// Add a method to get statistics about telemetry bandwidth usage
void LoraManager::reportBandwidthUsage() {
    if (!isInitialized()) {
        return;
    }
    
    static unsigned long lastByteCount = 0;
    static unsigned long lastReportTime = 0;
    unsigned long currentTime = millis();
    
    // Allow immediate reporting if parameter is true or use time interval
    unsigned long totalBytes = getTotalBytesSent();
    unsigned long bytesSinceLast = totalBytes - lastByteCount;
    float interval = (currentTime - lastReportTime) / 1000.0f;
    
    // Avoid division by zero
    if (interval < 0.001f) interval = 0.001f;
    
    float bytesPerSecond = bytesSinceLast / interval;    // Properly format the loss rate percentage
    char lossRateStr[16];
    snprintf(lossRateStr, sizeof(lossRateStr), "%.1f", getPacketLossRate() * 100.0f);
    
    // Calculate average packet size
    float avgPacketSize = (packetsReceived > 0) ? 
        ((float)totalBytesReceived / (float)packetsReceived) : 0.0f;
    
    // Calculate actual average throughput (including retries, overhead, etc.)
    float actualThroughput = bytesPerSecond;
    
    // Calculate pending packets in queue as a percentage of queue capacity
    uint8_t pendingCount = getPendingPacketCount();
    float queueUsage = (float)pendingCount / (float)QUEUE_SIZE * 100.0f;
    
    // Create the report
    char buffer[192]; // Increased buffer size for more data
    snprintf(buffer, sizeof(buffer), 
            "<DEBUG:BANDWIDTH:%.1f_B/s,AVG_PKT=%.1f_B,TOTAL_TX=%lu_B,TOTAL_RX=%lu_B,SENT=%u,RECV=%u,LOST=%u,LOSS=%s%%,QUEUE=%u/%.0f%%,RSSI=%d,SNR=%.1f>",
            actualThroughput,
            avgPacketSize,
            totalBytesSent,
            totalBytesReceived,
            packetsSent, 
            packetsReceived,
            packetsLost,
            lossRateStr,
            pendingCount,
            queueUsage,
            rssi, snr);
    
    Serial.println(buffer);
    
    lastByteCount = totalBytes;
    lastReportTime = currentTime;
}

// Helper method to get total bytes sent
unsigned long LoraManager::getTotalBytesSent() const {
    return totalBytesSent;
}