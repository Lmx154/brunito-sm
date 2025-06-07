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
    
    // Verify that bandwidth is a valid value
    float bandwidth = LORA_BANDWIDTH;
    if (bandwidth <= 0.0f) {
        bandwidth = 500.0f; // Use a safe default
        Serial.println("WARNING: Invalid bandwidth detected, using 500.0 kHz");
    }
    
    // Print initial settings
    char settingsBuffer[120];
    snprintf(settingsBuffer, sizeof(settingsBuffer), 
             "LoRa Initialization: FREQ=%.1f, BW=%.1f, SF=%d, CR=%d, SW=0x%02X, TXP=%d",
             LORA_FREQUENCY, bandwidth, LORA_SPREADING_FACTOR, 
             LORA_CODING_RATE, LORA_SYNC_WORD, LORA_TX_POWER);
    Serial.println(settingsBuffer);
      // Initialize radio with validated bandwidth
    int state = radio.begin(
        LORA_FREQUENCY,
        bandwidth,  // Use our validated bandwidth
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
    }      initialized = true;
    
    // Don't send settings during initialization to avoid loops
    // The FC will need to explicitly call sendSettings() if needed,
    // and only when initialized and connected to a GS
    return true;
}

bool LoraManager::sendSettings() {
    // Only FC sends settings
    if (address != LORA_FC_ADDR) {
        return false;
    }
    
    // Use a static flag to prevent multiple calls that could cause loops
    static bool settingsSent = false;
    static uint32_t lastAttempt = 0;
    uint32_t now = millis();
    
    // Completely block repeated settings transmission - no cooldown period
    // Settings will only be sent once per power cycle
    if (settingsSent) {
        Serial.println("Settings already sent, transmission blocked to prevent infinite loop");
        return true;
    }
    
    lastAttempt = now;
    
    // Create settings packet with sanity checks
    LoraSettings settings;
    settings.spreadingFactor = LORA_SPREADING_FACTOR;
    settings.bandwidth = LORA_BANDWIDTH > 0.0f ? LORA_BANDWIDTH : 500.0f; // Validate
    settings.codingRate = LORA_CODING_RATE;
    settings.syncWord = LORA_SYNC_WORD;
    settings.txPower = LORA_TX_POWER;
      
    // Debug print settings
    char debugBuffer[120];
    snprintf(debugBuffer, sizeof(debugBuffer), 
             "LoRa TX Settings: SF=%d, BW=%.1f, CR=%d, SW=0x%02X, TXP=%d",
             settings.spreadingFactor, settings.bandwidth, settings.codingRate, 
             settings.syncWord, settings.txPower);
    Serial.println(debugBuffer);
    
    // Special handling for settings packet to avoid loops
    // Clear all existing settings packets from queue
    for (int i = 0; i < QUEUE_SIZE; i++) {
        if (queueActive[i] && outQueue[i].type == LORA_TYPE_SETTINGS) {
            Serial.println("Removing existing settings packet from queue");
            queueActive[i] = false;
        }
    }
    
    // Send settings directly with no retries
    uint8_t buffer[sizeof(settings)];
    memcpy(buffer, &settings, sizeof(settings));
    
    // Find a free queue slot
    int slot = findFreeQueueSlot();
    if (slot < 0) {
        Serial.println("Failed to send settings: queue full");
        return false;
    }
      // Create packet manually with special ID
    LoraPacket packet;
    packet.type = LORA_TYPE_SETTINGS;
    packet.source = address;
    packet.dest = targetAddress;
    packet.id = 0xFFFF; // Special ID for settings only
    
    // Copy data
    size_t copyLen = sizeof(settings);
    memcpy(packet.data, buffer, copyLen);
    packet.len = copyLen;
    
    // Add to queue with NO retries - FIRE AND FORGET
    memcpy(&outQueue[slot], &packet, sizeof(packet));
    queueActive[slot] = true;
    queueRetries[slot] = 0; // No retries for settings
    queueRetryTime[slot] = millis(); // Send immediately
      settingsSent = true;
    Serial.println("Settings packet queued with special handling (fire-and-forget)");
    
    // Process queue immediately - but only once
    checkQueue();
    
    // Add a delay to allow processing before continuing
    delay(10);
    
    return true;
}

bool LoraManager::setSettings(LoraSettings settings) {
    // Apply new LoRa settings
    int state = radio.setSpreadingFactor(settings.spreadingFactor);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.print("Failed to set SF: ");
        Serial.println(state);
        return false;
    }
    
    state = radio.setBandwidth(settings.bandwidth);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.print("Failed to set BW: ");
        Serial.println(state);
        return false;
    }
      state = radio.setCodingRate(settings.codingRate);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.print("Failed to set CR: ");
        Serial.println(state);
        return false;
    }
    
    state = radio.setSyncWord(settings.syncWord);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.print("Failed to set SW: ");
        Serial.println(state);
        return false;
    }
    
    state = radio.setOutputPower(settings.txPower);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.print("Failed to set TX power: ");
        Serial.println(state);
        return false;
    }
    
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
    
    // For critical commands like DISARM and ENTER_RECOVERY, immediately process the queue to send with highest priority
    if (result && (strstr(cmd, "DISARM") != NULL || strstr(cmd, "ENTER_RECOVERY") != NULL)) {
        // Give a slight delay to ensure proper radio state
        delay(10);
        // Process queue multiple times for critical commands to ensure delivery
        checkQueue();
        delay(5);
        checkQueue();
        delay(5);
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
    
    // Special priority: Process SETTINGS packets first as they are critical for communication setup
    for (int i = 0; i < QUEUE_SIZE; i++) {
        if (queueActive[i] && outQueue[i].type == LORA_TYPE_SETTINGS) {
            // Encode packet
            uint8_t buffer[LORA_MAX_PACKET_SIZE];
            size_t size;
              if (encodePacket(&outQueue[i], buffer, &size)) {
                // Cancel receive mode and send packet
                radio.standby();
                
                // Removed LoRa TX debug message to reduce verbosity
                
                // In RadioLib 7.1.2, transmit takes an array and length
                int state = radio.transmit(buffer, size);
                
                // Update statistics
                packetsSent++;
                
                if (state == RADIOLIB_ERR_NONE) {
                    // Track bytes sent
                    totalBytesSent += outQueue[i].len + 6; // 6 bytes for header
                    
                    // Fire-and-forget approach - mark as inactive immediately
                    queueActive[i] = false;
                    
                    // Removed LoRa TX debug message to reduce verbosity
                } else {
                    // On error, also mark as inactive to avoid retransmission
                    queueActive[i] = false;
                    // Removed LoRa TX debug message to reduce verbosity
                }
                
                // Restart receiver and return immediately
                radio.startReceive();
                return;
            }
        }
    }
    
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
                
                // Removed LoRa TX debug logging to reduce verbosity
                
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
    
    // Second priority: Process command packets first (especially ENTER_RECOVERY and DISARM)
    for (int i = 0; i < QUEUE_SIZE; i++) {
        if (queueActive[i] && now >= queueRetryTime[i] && outQueue[i].type == LORA_TYPE_CMD) {
            // Check for critical commands (DISARM or ENTER_RECOVERY)
            bool isCriticalCmd = false;
            if (outQueue[i].len >= 6) // Minimum length to contain "DISARM" or part of "ENTER_"
                if (strstr((char*)outQueue[i].data, "DISARM") || strstr((char*)outQueue[i].data, "ENTER_RECOVERY")) {
                    isCriticalCmd = true;
                    // Force immediate send by ensuring time threshold is met
                    queueRetryTime[i] = now - 1;
                }
            
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
                  // Removed LoRa TX debug logging to reduce verbosity
                
                // Handle transmission result
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
                        
                        // If this is a DISARM or ENTER_RECOVERY command, try even harder (more retries)
                        if (outQueue[i].type == LORA_TYPE_CMD && isCriticalCmd) {
                            maxRetries = LORA_MAX_RETRIES + 3; // Three extra tries for safety critical commands
                        }
                        
                        // If max retries reached, but add a grace period for ACKs to arrive
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
                                  // Set a timeout for final ACK wait
                                queueRetryTime[i] = now + ackTimeout;
                                
                                queueRetries[i]++; // Increment to indicate we're in the grace period
                            } else {
                                // For critical commands, try more times even after timeout
                                if (isCriticalCmd && queueRetries[i] < (maxRetries + 5)) {                                    // Keep trying with exponential backoff
                                    uint32_t backoffMs = random(500, 1000) * (queueRetries[i] - maxRetries + 1);
                                    queueRetryTime[i] = now + backoffMs;
                                    queueRetries[i]++;
                                    
                                    // Removed LoRa TX debug message to reduce verbosity
                                } else {
                                    // Grace period expired, give up and remove from queue
                                    // Removed LoRa TX debug message to reduce verbosity
                                    queueActive[i] = false;
                                    packetsLost++;
                                }
                            }
                        } else if (queueRetries[i] < maxRetries) {
                            // Normal retry path
                            // Implement exponential backoff
                            uint32_t backoffMs = LORA_RETRY_BASE_MS / 2; // Start with half the base time
                            backoffMs += random(0, 100);
                            
                            // Apply exponential backoff based on retry count
                            for (uint8_t j = 1; j < queueRetries[i]; j++) {
                                backoffMs = backoffMs * 3 / 2; // 1.5x increase per retry
                            }
                            
                            // Check queue congestion and adjust if needed
                            uint8_t pendingCount = getPendingPacketCount();
                            if (pendingCount > QUEUE_SIZE / 2) {
                                // Queue congestion, add more backoff to reduce contention
                                backoffMs += pendingCount * 50;
                            }
                            
                            // Set the retry time
                            queueRetryTime[i] = now + backoffMs;
                            queueRetries[i]++; // Increment retry counter
                        }
                    }
                } else {
                    // Transmission failed, retry immediately on next check
                    queueRetryTime[i] = now;
                      // Only log failures for packets other than ping/pong to reduce noise
                    if (outQueue[i].type != LORA_TYPE_PING && outQueue[i].type != LORA_TYPE_PONG) {
                        // Removed LoRa TX failure debug message to reduce verbosity
                    }
                }
                
                // Restart receiver
                
                radio.startReceive();
                
                // For command packets, especially critical ones, return immediately to check for ACKs
                if (isCriticalCmd) {
                    return;
                }
            }
        }
    }
    
    // Third priority: Process remaining non-telemetry, non-command packets
    for (int i = 0; i < QUEUE_SIZE; i++) {
        if (queueActive[i] && now >= queueRetryTime[i] && 
            outQueue[i].type != LORA_TYPE_TELEM && 
            outQueue[i].type != LORA_TYPE_CMD) {
            
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
                
                // Removed LoRa TX debug logging to reduce verbosity
                
                // Process result similar to command packets
                if (state == RADIOLIB_ERR_NONE) {
                    // For ACK, STATUS, PING, PONG - fire-and-forget
                    if (outQueue[i].type == LORA_TYPE_ACK || outQueue[i].type == LORA_TYPE_STATUS || 
                        outQueue[i].type == LORA_TYPE_PING || outQueue[i].type == LORA_TYPE_PONG) {
                        queueActive[i] = false;
                        totalBytesSent += outQueue[i].len + 6;
                    } else {
                        // Standard retry logic 
                        // ... original retry logic for non-command packets ...
                    }
                } else {
                    // Transmission failed, retry on next check
                    queueRetryTime[i] = now;
                }
                
                // Restart receiver and return
                radio.startReceive();
                return;
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
        // Removed LoRa RX debug messages to reduce log verbosity
    
    // Update last received time if packet came from our target
    if (packet->source == targetAddress) {
        lastReceivedTime = millis();
    }
      
    // Declare variable before the switch statement to fix the "crosses initialization" error
    bool foundMatch = false;
    
    // Handle based on packet type
    switch (packet->type) {
        case LORA_TYPE_CMD:
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
                if (queueActive[i] && outQueue[i].id == packet->id) {                    queueActive[i] = false;
                    // Removed LoRa ACK debug message to reduce verbosity
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
            
            // If no match was found, check for settings packet acknowledgment
            if (!foundMatch) {
                // CRITICAL FIX: Clear ANY settings packets in the queue regardless of ID
                // This helps break the infinite loop by acknowledging settings even with mismatched IDs
                bool foundSettingsPacket = false;
                for (int i = 0; i < QUEUE_SIZE; i++) {
                    if (queueActive[i] && outQueue[i].type == LORA_TYPE_SETTINGS) {                        // Found a settings packet waiting for ACK - consider it acknowledged
                        queueActive[i] = false;
                        // Removed LoRa RX debug message to reduce log verbosity
                        foundMatch = true;
                        foundSettingsPacket = true;
                    }
                }
                  if (!foundSettingsPacket && !foundMatch) {
                    // Removed LoRa RX debug message to reduce log verbosity
                }
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
            break;
          
        case LORA_TYPE_SETTINGS:
            // Apply LoRa settings from packet
            if (packet->len >= sizeof(LoraSettings)) {
                LoraSettings settings;
                memcpy(&settings, packet->data, sizeof(settings));
                
                // Debug print received settings
                char debugBuffer[120];
                snprintf(debugBuffer, sizeof(debugBuffer), 
                         "LoRa RX Settings: SF=%d, BW=%.1f, CR=%d, SW=0x%02X, TXP=%d",
                         settings.spreadingFactor, settings.bandwidth, settings.codingRate, 
                         settings.syncWord, settings.txPower);
                Serial.println(debugBuffer);
                
                // Make sure bandwidth is a valid value before applying
                if (settings.bandwidth <= 0.0f) {
                    settings.bandwidth = LORA_BANDWIDTH; // Use default if invalid
                    Serial.println("WARNING: Invalid bandwidth received, using default");
                }
                
                // Apply the settings
                setSettings(settings);
                  // Removed LoRa settings update debug message to reduce verbosity
                
                // CRITICAL FIX: Always ACK settings packets multiple ways for robustness
                
                // 1. Send immediate ACK (bypasses queue for faster delivery)
                sendAckImmediate(packet->id);
                
                // 2. Short delay to ensure radio is ready
                delay(5);
                
                // 3. Send another ACK through queue
                sendAck(packet->id);
                
                // 4. Send confirmation message
                if (address == LORA_GS_ADDR) {
                    char confirmMsg[] = "<GS:SETTINGS_APPLIED>";
                    sendPacket(LORA_TYPE_STATUS, confirmMsg, strlen(confirmMsg));
                }
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
        // Cancel receive mode and send ACK packet directly
        radio.standby();
        
        // In RadioLib 7.1.2, transmit takes an array and length
        int state = radio.transmit(buffer, size);
        
        if (state != RADIOLIB_ERR_NONE) {
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