#include "../include/fc/CmdParser.h"
#include "../include/utils/FrameCodec.h"

CmdParser::CmdParser(StateManager& sm) : stateManager(sm), cmdIndex(0) {
    // Initialize command buffer
    memset(cmdBuffer, 0, MAX_CMD_LENGTH);
}

void CmdParser::processChar(char c) {
    // Skip carriage return and newline characters, which might be part of line endings
    if (c == '\r' || c == '\n') {
        return;
    }
    
    // Check for end of command
    if (c == '>') {
        // Process complete command
        if (cmdIndex > 0 && cmdBuffer[0] == '<') {
            cmdBuffer[cmdIndex] = 0; // Null terminate
            parseAndExecute();
        } else {
            char buffer[64];
            FrameCodec::formatDebug(buffer, sizeof(buffer), "INVALID_CMD_FORMAT");
            Serial.println(buffer);
        }
        
        // Reset buffer
        cmdIndex = 0;
        memset(cmdBuffer, 0, MAX_CMD_LENGTH);
    } 
    // Store character if buffer not full
    else if (cmdIndex < MAX_CMD_LENGTH - 1) {
        cmdBuffer[cmdIndex++] = c;
    }
}

bool CmdParser::parseAndExecute() {
    // Check for valid command format: <CMD:payload>
    if (strncmp(cmdBuffer, "<CMD:", 5) != 0) {
        sendAck(false, "INVALID_FORMAT");
        return false;
    }
    
    // Extract command string and parameters
    char cmdStr[MAX_CMD_LENGTH] = {0};
    char params[MAX_CMD_LENGTH] = {0};
    char checksumStr[10] = {0};
    
    // Parse command format: <CMD:command:param1=val1,param2=val2:CHECKSUM>
    int numScanned = sscanf(cmdBuffer, "<CMD:%[^:]:%[^:]:%[^>]>", cmdStr, params, checksumStr);
    
    // Command must have at least the command portion
    if (numScanned < 1) {
        sendAck(false, "PARSE_ERROR");
        return false;
    }
    
    // Get command type
    CommandType cmd = getCommandType(cmdStr);
    
    // Validate command parameters if present
    if (numScanned >= 2 && strlen(params) > 0) {
        if (!validateParameters(cmdStr, params)) {
            sendAck(false, "INVALID_PARAMS");
            return false;
        }
    }
    
    // Verify checksum if present
    if (numScanned >= 3 && strlen(checksumStr) > 0) {
        uint16_t providedChecksum = strtoul(checksumStr, NULL, 16);
        if (!verifyChecksum(cmdBuffer, providedChecksum)) {
            sendAck(false, "CHECKSUM_ERROR");
            return false;
        }    }
    
    // Process command
    bool success = stateManager.processCommand(cmd, cmdBuffer);
    
    // Enhanced command acknowledgment reporting
    if (success) {
        // For state transition commands, report the resulting state
        if (cmd == CMD_DISARM) {
            sendAck(true, "IDLE");
        } else if (cmd == CMD_ARM) {
            sendAck(true, "ARMED");
        } else if (cmd == CMD_ENTER_TEST) {
            sendAck(true, "TEST");        } else if (cmd == CMD_ENTER_RECOVERY) {
            sendAck(true, "RECOVERY");
        } else if (cmd == CMD_TEST) {
            sendAck(true, "TEST_SUCCESS");
        } else if (cmd == CMD_QUERY) {
            // For query, include the current state in the acknowledgment
            sendAck(true, stateManager.getStateString());        } else if (cmd == CMD_CONTROL) {
            sendAck(true, "CONTROL_APPLIED");          } else if (cmd == CMD_NAVC_RESET_STATS) {
            sendAck(true, "NAVC_STATS_RESET");        } else if (cmd == CMD_TEST_DEVICE) {
            sendAck(true, "TEST_SUCCESS");        } else if (cmd == CMD_TEST_SERVO) {
            sendAck(true, "SERVO_TEST_SUCCESS");
        } else if (cmd == CMD_TEST_ALTITUDE) {
            sendAck(true, "ALTITUDE_TEST_SUCCESS");        } else if (cmd == CMD_ENABLE_ALTITUDE_TEST) {
            sendAck(true, "ALTITUDE_TEST_ENABLED");
        } else if (cmd == CMD_DISABLE_ALTITUDE_TEST) {
            sendAck(true, "ALTITUDE_TEST_DISABLED");
        } else if (cmd == CMD_VELOCITY_TEST) {
            sendAck(true, "VELOCITY_TEST_SUCCESS");
        } else if (cmd == CMD_ENABLE_VELOCITY_TEST) {
            sendAck(true, "VELOCITY_TEST_ENABLED");
        } else if (cmd == CMD_DISABLE_VELOCITY_TEST) {
            sendAck(true, "VELOCITY_TEST_DISABLED");
        } else {
            // For any other command, use the current state
            sendAck(true, stateManager.getStateString());
        }
    } else {
        // For failed commands, provide a more descriptive denial reason
        if (cmd == CMD_ARM && stateManager.getCurrentState() == STATE_ARMED) {
            sendAck(false, "ALREADY_ARMED");
        } else if (cmd == CMD_ENTER_TEST && stateManager.getCurrentState() != STATE_IDLE && stateManager.getCurrentState() != STATE_TEST) {
            sendAck(false, "MUST_BE_IDLE");
        } else if (cmd == CMD_ENTER_RECOVERY && stateManager.getCurrentState() != STATE_ARMED && stateManager.getCurrentState() != STATE_RECOVERY) {
            sendAck(false, "MUST_BE_ARMED");
        } else {
            sendAck(false, "DENIED");
        }
    }
    
    return success;
}

CommandType CmdParser::getCommandType(const char* cmdStr) {    if (strcmp(cmdStr, "DISARM") == 0) return CMD_DISARM;
    if (strcmp(cmdStr, "ARM") == 0) return CMD_ARM;
    if (strcmp(cmdStr, "ENTER_TEST") == 0) return CMD_ENTER_TEST;
    if (strcmp(cmdStr, "ENTER_RECOVERY") == 0) return CMD_ENTER_RECOVERY;
    if (strcmp(cmdStr, "TEST") == 0) return CMD_TEST_DEVICE; // Map "TEST" to our new command
    if (strcmp(cmdStr, "SERVO_TEST") == 0) return CMD_TEST_SERVO; // New servo test command    if (strcmp(cmdStr, "ALTITUDE_TEST") == 0) return CMD_TEST_ALTITUDE; // New altitude-based test command
    if (strcmp(cmdStr, "ENABLE_ALTITUDE_TEST") == 0) return CMD_ENABLE_ALTITUDE_TEST; // New background altitude test command
    if (strcmp(cmdStr, "DISABLE_ALTITUDE_TEST") == 0) return CMD_DISABLE_ALTITUDE_TEST; // Disable background altitude test command
    if (strcmp(cmdStr, "VELOCITY_TEST") == 0) return CMD_VELOCITY_TEST; // New velocity-based test command
    if (strcmp(cmdStr, "ENABLE_VELOCITY_TEST") == 0) return CMD_ENABLE_VELOCITY_TEST; // New background velocity test command
    if (strcmp(cmdStr, "DISABLE_VELOCITY_TEST") == 0) return CMD_DISABLE_VELOCITY_TEST; // Disable background velocity test command
    if (strcmp(cmdStr, "QUERY") == 0) return CMD_QUERY;
    if (strncmp(cmdStr, "CONTROL", 7) == 0) return CMD_CONTROL;
    if (strcmp(cmdStr, "NAVC_RESET_STATS") == 0) return CMD_NAVC_RESET_STATS;
    
    // Default
    return CMD_QUERY;
}

bool CmdParser::isCommandAllowed(const char* cmdStr) {
    return stateManager.isCommandAllowed(getCommandType(cmdStr));
}

void CmdParser::sendAck(bool success, const char* message) {
    char ackBuffer[64];
    FrameCodec::formatCmdAck(ackBuffer, sizeof(ackBuffer), success, message);
    Serial.println(ackBuffer);
    
    // Also forward ACK over LoRa (if available)
    #ifdef FC_BUILD
    extern LoraManager loraManager;
    if (loraManager.isInitialized()) {
        loraManager.sendPacket(LORA_TYPE_CMD, ackBuffer, strlen(ackBuffer));
    }
    #endif
}

// Calculate CRC16-CCITT checksum for command
uint16_t CmdParser::calculateChecksum(const char* cmd, size_t length) {
    // CRC16-CCITT calculation
    uint16_t crc = 0xFFFF;
    
    for (size_t i = 0; i < length; i++) {
        crc ^= (uint16_t)cmd[i] << 8;
        
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc = crc << 1;
            }
        }
    }
    
    return crc;
}

// Verify command checksum
bool CmdParser::verifyChecksum(const char* cmd, uint16_t providedChecksum) {
    // Calculate length of command without checksum portion
    // Format: <CMD:command:params:checksum>
    // We need to calculate checksum for "<CMD:command:params:"
    
    // Find last colon which precedes the checksum
    const char* lastColon = strrchr(cmd, ':');
    if (!lastColon) {
        return false;
    }
    
    // Calculate checksum from start to last colon (inclusive)
    size_t checksumLen = lastColon - cmd + 1;
    uint16_t calculatedChecksum = calculateChecksum(cmd, checksumLen);
    
    return calculatedChecksum == providedChecksum;
}

// Validate command parameters based on the command type
bool CmdParser::validateParameters(const char* cmdStr, const char* params) {
    // Parameter validation based on command type
    if (strcmp(cmdStr, "CONTROL") == 0) {
        // Example: CONTROL:servo=45,buzzer=1
        char key[16] = {0};
        int32_t value = 0;
        
        // Make a copy of params string since strtok modifies the string
        char paramsCopy[MAX_CMD_LENGTH];
        strncpy(paramsCopy, params, sizeof(paramsCopy) - 1);
        
        // Parse each key-value pair
        char* token = strtok(paramsCopy, ",");
        while (token != NULL) {
            if (!parseParams(token, key, sizeof(key), &value)) {
                return false;
            }
            
            // Check specific parameter ranges
            if (strcmp(key, "servo") == 0) {
                // Servo position (degrees: 0-180)
                if (value < 0 || value > 180) {
                    return false;
                }
            } else if (strcmp(key, "buzzer") == 0) {
                // Buzzer (0=off, 1=on)
                if (value < 0 || value > 1) {
                    return false;
                }
            }
            
            token = strtok(NULL, ",");
        }
        
        return true;    } else if (strcmp(cmdStr, "ALTITUDE_TEST") == 0) {
        // Example: ALTITUDE_TEST:threshold=1000
        char key[16] = {0};
        int32_t value = 0;
        
        // Make a copy of params string since strtok modifies the string
        char paramsCopy[MAX_CMD_LENGTH];
        strncpy(paramsCopy, params, sizeof(paramsCopy) - 1);
        
        // Parse threshold parameter
        char* token = strtok(paramsCopy, ",");
        if (token != NULL) {
            if (!parseParams(token, key, sizeof(key), &value)) {
                return false;
            }
            
            // Check if parameter is threshold and has a valid value (in cm)
            if (strcmp(key, "threshold") == 0) {
                // Threshold must be positive and reasonable (1cm to 30000cm / 300m)
                if (value < 1 || value > 30000) {
                    return false;
                }
            } else {
                // Unknown parameter
                return false;
            }
        }
        
        return true;
    } else if (strcmp(cmdStr, "ENABLE_ALTITUDE_TEST") == 0) {
        // Example: ENABLE_ALTITUDE_TEST:threshold=1000
        char key[16] = {0};
        int32_t value = 0;
        
        // Make a copy of params string since strtok modifies the string
        char paramsCopy[MAX_CMD_LENGTH];
        strncpy(paramsCopy, params, sizeof(paramsCopy) - 1);
        
        // Parse threshold parameter
        char* token = strtok(paramsCopy, ",");
        if (token != NULL) {
            if (!parseParams(token, key, sizeof(key), &value)) {
                return false;
            }
            
            // Check if parameter is threshold and has a valid value (in cm)
            if (strcmp(key, "threshold") == 0) {
                // Threshold must be positive - no upper limit for background altitude test
                if (value < 1) {
                    return false;
                }
            } else {
                // Unknown parameter
                return false;
            }
        }
        
        return true;    } else if (strcmp(cmdStr, "DISABLE_ALTITUDE_TEST") == 0) {
        // DISABLE_ALTITUDE_TEST should not have any parameters
        return strlen(params) == 0;
    } else if (strcmp(cmdStr, "VELOCITY_TEST") == 0) {
        // Example: VELOCITY_TEST:threshold=5
        char key[16] = {0};
        int32_t value = 0;
        
        // Make a copy of params string since strtok modifies the string
        char paramsCopy[MAX_CMD_LENGTH];
        strncpy(paramsCopy, params, sizeof(paramsCopy) - 1);
        
        // Parse threshold parameter
        char* token = strtok(paramsCopy, ",");
        if (token != NULL) {
            if (!parseParams(token, key, sizeof(key), &value)) {
                return false;
            }
            
            // Check if parameter is threshold and has a valid value (in m/s as integer)
            if (strcmp(key, "threshold") == 0) {
                // Threshold must be positive and reasonable (1 to 50 m/s)
                if (value < 1 || value > 50) {
                    return false;
                }
            } else {
                // Unknown parameter
                return false;
            }
        }
        
        return true;
    } else if (strcmp(cmdStr, "ENABLE_VELOCITY_TEST") == 0) {
        // Example: ENABLE_VELOCITY_TEST:threshold=5
        char key[16] = {0};
        int32_t value = 0;
        
        // Make a copy of params string since strtok modifies the string
        char paramsCopy[MAX_CMD_LENGTH];
        strncpy(paramsCopy, params, sizeof(paramsCopy) - 1);
        
        // Parse threshold parameter
        char* token = strtok(paramsCopy, ",");
        if (token != NULL) {
            if (!parseParams(token, key, sizeof(key), &value)) {
                return false;
            }
            
            // Check if parameter is threshold and has a valid value (in m/s as integer)
            if (strcmp(key, "threshold") == 0) {
                // Threshold must be positive and reasonable (1 to 50 m/s)
                if (value < 1 || value > 50) {
                    return false;
                }
            } else {
                // Unknown parameter
                return false;
            }
        }
        
        return true;
    } else if (strcmp(cmdStr, "DISABLE_VELOCITY_TEST") == 0) {
        // DISABLE_VELOCITY_TEST should not have any parameters
        return strlen(params) == 0;
    }
    
    // For other commands, no parameters expected or needed
    return true;
}

// Parse a parameter key-value pair (e.g., "servo=45")
bool CmdParser::parseParams(const char* paramStr, char* key, size_t keySize, int32_t* value) {
    char* equals = strchr((char*)paramStr, '=');
    if (!equals) {
        return false;
    }
    
    // Calculate key length and copy to output buffer
    size_t keyLen = equals - paramStr;
    if (keyLen >= keySize) {
        return false;
    }
    
    // Copy key portion
    strncpy(key, paramStr, keyLen);
    key[keyLen] = '\0';
    
    // Parse value portion
    *value = atol(equals + 1);
    
    return true;
}

// Process a complete command string (for testing)
bool CmdParser::processCommand(const char* cmd) {
    // Reset buffer
    cmdIndex = 0;
    memset(cmdBuffer, 0, MAX_CMD_LENGTH);
    
    // Copy command to buffer
    strncpy(cmdBuffer, cmd, MAX_CMD_LENGTH - 1);
    
    // Process the command
    return parseAndExecute();
}
