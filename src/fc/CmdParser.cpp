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
        }
    }
    
    // Process command
    bool success = stateManager.processCommand(cmd);
    
    // Send acknowledgment
    if (success) {
        sendAck(true, stateManager.getStateString());
    } else {
        sendAck(false, "DENIED");
    }
    
    return success;
}

CommandType CmdParser::getCommandType(const char* cmdStr) {
    if (strcmp(cmdStr, "DISARM") == 0) return CMD_DISARM;
    if (strcmp(cmdStr, "ARM") == 0) return CMD_ARM;
    if (strcmp(cmdStr, "ENTER_TEST") == 0) return CMD_ENTER_TEST;
    if (strcmp(cmdStr, "ENTER_RECOVERY") == 0) return CMD_ENTER_RECOVERY;
    if (strcmp(cmdStr, "TEST") == 0) return CMD_TEST;
    if (strcmp(cmdStr, "QUERY") == 0) return CMD_QUERY;
    if (strcmp(cmdStr, "FIND_ME") == 0) return CMD_FIND_ME;
    if (strncmp(cmdStr, "CONTROL", 7) == 0) return CMD_CONTROL;
    
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
        
        return true;
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
