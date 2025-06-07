/**
 * CmdParser.cpp - Command Parser with FreeRTOS thread safety
 * 
 * This file implements the command parser with proper mutex protection
 * for thread-safe operation in the FreeRTOS environment.
 */

#include "../include/fc/CmdParser.h"
#include "../include/utils/FrameCodec.h"
#include <STM32FreeRTOS.h>

// External mutex for thread safety
extern SemaphoreHandle_t stateMutex;
extern SemaphoreHandle_t loraMutex;

CmdParser::CmdParser(StateManager& sm) : stateManager(sm), cmdIndex(0) {
    memset(cmdBuffer, 0, MAX_CMD_LENGTH);
}

void CmdParser::processChar(char c) {
    if (c == '\r' || c == '\n') {
        return;
    }
    
    if (c == '>') {
        if (cmdIndex > 0 && cmdBuffer[0] == '<') {
            cmdBuffer[cmdIndex] = 0;
            parseAndExecute();
        } else {
            char buffer[64];
            FrameCodec::formatDebug(buffer, sizeof(buffer), "INVALID_CMD_FORMAT");
            Serial.println(buffer);
        }
        
        cmdIndex = 0;
        memset(cmdBuffer, 0, MAX_CMD_LENGTH);
    } 
    else if (cmdIndex < MAX_CMD_LENGTH - 1) {
        cmdBuffer[cmdIndex++] = c;
    }
}

bool CmdParser::parseAndExecute() {
    if (strncmp(cmdBuffer, "<CMD:", 5) != 0) {
        sendAck(false, "INVALID_FORMAT");
        return false;
    }
    
    char cmdStr[MAX_CMD_LENGTH] = {0};
    char params[MAX_CMD_LENGTH] = {0};
    char checksumStr[10] = {0};
    
    int numScanned = sscanf(cmdBuffer, "<CMD:%[^:]:%[^:]:%[^>]>", cmdStr, params, checksumStr);
    
    if (numScanned < 1) {
        sendAck(false, "PARSE_ERROR");
        return false;
    }
    
    CommandType cmd = getCommandType(cmdStr);
    
    if (numScanned >= 2 && strlen(params) > 0) {
        if (!validateParameters(cmdStr, params)) {
            sendAck(false, "INVALID_PARAMS");
            return false;
        }
    }
    
    if (numScanned >= 3 && strlen(checksumStr) > 0) {
        uint16_t providedChecksum = strtoul(checksumStr, NULL, 16);
        if (!verifyChecksum(cmdBuffer, providedChecksum)) {
            sendAck(false, "CHECKSUM_ERROR");
            return false;
        }
    }
    
    // Thread-safe command processing
    bool success = false;
    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        success = stateManager.processCommand(cmd, cmdBuffer);
        xSemaphoreGive(stateMutex);
    } else {
        sendAck(false, "MUTEX_TIMEOUT");
        return false;
    }
    
    // Send appropriate acknowledgment
    if (success) {
        switch (cmd) {
            case CMD_DISARM:
                sendAck(true, "IDLE");
                break;
            case CMD_ARM:
                sendAck(true, "ARMED");
                break;
            case CMD_ENTER_TEST:
                sendAck(true, "TEST");
                break;
            case CMD_ENTER_RECOVERY:
                sendAck(true, "RECOVERY");
                break;
            case CMD_TEST:
                sendAck(true, "TEST_SUCCESS");
                break;
            case CMD_QUERY:
                {
                    const char* state = "UNKNOWN";
                    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                        state = stateManager.getStateString();
                        xSemaphoreGive(stateMutex);
                    }
                    sendAck(true, state);
                }
                break;
            case CMD_CONTROL:
                sendAck(true, "CONTROL_APPLIED");
                break;
            case CMD_NAVC_RESET_STATS:
                sendAck(true, "NAVC_STATS_RESET");
                break;
            case CMD_TEST_DEVICE:
                sendAck(true, "TEST_SUCCESS");
                break;
            case CMD_TEST_SERVO:
                sendAck(true, "SERVO_TEST_SUCCESS");
                break;
            case CMD_TEST_ALTITUDE:
                sendAck(true, "ALTITUDE_TEST_SUCCESS");
                break;
            case CMD_ENABLE_ALTITUDE_TEST:
                sendAck(true, "ALTITUDE_TEST_ENABLED");
                break;
            case CMD_DISABLE_ALTITUDE_TEST:
                sendAck(true, "ALTITUDE_TEST_DISABLED");
                break;
            case CMD_VELOCITY_TEST:
                sendAck(true, "VELOCITY_TEST_SUCCESS");
                break;
            case CMD_ENABLE_VELOCITY_TEST:
                sendAck(true, "VELOCITY_TEST_ENABLED");
                break;
            case CMD_DISABLE_VELOCITY_TEST:
                sendAck(true, "VELOCITY_TEST_DISABLED");
                break;
            default:
                {
                    const char* state = "UNKNOWN";
                    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                        state = stateManager.getStateString();
                        xSemaphoreGive(stateMutex);
                    }
                    sendAck(true, state);
                }
                break;
        }
    } else {
        // Get current state for better error messages
        uint8_t currentState = STATE_IDLE;
        if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            currentState = stateManager.getCurrentState();
            xSemaphoreGive(stateMutex);
        }
        
        if (cmd == CMD_ARM && currentState == STATE_ARMED) {
            sendAck(false, "ALREADY_ARMED");
        } else if (cmd == CMD_ENTER_TEST && currentState != STATE_IDLE && currentState != STATE_TEST) {
            sendAck(false, "MUST_BE_IDLE");
        } else if (cmd == CMD_ENTER_RECOVERY && currentState != STATE_ARMED && currentState != STATE_RECOVERY) {
            sendAck(false, "MUST_BE_ARMED");
        } else {
            sendAck(false, "DENIED");
        }
    }
    
    return success;
}

CommandType CmdParser::getCommandType(const char* cmdStr) {
    if (strcmp(cmdStr, "DISARM") == 0) return CMD_DISARM;
    if (strcmp(cmdStr, "ARM") == 0) return CMD_ARM;
    if (strcmp(cmdStr, "ENTER_TEST") == 0) return CMD_ENTER_TEST;
    if (strcmp(cmdStr, "ENTER_RECOVERY") == 0) return CMD_ENTER_RECOVERY;
    if (strcmp(cmdStr, "TEST") == 0) return CMD_TEST_DEVICE;
    if (strcmp(cmdStr, "SERVO_TEST") == 0) return CMD_TEST_SERVO;
    if (strcmp(cmdStr, "ALTITUDE_TEST") == 0) return CMD_TEST_ALTITUDE;
    if (strcmp(cmdStr, "ENABLE_ALTITUDE_TEST") == 0) return CMD_ENABLE_ALTITUDE_TEST;
    if (strcmp(cmdStr, "DISABLE_ALTITUDE_TEST") == 0) return CMD_DISABLE_ALTITUDE_TEST;
    if (strcmp(cmdStr, "VELOCITY_TEST") == 0) return CMD_VELOCITY_TEST;
    if (strcmp(cmdStr, "ENABLE_VELOCITY_TEST") == 0) return CMD_ENABLE_VELOCITY_TEST;
    if (strcmp(cmdStr, "DISABLE_VELOCITY_TEST") == 0) return CMD_DISABLE_VELOCITY_TEST;
    if (strcmp(cmdStr, "QUERY") == 0) return CMD_QUERY;
    if (strncmp(cmdStr, "CONTROL", 7) == 0) return CMD_CONTROL;
    if (strcmp(cmdStr, "NAVC_RESET_STATS") == 0) return CMD_NAVC_RESET_STATS;
    
    return CMD_QUERY;
}

bool CmdParser::isCommandAllowed(const char* cmdStr) {
    bool allowed = false;
    if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        allowed = stateManager.isCommandAllowed(getCommandType(cmdStr));
        xSemaphoreGive(stateMutex);
    }
    return allowed;
}

void CmdParser::sendAck(bool success, const char* message) {
    char ackBuffer[64];
    FrameCodec::formatCmdAck(ackBuffer, sizeof(ackBuffer), success, message);
    Serial.println(ackBuffer);
    
    #ifdef FC_BUILD
    extern LoraManager loraManager;
    if (xSemaphoreTake(loraMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (loraManager.isInitialized()) {
            loraManager.sendPacket(LORA_TYPE_CMD, ackBuffer, strlen(ackBuffer));
        }
        xSemaphoreGive(loraMutex);
    }
    #endif
}

uint16_t CmdParser::calculateChecksum(const char* cmd, size_t length) {
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

bool CmdParser::verifyChecksum(const char* cmd, uint16_t providedChecksum) {
    const char* lastColon = strrchr(cmd, ':');
    if (!lastColon) {
        return false;
    }
    
    size_t checksumLen = lastColon - cmd + 1;
    uint16_t calculatedChecksum = calculateChecksum(cmd, checksumLen);
    
    return calculatedChecksum == providedChecksum;
}

bool CmdParser::validateParameters(const char* cmdStr, const char* params) {
    if (strcmp(cmdStr, "CONTROL") == 0) {
        char key[16] = {0};
        int32_t value = 0;
        char paramsCopy[MAX_CMD_LENGTH];
        strncpy(paramsCopy, params, sizeof(paramsCopy) - 1);
        
        char* token = strtok(paramsCopy, ",");
        while (token != NULL) {
            if (!parseParams(token, key, sizeof(key), &value)) {
                return false;
            }
            
            if (strcmp(key, "servo") == 0) {
                if (value < 0 || value > 180) {
                    return false;
                }
            } else if (strcmp(key, "buzzer") == 0) {
                if (value < 0 || value > 1) {
                    return false;
                }
            }
            
            token = strtok(NULL, ",");
        }
        
        return true;
    } else if (strcmp(cmdStr, "ALTITUDE_TEST") == 0 || strcmp(cmdStr, "ENABLE_ALTITUDE_TEST") == 0) {
        char key[16] = {0};
        int32_t value = 0;
        char paramsCopy[MAX_CMD_LENGTH];
        strncpy(paramsCopy, params, sizeof(paramsCopy) - 1);
        
        char* token = strtok(paramsCopy, ",");
        if (token != NULL) {
            if (!parseParams(token, key, sizeof(key), &value)) {
                return false;
            }
            
            if (strcmp(key, "threshold") == 0) {
                if (strcmp(cmdStr, "ALTITUDE_TEST") == 0) {
                    if (value < 1 || value > 30000) {
                        return false;
                    }
                } else {
                    if (value < 1) {
                        return false;
                    }
                }
            } else {
                return false;
            }
        }
        
        return true;
    } else if (strcmp(cmdStr, "DISABLE_ALTITUDE_TEST") == 0) {
        return strlen(params) == 0;
    } else if (strcmp(cmdStr, "VELOCITY_TEST") == 0 || strcmp(cmdStr, "ENABLE_VELOCITY_TEST") == 0) {
        char key[16] = {0};
        int32_t value = 0;
        char paramsCopy[MAX_CMD_LENGTH];
        strncpy(paramsCopy, params, sizeof(paramsCopy) - 1);
        
        char* token = strtok(paramsCopy, ",");
        if (token != NULL) {
            if (!parseParams(token, key, sizeof(key), &value)) {
                return false;
            }
            
            if (strcmp(key, "threshold") == 0) {
                if (value < 1 || value > 50) {
                    return false;
                }
            } else {
                return false;
            }
        }
        
        return true;
    } else if (strcmp(cmdStr, "DISABLE_VELOCITY_TEST") == 0) {
        return strlen(params) == 0;
    }
    
    return true;
}

bool CmdParser::parseParams(const char* paramStr, char* key, size_t keySize, int32_t* value) {
    char* equals = strchr((char*)paramStr, '=');
    if (!equals) {
        return false;
    }
    
    size_t keyLen = equals - paramStr;
    if (keyLen >= keySize) {
        return false;
    }
    
    strncpy(key, paramStr, keyLen);
    key[keyLen] = '\0';
    
    *value = atol(equals + 1);
    
    return true;
}

bool CmdParser::processCommand(const char* cmd) {
    cmdIndex = 0;
    memset(cmdBuffer, 0, MAX_CMD_LENGTH);
    strncpy(cmdBuffer, cmd, MAX_CMD_LENGTH - 1);
    
    return parseAndExecute();
}