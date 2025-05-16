# Brunito Communication Architecture

## Overview

This document describes the communication architecture between the three main components of the Brunito project:
- **Navigation Controller (NAVC)**: Manages sensors and autonomous flight
- **Flight Controller (FC)**: Acts as middleware between NAVC and ground systems
- **Ground Station (GS)**: Provides command and control interface

The architecture supports multiple communication modes including command-response patterns and high-frequency telemetry streaming.

```
┌────────────┐    UART2    ┌────────────┐   USB-CDC   ┌────────────┐
│    NAVC    │<----------->│     FC     │<----------->│    User    │
└────────────┘             └────────────┘             └────────────┘
                                 │
                                 │ LoRa
                                 ▼
                           ┌────────────┐
                           │Ground Station│
                           └────────────┘
```

## Message Protocol

### 1. Message Framing

All messages use a consistent framing format:

```
<TYPE:PAYLOAD>
```

Where:
- `<` and `>` are frame delimiters
- `TYPE` identifies the message category
- `:` separates type from payload
- `PAYLOAD` contains the message data

### 2. Message Types

| Type | Description | Example |
|------|-------------|---------|
| `CMD` | Command or command response | `<CMD:DISARM>` |
| `TEST` | Test-related output | `<TEST:BMP280_OK>` |
| `TELEM` | Telemetry data | `<TELEM:ALT=1000,TEMP=25>` |
| `ALERT` | Critical notifications | `<ALERT:LOW_BATTERY>` |
| `DEBUG` | Debug information | `<DEBUG:INIT_SENSORS>` |

## State-Based Communication

The system operates in three main states, each with specific communication behaviors:

### IDLE State
- Full command set available
- Low-frequency status updates
- No telemetry streaming

### TEST State
- Limited command set (test commands only)
- High-frequency test outputs
- No telemetry streaming
- All test outputs forwarded to both Serial and LoRa

### ARMED State
- Full command set except TEST commands
- High-frequency telemetry streaming
- Command responses prioritized over telemetry
- Critical commands (e.g., DISARM) given highest priority

## Command Processing

### Command Priority Levels

1. **Critical** (Highest): Safety-critical commands like DISARM
2. **Control**: Flight control commands (servo adjustments, etc.)
3. **Query**: Status requests and non-critical commands
4. **Test**: Test related commands (lowest priority when ARMED)

### Command Flow

1. FC receives commands from USB-CDC or LoRa
2. FC filters commands based on current state
3. Valid commands are forwarded to NAVC via UART2
4. NAVC processes commands and sends responses back to FC
5. FC forwards responses to both USB-CDC and LoRa

## Telemetry Streaming

### Implementation Strategy

1. **Activation**: Telemetry streaming is activated only in ARMED state
2. **Frequency Control**: Adjustable transmission rate to balance detail vs. bandwidth
3. **Suspend/Resume**: Can be temporarily suspended for critical command processing

### Telemetry Message Structure

```
<TELEM:timestamp,alt,temp,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,mag_x,mag_y,mag_z>
```

## Implementation Recommendations

### 1. Unified Output Handler

Implement a message router on FC that processes and routes all messages based on type:

```cpp
void sendToAllOutputs(String messageType, String payload) {
  String framedMessage = "<" + messageType + ":" + payload + ">";
  Serial.println(framedMessage);  // Send to USB-CDC
  lora.send(framedMessage);       // Send to LoRa
}
```

### 2. Non-Blocking I/O Processing

Ensure all I/O operations are non-blocking to maintain system responsiveness:

```cpp
void loop() {
  // Process incoming messages without blocking
  processSerialCommands();
  processLoRaMessages();
  
  // Process NAVC responses
  processNavcResponses();
  
  // Handle telemetry if in ARMED state
  if (currentState == STATE_ARMED) {
    processTelemetry();
  }
}
```

### 3. Command Buffer with Priority Queue

Implement a priority queue to ensure critical commands are processed first:

```cpp
struct Command {
  String payload;
  int priority;
  unsigned long timestamp;
};

// Compare function for priority queue
bool compareCommands(Command a, Command b) {
  // Higher priority number = higher priority
  if (a.priority != b.priority) return a.priority > b.priority;
  
  // If same priority, process oldest first
  return a.timestamp < b.timestamp;
}
```

### 4. Telemetry Rate Control

Implement adaptive telemetry rate control based on system load and bandwidth:

```cpp
void adjustTelemetryRate() {
  // Base rate in ms between transmissions
  int baseRate = 50;  // 20Hz default
  
  // If command processing is active, reduce rate
  if (commandsInQueue > 0) {
    baseRate = 100;  // 10Hz when commands are pending
  }
  
  // If critical operation, reduce further
  if (criticalOperationInProgress) {
    baseRate = 200;  // 5Hz during critical operations
  }
  
  telemetryInterval = baseRate;
}
```

### 5. Acknowledge Critical Commands

Always acknowledge receipt of critical commands:

```cpp
void processCriticalCommand(String command) {
  // Send immediate acknowledgment
  sendToAllOutputs("ACK", command);
  
  // Process command
  executeCommand(command);
  
  // Send completion notification
  sendToAllOutputs("DONE", command);
}
```

## Integration Testing

Test the communication system thoroughly, especially:

1. Command handling during high-rate telemetry
2. State transitions while commands are pending
3. System behavior under communication errors
4. Recovery from communication failures
5. Maximum sustainable telemetry rate

## Future Enhancements

1. **Error Correction**: Add checksums or CRC to catch transmission errors
2. **Message Compression**: Implement binary protocol for telemetry to reduce bandwidth
3. **Buffering**: Add message buffering for telemetry during communication outages
4. **Heartbeat**: Implement heartbeat mechanism to detect component failures
5. **Encryption**: Add encryption for sensitive commands if required
