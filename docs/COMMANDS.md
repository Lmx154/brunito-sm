# Flight Controller Commands

This document lists all commands that can be sent to the Flight Controller (FC) in the Brunito project. These commands are used by the Ground Station (GS) via LoRa communication and by the USB-CDC debug interface (at 921600 bps) for monitoring, control, and debugging. Each command follows the ASCII-framed format `<CMD:command_name>` or `<CMD:command_name:parameters>` where applicable.

## Overview

Commands are processed by the FC based on its current state (IDLE, TEST, ARMED, RECOVERY) as defined in the finite-state machine (FSM). The GS and USB-CDC interfaces support the same command set, with no differences in availability. Commands are case-sensitive and must include the `<` and `>` delimiters. For GS usage, entering a command without delimiters (e.g., `ARM`) will automatically be framed as `<CMD:ARM>`.

## Command List

Below is the complete list of commands, including their descriptions, allowed states, and example usage.

### 1. `<CMD:ARM>`
- **Description**: Arms the FC, transitioning to the ARMED state and starting full telemetry at 20 Hz.
- **Allowed States**: IDLE, TEST
- **Example**: `<CMD:ARM>`

### 2. `<CMD:DISARM>`
- **Description**: Disarms the FC, returning to the IDLE state and stopping telemetry/logging.
- **Allowed States**: All (IDLE, TEST, ARMED, RECOVERY)
- **Example**: `<CMD:DISARM>`

### 3. `<CMD:ENTER_TEST>`
- **Description**: Enters TEST mode for pre-flight diagnostics, streaming test outputs at 50 Hz.
- **Allowed States**: IDLE
- **Example**: `<CMD:ENTER_TEST>`

### 4. `<CMD:ENTER_RECOVERY>`
- **Description**: Enters RECOVERY mode, activating location services (e.g., buzzer) and sending GPS-only telemetry at 1 Hz.
- **Allowed States**: ARMED
- **Example**: `<CMD:ENTER_RECOVERY>`

### 5. `<CMD:QUERY>`
- **Description**: Queries the FC's current state, returning a response like `<CMD_ACK:OK:state>`.
- **Allowed States**: All (IDLE, TEST, ARMED, RECOVERY)
- **Example**: `<CMD:QUERY>`

### 6. `<CMD:FIND_ME>`
- **Description**: Activates the buzzer and/or LED to aid in locating the vehicle, typically in RECOVERY mode.
- **Allowed States**: RECOVERY
- **Example**: `<CMD:FIND_ME>`

### 7. `<CMD:CONTROL:params>`
- **Description**: Controls actuators such as servos or buzzers. Parameters are specified as `key=value` pairs, separated by commas if multiple.
- **Supported Parameters**:
  - `servo=value` (0–180): Sets servo position in degrees.
  - `buzzer=value` (0 or 1): Turns buzzer off (0) or on (1).
- **Allowed States**: All except TEST
- **Examples**:
  - `<CMD:CONTROL:servo=90>`
  - `<CMD:CONTROL:buzzer=1>`

### 8. `<CMD:ALTITUDE_TEST>`
- **Description**: Tests altitude-based behavior - when altitude exceeds threshold, activates the buzzer and moves servo to 90 degrees and back.
- **Parameters**:
  - `threshold` (optional): The altitude threshold in centimeters. Default is 200cm (2m).
- **Altitude Format**: Altitude values are specified in centimeters. For example:
  - 200cm = 2 meters
  - 1000cm = 10 meters (approximately 33ft)
  - 3048cm = 30.48 meters (100ft)
  - 10000cm = 100 meters (approximately 328ft)
- **Allowed States**: TEST only
- **Examples**: 
  - `<CMD:ALTITUDE_TEST>` - Uses default 2m threshold
  - `<CMD:ALTITUDE_TEST:threshold=1000>` - Sets threshold to 10m (33ft)

### 9. `<CMD:ENABLE_ALTITUDE_TEST>`
- **Description**: Enables background altitude monitoring that runs continuously in TEST state. When altitude exceeds the specified threshold, automatically triggers servo sequence (move to 90°, then back to 0°) and then disables itself.
- **Parameters**:
  - `threshold` (required): The altitude threshold in centimeters (minimum 1cm, no upper limit).
- **Behavior**: 
  - Runs in background until altitude threshold is reached
  - Automatically executes servo test sequence when triggered
  - Self-disables after execution
  - OFF by default at startup
- **Allowed States**: TEST only
- **Examples**: 
  - `<CMD:ENABLE_ALTITUDE_TEST:threshold=1000>` - Monitor for 10m altitude
  - `<CMD:ENABLE_ALTITUDE_TEST:threshold=50000>` - Monitor for 500m altitude

### 10. `<CMD:DISABLE_ALTITUDE_TEST>`
- **Description**: Disables the background altitude monitoring system if currently active.
- **Parameters**: None
- **Allowed States**: TEST only
- **Example**: `<CMD:DISABLE_ALTITUDE_TEST>`

### 11. `<CMD:NAVC_RESET_STATS>`
- **Description**: Resets packet statistics counters on the Navigation Controller (NAVC), forwarded via the FC.
- **Allowed States**: All (IDLE, TEST, ARMED, RECOVERY)
- **Example**: `<CMD:NAVC_RESET_STATS>`

### 12. `<CMD:LORA_RESET_STATS>`
- **Description**: Resets LoRa communication statistics counters (e.g., packets sent, received, lost).
- **Allowed States**: All (IDLE, TEST, ARMED, RECOVERY)
- **Example**: `<CMD:LORA_RESET_STATS>`

### 13. `<CMD:LORA_STATS>`
- **Description**: Requests detailed LoRa communication statistics, including RSSI, SNR, and packet counts.
- **Allowed States**: All (IDLE, TEST, ARMED, RECOVERY)
- **Example**: `<CMD:LORA_STATS>`

### 14. `<CMD:TELEM_STATUS>`
- **Description**: Requests the current telemetry status, including system state, telemetry state, rate, and RSSI.
- **Allowed States**: All (IDLE, TEST, ARMED, RECOVERY)
- **Example**: `<CMD:TELEM_STATUS>`

### 15. `<CMD:TEST>`
- **Description**: Performs testing functions in TEST mode. Currently enables the buzzer on the NAVC board (pin A0) for testing purposes. Future enhancements will add more test functionality.
- **Allowed States**: TEST only
- **Example**: `<CMD:TEST>`

### 16. `<CMD:SERVO_TEST>`
- **Description**: Tests the servo on pin PB14 of the Flight Controller by moving it to 90 degrees and back to 0 degrees.
- **Allowed States**: TEST only
- **Example**: `<CMD:SERVO_TEST>`

## Usage Notes

- **Command Format**: Commands must be sent as `<CMD:command_name>` or `<CMD:command_name:parameters>`. Ensure proper formatting, as the FC expects exact syntax.
- **State Restrictions**: Commands are only executed if the FC is in an allowed state (see above). Invalid commands return an error response (e.g., `<CMD_ACK:ERROR:invalid_state>`).
- **USB-CDC Interface**: Connect via a serial terminal (e.g., PuTTY, minicom) at 921600 bps to send commands. The interface also outputs debug messages and telemetry.
- **GS Interface**: Commands sent via LoRa are forwarded to the FC. The GS command-line interface automatically adds `<CMD:` and `>` if omitted.
- **Parameter Validation**: For `<CMD:CONTROL>`, only `servo` (0–180) and `buzzer` (0 or 1) are currently validated. Future extensions may add more parameters.
- **Special Commands**: `<CMD:LORA_RESET_STATS>`, `<CMD:LORA_STATS>`, and `<CMD:TELEM_STATUS>` are handled directly in `FC.cpp` but are fully supported by both interfaces.

## Source Files

This documentation is based on the following project files:
- `GS.cpp`: Lists commands in `printHelp()`.
- `CmdParser.cpp`: Parses and maps commands to `CommandType` enums.
- `State.h`: Defines `CommandType` and state-specific command allowances.
- `FC.cpp`: Handles command processing, including special commands.
- `COMMUNICATION.md`: Describes command formats and FSM behavior.

## Last Updated

May 17, 2025