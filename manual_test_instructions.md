# Manual Testing Instructions for Phase 2 - Command Parser & Routing

This document provides step-by-step instructions to manually test the Phase 2 implementation (command parsing, parameter validation, and checksum verification) on the actual hardware.

## Prerequisites
1. Flight Controller (FC) board flashed with the latest firmware
2. Serial monitor (like PlatformIO Serial Monitor or another terminal program)
3. Serial connection at 921600 baud

## Test Setup
1. Connect the FC board via USB
2. Open a serial monitor at 921600 baud
3. Run `pio run -e fc -t upload` to upload the latest firmware

## Test Cases

### 1. Basic Command Parsing

Send these commands and verify the responses:

```
<CMD:QUERY>
```
Expected: `<CMD_ACK:OK:IDLE>` (or current state)

```
<CMD:ENTER_TEST>
```
Expected: `<CMD_ACK:OK:TEST>`

```
<CMD:DISARM>
```
Expected: `<CMD_ACK:OK:IDLE>`

### 2. Invalid Command Format Tests

```
CMD:QUERY>
```
Expected: `<CMD_ACK:ERR:INVALID_FORMAT>`

```
<CMD:QUERY
```
Expected: No response (incomplete command) or error

```
<CMD:INVALID_COMMAND>
```
Expected: `<CMD_ACK:ERR:DENIED>` or similar rejection message

### 3. Parameter Validation Tests

```
<CMD:CONTROL:servo=90>
```
Expected: `<CMD_ACK:OK:IDLE>` (or current state)

```
<CMD:CONTROL:servo=200>
```
Expected: `<CMD_ACK:ERR:INVALID_PARAMS>` (out of range)

```
<CMD:CONTROL:unknown_param=10>
```
Expected: `<CMD_ACK:ERR:INVALID_PARAMS>` or rejection

```
<CMD:CONTROL:servo=90,buzzer=1>
```
Expected: `<CMD_ACK:OK:IDLE>` (or current state)

### 4. Checksum Validation Tests

To test checksums, you'll need to calculate the CRC16-CCITT checksum for your commands. Here's a process:

1. Calculate the checksum for `<CMD:ARM:`:
   - For example, use an online CRC calculator or this Python code:
   ```python
   def crc16(data):
       crc = 0xFFFF
       for b in data:
           crc ^= (ord(b) << 8)
           for _ in range(8):
               if crc & 0x8000:
                   crc = ((crc << 1) ^ 0x1021) & 0xFFFF
               else:
                   crc = (crc << 1) & 0xFFFF
       return crc
   
   cmd = "<CMD:ARM:"
   print(f"Checksum: {crc16(cmd):04X}")
   ```

2. Send commands with valid and invalid checksums:
   ```
   <CMD:ARM::7D67>
   ```
   Expected: `<CMD_ACK:OK:ARMED>` (if checksum matches)
   
   ```
   <CMD:ARM::0000>
   ```
   Expected: `<CMD_ACK:ERR:CHECKSUM_ERROR>` (if checksum doesn't match)

## State Transition Tests

Test the full FSM cycle with commands:

1. `<CMD:DISARM>` (ensure we start in IDLE)
2. `<CMD:ENTER_TEST>` (should transition to TEST)
3. `<CMD:ARM>` (should transition to ARMED)
4. `<CMD:ENTER_RECOVERY>` (should transition to RECOVERY)
5. `<CMD:DISARM>` (should transition back to IDLE)

Verify LED patterns change according to the state transitions.

## Verification Criteria (Phase 2 Requirements)

According to the Roadmap, Phase 2 is complete when:
- All valid frames are properly acknowledged
- All invalid frames are rejected with appropriate error messages
- Parameter validation works correctly
- Checksum verification works correctly

To fully validate Phase 2, test with at least 10-20 different commands with various combinations of parameters and checksums.
