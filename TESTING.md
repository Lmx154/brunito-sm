# Phase 2 Testing Tools

This directory contains tools to manually test the Phase 2 implementation (Command Parser & Routing) on the actual hardware.

## Testing with the Hardware

Since the automated tests are experiencing issues, we've created tools to manually test the command parser on the actual hardware:

### 1. Manual Test Instructions

See the `manual_test_instructions.md` file for detailed instructions on how to test various aspects of the command parser:
- Basic command handling
- Parameter validation
- Checksum verification
- State transitions

### 2. Command Generator Tool

The `test_command_generator.py` script helps you generate valid commands for testing, including properly calculated checksums.

#### Requirements
- Python 3
- pyserial library (`pip install pyserial`)

#### Usage Examples

1. Generate a list of all test commands:
```
python test_command_generator.py --test-all
```

2. Generate and show a basic command:
```
python test_command_generator.py --command ARM
```

3. Generate a command with parameters:
```
python test_command_generator.py --command CONTROL --params "servo=90,buzzer=1"
```

4. Generate a command with a checksum:
```
python test_command_generator.py --command ARM --checksum
```

5. Send a command to the flight controller:
```
python test_command_generator.py --command ARM --port COM3 --send
```

6. Run a set of basic command tests and send them to the device:
```
python test_command_generator.py --test-basic --port COM3 --send
```

7. Run parameter validation tests:
```
python test_command_generator.py --test-params --port COM3 --send
```

8. Run checksum validation tests:
```
python test_command_generator.py --test-checksum --port COM3 --send
```

### Phase 2 Verification Checklist

According to the Roadmap, Phase 2 is complete when:
- All valid frames are properly acknowledged
- All invalid frames are rejected with appropriate error messages
- Parameter validation works correctly
- Checksum verification works correctly

Use the testing tools to verify these requirements on the hardware.
