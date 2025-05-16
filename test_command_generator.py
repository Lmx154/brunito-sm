#!/usr/bin/env python3
"""
Command Generator Tool for Brunito FC Testing

This script helps generate properly formatted commands with valid CRC16-CCITT checksums
for testing the Brunito flight controller command parser.
"""

import argparse
import serial
import time

def calculate_crc16(data):
    """Calculate CRC16-CCITT checksum for a string"""
    crc = 0xFFFF
    for b in data:
        if isinstance(b, str):
            b = ord(b)
        crc ^= (b << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc

def format_cmd(command, params=None, include_checksum=False):
    """Format a command with optional parameters and checksum"""
    if params:
        cmd_str = f"<CMD:{command}:{params}"
    else:
        cmd_str = f"<CMD:{command}"
    
    if include_checksum:
        # Calculate checksum for everything up to this point plus the colon
        checksum_str = cmd_str + ":"
        checksum = calculate_crc16(checksum_str)
        cmd_str = f"{cmd_str}:{checksum:04X}>"
    else:
        cmd_str = f"{cmd_str}>"
    
    return cmd_str

def send_command(port, command, wait_for_response=True):
    """Send a command to the serial port and optionally wait for response"""
    try:
        ser = serial.Serial(port, 921600, timeout=1)
        print(f"Sending: {command}")
        ser.write(command.encode() + b'\n')  # Add newline for easier manual testing
        
        if wait_for_response:
            # Wait for response with timeout
            start_time = time.time()
            response = ""
            
            while time.time() - start_time < 2:  # 2 second timeout
                if ser.in_waiting:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    print(f"Received: {line}")
                    if "<CMD_ACK" in line:
                        response = line
                        break
                time.sleep(0.1)
                
            if not response:
                print("No acknowledgment received within timeout")
        
        ser.close()
    except Exception as e:
        print(f"Serial communication error: {e}")

def main():
    parser = argparse.ArgumentParser(description='Generate and send commands to Brunito FC')
    parser.add_argument('--command', '-c', type=str, help='Command name (e.g., ARM, DISARM, ENTER_TEST)')
    parser.add_argument('--params', '-p', type=str, help='Optional parameters (e.g., "servo=90,buzzer=1")')
    parser.add_argument('--checksum', '-s', action='store_true', help='Include checksum')
    parser.add_argument('--port', type=str, help='Serial port (e.g., COM3)')
    parser.add_argument('--send', '-x', action='store_true', help='Send the command to the serial port')
    
    # Command presets for testing
    parser.add_argument('--test-all', action='store_true', help='Generate all test commands')
    parser.add_argument('--test-basic', action='store_true', help='Test basic commands')
    parser.add_argument('--test-params', action='store_true', help='Test parameter validation')
    parser.add_argument('--test-checksum', action='store_true', help='Test checksum validation')
    
    args = parser.parse_args()
    
    if args.test_all:
        # Generate a test report with all commands
        print("=== BRUNITO COMMAND TEST CASES ===")
        
        print("\n--- Basic Commands ---")
        for cmd in ['QUERY', 'ARM', 'DISARM', 'ENTER_TEST', 'ENTER_RECOVERY', 'TEST']:
            formatted = format_cmd(cmd)
            print(f"{formatted}")
            
        print("\n--- Commands with Parameters ---")
        print(format_cmd("CONTROL", "servo=90"))
        print(format_cmd("CONTROL", "servo=180,buzzer=1"))
        print(format_cmd("CONTROL", "servo=0,buzzer=0"))
        print("# Invalid parameter tests:")
        print(format_cmd("CONTROL", "servo=200"))
        print(format_cmd("CONTROL", "unknown=10"))
            
        print("\n--- Commands with Checksums ---")
        for cmd in ['ARM', 'DISARM', 'QUERY']:
            formatted = format_cmd(cmd, None, True)
            print(f"{formatted}")
        
        print(format_cmd("CONTROL", "servo=90,buzzer=1", True))
        
        print("\n--- Invalid Format Tests ---")
        print("CMD:QUERY>  # Missing opening bracket")
        print("<CMD:QUERY  # Missing closing bracket")
        print("<CMD:>  # Missing command name")
        print("<CMD:INVALID_COMMAND>  # Invalid command name")
        
        return
    
    if args.test_basic:
        cmds = ['QUERY', 'ARM', 'DISARM', 'ENTER_TEST', 'ENTER_RECOVERY', 'TEST']
        for cmd in cmds:
            formatted = format_cmd(cmd)
            print(formatted)
            if args.send and args.port:
                send_command(args.port, formatted)
                time.sleep(0.5)  # Small delay between commands
        return
    
    if args.test_params:
        param_tests = [
            ("CONTROL", "servo=90"),
            ("CONTROL", "servo=180,buzzer=1"),
            ("CONTROL", "servo=0,buzzer=0"),
            ("CONTROL", "servo=200"),  # Invalid
            ("CONTROL", "unknown=10")  # Invalid
        ]
        for cmd, params in param_tests:
            formatted = format_cmd(cmd, params)
            print(formatted)
            if args.send and args.port:
                send_command(args.port, formatted)
                time.sleep(0.5)
        return
    
    if args.test_checksum:
        checksum_tests = ['ARM', 'DISARM', 'QUERY', ('CONTROL', 'servo=90')]
        for test in checksum_tests:
            if isinstance(test, tuple):
                cmd, params = test
                formatted = format_cmd(cmd, params, True)
            else:
                formatted = format_cmd(test, None, True)
            print(formatted)
            if args.send and args.port:
                send_command(args.port, formatted)
                time.sleep(0.5)
                
            # Also test with invalid checksum
            if isinstance(test, tuple):
                cmd, params = test
                formatted = f"<CMD:{cmd}:{params}:0000>"
            else:
                formatted = f"<CMD:{test}::0000>"
            print(f"{formatted} # Invalid checksum")
            if args.send and args.port:
                send_command(args.port, formatted)
                time.sleep(0.5)
        return
    
    if args.command:
        formatted = format_cmd(args.command, args.params, args.checksum)
        print(formatted)
        
        if args.send and args.port:
            send_command(args.port, formatted)
    else:
        parser.print_help()

if __name__ == "__main__":
    main()
