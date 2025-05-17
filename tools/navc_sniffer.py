#!/usr/bin/env python3
"""
NAVC Binary Packet Sniffer

This script sniffs the UART connection between NAVC and FC boards to validate
binary packet reception. It helps verify that the packet loss rate is < 1% over
a 10-minute bench run.

Usage:
  python navc_sniffer.py COM_PORT [BAUD_RATE]

Default BAUD_RATE is 115200.
"""

import serial
import struct
import time
import sys
import argparse
import datetime
from collections import deque
import signal

# Define packet structure (38 bytes total)
# Matches the SensorPacket struct in Sensors.h
PACKET_FORMAT = '<HI3h6h2iH'  # uint16, uint32, 3xi16, 6xi16, 2xi32, uint16 (CRC)
PACKET_SIZE = 38

# Parameters for statistics
WINDOW_SIZE = 100  # Number of packets to keep for rolling stats
MAX_EXPECTED_GAP = 10  # Maximum expected gap in packet IDs

class PacketSniffer:
    def __init__(self, port, baud_rate=115200):
        self.port = port
        self.baud_rate = baud_rate
        self.ser = None
        
        # Statistics
        self.packets_received = 0
        self.crc_errors = 0
        self.sequence_errors = 0
        self.buffer_overflows = 0
        self.running = True
        
        # Tracking
        self.last_packet_id = None
        self.start_time = None
        self.last_stats_time = 0
        self.last_timestamps = deque(maxlen=WINDOW_SIZE)
    
    def calculate_crc16(self, data):
        """Calculate CRC16-CCITT (0xFFFF) for data."""
        crc = 0xFFFF
        for b in data:
            crc ^= (b << 8)
            for _ in range(8):
                if crc & 0x8000:
                    crc = ((crc << 1) ^ 0x1021) & 0xFFFF
                else:
                    crc = (crc << 1) & 0xFFFF
        return crc
    
    def connect(self):
        """Connect to the serial port."""
        try:
            self.ser = serial.Serial(self.port, self.baud_rate, timeout=1)
            print(f"Connected to {self.port} at {self.baud_rate} baud")
            return True
        except serial.SerialException as e:
            print(f"Error opening serial port: {e}")
            return False
    
    def read_packet(self):
        """Read a single packet from the serial port."""
        if not self.ser or not self.ser.is_open:
            return None
            
        # Read exactly PACKET_SIZE bytes
        data = self.ser.read(PACKET_SIZE)
        
        if len(data) != PACKET_SIZE:
            # Timeout or incomplete packet
            if len(data) > 0:
                # Discard partial data and realign
                self.ser.reset_input_buffer()
                self.buffer_overflows += 1
            return None
        
        # Calculate and verify CRC
        calculated_crc = self.calculate_crc16(data[:-2])
        packet_crc = struct.unpack('<H', data[-2:])[0]
        
        if calculated_crc != packet_crc:
            print(f"CRC Error: calculated 0x{calculated_crc:04X}, received 0x{packet_crc:04X}")
            self.crc_errors += 1
            return None
        
        # Parse the packet
        try:
            packet = struct.unpack(PACKET_FORMAT, data)
            return packet
        except struct.error as e:
            print(f"Error unpacking packet: {e}")
            return None
    
    def process_packet(self, packet):
        """Process a received packet."""
        if not packet:
            return
        
        # Extract fields
        packet_id, timestamp, alt, ax, ay, az, gx, gy, gz, mx, my, mz, lat, lon, crc = packet
        
        # Check sequence
        if self.last_packet_id is not None:
            expected_id = (self.last_packet_id + 1) & 0xFFFF
            if packet_id != expected_id:
                gap = (packet_id - expected_id) & 0xFFFF
                if gap > 0 and gap < MAX_EXPECTED_GAP:
                    self.sequence_errors += gap
                    print(f"Sequence gap: expected {expected_id}, got {packet_id} (gap of {gap})")
                elif gap == 0:
                    print(f"Duplicate packet ID: {packet_id}")
        
        self.last_packet_id = packet_id
        self.packets_received += 1
        self.last_timestamps.append(time.time())
        
        # Initialize start time on first packet
        if self.start_time is None:
            self.start_time = time.time()
        
        # Print packet data occasionally
        if self.packets_received % 100 == 0:
            print(f"Packet #{packet_id}: Alt={alt/100:.2f}m, Accel=({ax/1000:.2f},{ay/1000:.2f},{az/1000:.2f})g")
            
        # Print statistics periodically
        current_time = time.time()
        if current_time - self.last_stats_time >= 5:  # Every 5 seconds
            self.print_statistics()
            self.last_stats_time = current_time
    
    def calculate_packet_rate(self):
        """Calculate current packet rate based on recent timestamps."""
        if len(self.last_timestamps) < 2:
            return 0
        
        time_delta = self.last_timestamps[-1] - self.last_timestamps[0]
        if time_delta <= 0:
            return 0
            
        return (len(self.last_timestamps) - 1) / time_delta
    
    def print_statistics(self):
        """Print current statistics."""
        if self.start_time is None:
            return
            
        runtime = time.time() - self.start_time
        hours, remainder = divmod(runtime, 3600)
        minutes, seconds = divmod(remainder, 60)
        
        total_errors = self.crc_errors + self.sequence_errors
        error_rate = (total_errors / max(1, self.packets_received + total_errors)) * 100
        
        packet_rate = self.calculate_packet_rate()
        
        print("\n--- NAVC Packet Statistics ---")
        print(f"Runtime: {int(hours):02d}:{int(minutes):02d}:{int(seconds):02.1f}")
        print(f"Packets received: {self.packets_received}")
        print(f"Current rate: {packet_rate:.2f} packets/sec")
        print(f"CRC errors: {self.crc_errors}")
        print(f"Sequence errors: {self.sequence_errors}")
        print(f"Buffer overflows: {self.buffer_overflows}")
        print(f"Total error rate: {error_rate:.3f}%")
        
        # Report pass/fail based on error rate
        if runtime > 60 and self.packets_received > 100:
            if error_rate < 1.0:
                print("STATUS: PASS - Error rate < 1%")
            else:
                print("STATUS: FAIL - Error rate > 1%")
        else:
            print("STATUS: COLLECTING DATA...")
        
        print(f"Last packet ID: {self.last_packet_id}")
        print("-----------------------------\n")
    
    def run(self):
        """Main processing loop."""
        if not self.connect():
            return
            
        print("Starting packet capture...")
        print("Press Ctrl+C to exit")
        
        try:
            while self.running:
                packet = self.read_packet()
                if packet:
                    self.process_packet(packet)
        except KeyboardInterrupt:
            print("\nCapture stopped by user")
        finally:
            self.print_statistics()
            if self.ser and self.ser.is_open:
                self.ser.close()
                print(f"Connection to {self.port} closed")
    
    def stop(self):
        """Stop the capture."""
        self.running = False


def parse_arguments():
    parser = argparse.ArgumentParser(description='NAVC Binary Packet Sniffer')
    parser.add_argument('port', help='Serial port (e.g., COM3 or /dev/ttyACM0)')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate (default: 115200)')
    return parser.parse_args()


def signal_handler(sig, frame):
    print("\nCapture stopped by user")
    sys.exit(0)


if __name__ == "__main__":
    args = parse_arguments()
    
    signal.signal(signal.SIGINT, signal_handler)
    
    sniffer = PacketSniffer(args.port, args.baud)
    sniffer.run()
