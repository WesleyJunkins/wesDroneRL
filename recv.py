#!/usr/bin/env python3
"""
Drone Command Sender
Connects to the drone's UDP port and sends movement and yaw commands in JSON format.
"""

import socket
import json
import time
import sys

# Configuration
DRONE_IP = "127.0.0.1"
DRONE_PORT = 8888
BUFFER_SIZE = 1024

def create_command_json(movement=0.0, yaw=0.0):
    """Create a JSON command with movement and yaw values."""
    command = {
        "movement": movement,
        "yaw": yaw
    }
    return json.dumps(command)

def send_command(sock, movement, yaw):
    """Send a command to the drone."""
    command_json = create_command_json(movement, yaw)
    try:
        sock.sendto(command_json.encode('utf-8'), (DRONE_IP, DRONE_PORT))
        print(f"Sent command: {command_json}")
        return True
    except Exception as e:
        print(f"Error sending command: {e}")
        return False

def main():
    """Main function to handle drone commands."""
    print("Drone Command Sender")
    print("=" * 50)
    print(f"Connecting to drone at {DRONE_IP}:{DRONE_PORT}")
    print()
    
    # Create UDP socket
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        print("Socket created successfully")
    except Exception as e:
        print(f"Error creating socket: {e}")
        return
    
    print("\nCommand Format: movement yaw")
    print("Examples:")
    print("  0.5 0.0    - Move forward at 0.5 velocity")
    print("  -0.3 0.0   - Move backward at 0.3 velocity")
    print("  0.0 1.0    - Rotate 1.0 radians")
    print("  0.5 0.5    - Move forward and rotate")
    print("  0.0 0.0    - Stop movement (keep current yaw)")
    print("  q           - Quit")
    print()
    
    try:
        while True:
            # Get user input
            user_input = input("Enter command (movement yaw): ").strip()
            
            # Check for quit command
            if user_input.lower() in ['q', 'quit', 'exit']:
                print("Quitting...")
                break
            
            # Parse input
            try:
                parts = user_input.split()
                if len(parts) != 2:
                    print("Error: Please enter exactly 2 values (movement yaw)")
                    continue
                
                movement = float(parts[0])
                yaw = float(parts[1])
                
                # Send command
                if send_command(sock, movement, yaw):
                    print("Command sent successfully!")
                else:
                    print("Failed to send command")
                    
            except ValueError:
                print("Error: Please enter valid numbers")
            except KeyboardInterrupt:
                print("\nQuitting...")
                break
            except Exception as e:
                print(f"Error: {e}")
    
    finally:
        sock.close()
        print("Socket closed")

if __name__ == "__main__":
    main()
