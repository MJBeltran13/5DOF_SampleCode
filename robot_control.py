import numpy as np
from flask import Flask, render_template, request, jsonify
import math
import serial
import serial.tools.list_ports
import time
from collections import deque
from datetime import datetime
import traceback
import pygame
import threading
import socket
import webbrowser

def get_local_ip():
    try:
        # Create a socket object
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # Connect to an external server (doesn't actually send any data)
        s.connect(('8.8.8.8', 80))
        # Get the local IP address
        local_ip = s.getsockname()[0]
        s.close()
        return f"{local_ip}:5000"
    except Exception:
        return '127.0.0.1:5000'

app = Flask(__name__)

class ScaraRobot:
    def __init__(self):
        # Default arm configurations in mm
        self.link1 = 85.47  # Link 1 length (L1)
        self.link2 = 86.87  # Link 2 length (L2)
        
        # Current positions
        self.current_x = -86.87  # Initial X position
        self.current_y = 85.47   # Initial Y position
        self.current_z = 0       # Initial Z position
        self.current_yaw = 0     # Initial Yaw position
        self.theta1 = 90         # Base rotation
        self.theta2 = 90         # First arm joint
        
        # Gripper state
        self.is_gripper_open = True
        self.gripper_rotation = 90  # Initial gripper rotation angle (0-180)
        
        # Manual control state
        self.manual_control = None
        self.is_manual_mode = False
        
        # Serial communication history
        self.serial_history = deque(maxlen=100)  # Keep last 100 messages
        
        # Serial connection settings
        self.com_port = '/dev/ttyUSB0'  # Default port for Raspberry Pi
        self.baudrate = 115200   # Set default baudrate to 115200
        self.ser = None
        self.last_error = None
        self.status = "Not connected"
        
        # Add initial status to history
        self.add_to_history("System", "Robot control initialized. Please connect to a serial port.")
        
        # Removed auto-connect on initialization

    def disconnect_serial(self):
        """Disconnect from serial port"""
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
                time.sleep(0.5)  # Wait for port to fully close
            self.ser = None
            self.status = f"Disconnected from {self.com_port}"
            self.add_to_history("System", self.status)
            return True
        except Exception as e:
            self.status = f"Error disconnecting: {str(e)}"
            self.add_to_history("Error", self.status)
            return False

    def read_serial_response(self):
        """Read and decode serial response with multiple encoding attempts"""
        try:
            if not self.ser or not self.ser.is_open:
                return None

            # Wait for data to be available
            start_time = time.time()
            while time.time() - start_time < 1.0:  # Timeout after 1 second
                if self.ser and self.ser.is_open and self.ser.in_waiting > 0:
                    break
                time.sleep(0.01)
            else:
                return None  # Timeout reached

            if not self.ser or not self.ser.is_open:
                return None

            raw_response = self.ser.readline()
            if not raw_response:
                return None

            # Try different encodings
            encodings = ['utf-8', 'ascii', 'iso-8859-1', 'cp1252']
            response = None
            
            for encoding in encodings:
                try:
                    response = raw_response.decode(encoding).strip()
                    if response:  # If we got a valid string
                        break
                except UnicodeDecodeError:
                    continue
            
            # If all encodings fail, use hex representation
            if response is None:
                response = f"Raw response (hex): {raw_response.hex()}"
            
            return response
        except Exception as e:
            return f"Error reading response: {str(e)}"

    def connect_serial(self, port=None, baud=None):
        """Connect to serial port with given settings"""
        try:
            if port:
                self.com_port = port
            if baud:
                self.baudrate = baud

            # If already connected to the same port, disconnect first
            if self.ser and self.ser.is_open:
                if self.ser.port == self.com_port:
                    self.add_to_history("System", "Already connected to this port. Disconnecting first...")
                    self.disconnect_serial()
                    time.sleep(1)  # Wait a bit before reconnecting

            # Import serial here to ensure it's available
            import serial

            try:
                self.ser = serial.Serial(
                    port=self.com_port,
                    baudrate=self.baudrate,
                    timeout=1,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE
                )
            except serial.SerialException as e:
                self.ser = None
                if "PermissionError" in str(e) or "Access is denied" in str(e):
                    self.status = f"Port {self.com_port} is in use or access denied. Please disconnect first."
                else:
                    self.status = f"Failed to connect to {self.com_port}: {str(e)}"
                self.add_to_history("Error", self.status)
                return False

            # Verify the connection is open
            if not self.ser or not self.ser.is_open:
                self.status = f"Failed to open connection to {self.com_port}"
                self.add_to_history("Error", self.status)
                return False

            # Wait for Arduino/ESP32 to reset and send startup messages
            time.sleep(3)  # Increased from 2 to 3 seconds for Raspberry Pi
            
            # Clear any pending data
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            
            # Read any available startup messages
            start_time = time.time()
            while time.time() - start_time < 3:  # Wait up to 3 seconds for startup messages
                if self.ser and self.ser.is_open and self.ser.in_waiting:
                    response = self.read_serial_response()
                    if response:
                        self.add_to_history("← Startup", response)
                time.sleep(0.1)
            
            # Send local IP address to ESP32
            local_ip = get_local_ip()
            ip_command = f"IP:{local_ip}\n"
            
            # Try sending the IP command up to 3 times
            max_attempts = 3
            for attempt in range(max_attempts):
                if self.ser and self.ser.is_open:
                    self.add_to_history("→ Sent", f"IP command (attempt {attempt+1}): {ip_command.strip()}")
                    self.ser.write(ip_command.encode())
                    self.ser.flush()  # Ensure the command is sent
                    
                    # Give more time for ESP32 to process and respond
                    time.sleep(0.5)  # Increased from 0.1 to 0.5 seconds
                    
                    # Read acknowledgment with multiple attempts
                    for _ in range(5):  # Try reading response up to 5 times
                        if self.ser and self.ser.is_open and self.ser.in_waiting > 0:
                            response = self.read_serial_response()
                            if response:
                                self.add_to_history("← Received", response)
                                if "OK" in response or "ACK" in response or "IP" in response:
                                    self.add_to_history("System", f"ESP32 acknowledged IP: {local_ip}")
                                    break
                        time.sleep(0.1)
                    
                    # If we got a response with OK, break the retry loop
                    if response and ("OK" in response or "ACK" in response or "IP" in response):
                        break
                        
                    # If this was the last attempt and still no acknowledgment
                    if attempt == max_attempts - 1:
                        self.add_to_history("Warning", "ESP32 did not acknowledge IP address after multiple attempts")
                else:
                    break
                    
            self.status = f"Connected to {self.com_port} at {self.baudrate} baud"
            self.add_to_history("System", self.status)
            return True

        except ImportError:
            self.status = "Failed to import serial module. Please install pyserial: pip install pyserial"
            self.add_to_history("System", self.status)
            return False
        except Exception as e:
            self.ser = None
            self.status = f"Failed to connect: {str(e)}"
            self.add_to_history("System", self.status)
            return False

    def get_available_ports(self):
        """Get list of available COM ports"""
        ports = []
        
        # Create a list to store all detected ports
        detected_ports = []
        for port in serial.tools.list_ports.comports():
            detected_ports.append({
                'port': port.device,
                'description': port.description,
                'hwid': port.hwid
            })
        
        # Always add /dev/ttyUSB0 as the first option
        ttyUSB0_exists = False
        for port in detected_ports:
            if port['port'] == '/dev/ttyUSB0':
                ttyUSB0_exists = True
                ports.append(port)
                break
        
        if not ttyUSB0_exists:
            ports.append({
                'port': '/dev/ttyUSB0',
                'description': 'Raspberry Pi USB Serial Port',
                'hwid': 'N/A'
            })
        
        # Add all other detected ports
        for port in detected_ports:
            if port['port'] != '/dev/ttyUSB0':
                ports.append(port)
        
        # Add other common Raspberry Pi ports if they're not already in the list
        raspbi_ports = [
            '/dev/ttyUSB1', 
            '/dev/ttyACM0', '/dev/ttyACM1',
            '/dev/ttyAMA0', '/dev/serial0'
        ]
        
        existing_ports = [p['port'] for p in ports]
        for port in raspbi_ports:
            if port not in existing_ports:
                ports.append({
                    'port': port,
                    'description': 'Raspberry Pi port',
                    'hwid': 'N/A'
                })
        
        return ports

    def get_current_settings(self):
        """Get current serial port settings"""
        return {
            'com_port': self.com_port,
            'baudrate': self.baudrate,
            'connected': bool(self.ser and self.ser.is_open),
            'available_ports': self.get_available_ports(),
            'available_baudrates': [1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200]
        }

    def add_to_history(self, direction, message):
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.serial_history.append({
            'timestamp': timestamp,
            'direction': direction,
            'message': message
        })

    def send_command(self, cmd):
        """Send command to Arduino with error handling"""
        try:
            if not self.ser or not self.ser.is_open:
                self.last_error = "Not connected to Arduino"
                self.add_to_history("Error", self.last_error)
                # Try to reconnect
                if self.connect_serial():
                    self.add_to_history("System", "Reconnected successfully")
                else:
                    return None

            # Add newline if not present
            if not cmd.endswith('\n'):
                cmd = cmd + '\n'

            self.add_to_history("→ Sent", cmd.strip())
            self.ser.write(cmd.encode())
            self.ser.flush()  # Ensure the command is sent
            time.sleep(0.1)  # Wait for command to process
            
            try:
                response = self.read_serial_response()
                if response:
                    self.add_to_history("← Received", response)
                    return response
                return "No response from Arduino"
                
            except serial.SerialException as e:
                self.last_error = f"Serial communication error: {str(e)}"
                self.add_to_history("Error", self.last_error)
                # Try to reconnect on error
                if "PermissionError" not in str(e) and self.connect_serial():
                    self.add_to_history("System", "Reconnected after error")
                return None
            except Exception as e:
                self.last_error = f"Error reading response: {str(e)}"
                self.add_to_history("Error", self.last_error)
                return None
        except Exception as e:
            self.last_error = f"Command failed: {str(e)}"
            self.add_to_history("Error", self.last_error)
            return None

    def get_serial_history(self):
        return list(self.serial_history)

    def move_z(self, steps):
        """Move Z axis with error handling"""
        try:
            command = None
            if steps > 0:
                command = f"u{abs(steps)}"
            else:
                command = f"d{abs(steps)}"
            
            if command:
                response = self.send_command(command)
                if response is None:
                    return {'success': False, 'message': self.last_error}
                
                # Update Z position
                self.current_z += steps
                
                return {
                    'success': True,
                    'message': response,
                    'z': self.current_z
                }
        except Exception as e:
            self.last_error = f"Z movement failed: {str(e)}"
            return {'success': False, 'message': self.last_error}

    def move_yaw(self, steps):
        """Move Yaw with error handling"""
        try:
            command = None
            if steps > 0:
                command = f"r{abs(steps)}"
            else:
                command = f"l{abs(steps)}"
            
            if command:
                response = self.send_command(command)
                if response is None:
                    return {'success': False, 'message': self.last_error}
                
                # Update Yaw position
                self.current_yaw += steps
                
                return {
                    'success': True,
                    'message': response,
                    'yaw': self.current_yaw
                }
        except Exception as e:
            self.last_error = f"Yaw movement failed: {str(e)}"
            return {'success': False, 'message': self.last_error}

    def calculate_position(self, angle1, angle2):
        """Calculate X,Y position from angles"""
        try:
            # Convert angles to radians
            theta1 = math.radians(angle1)
            theta2 = math.radians(angle2)
            
            # Calculate end effector position
            x = self.link1 * math.cos(theta1) + self.link2 * math.cos(theta1 + theta2)
            y = self.link1 * math.sin(theta1) + self.link2 * math.sin(theta1 + theta2)
            
            return x, y
        except Exception as e:
            self.last_error = f"Position calculation failed: {str(e)}"
            return None, None

    def calculate_angles(self, x, y):
        """Calculate angles from X,Y position"""
        try:
            # Calculate distance from origin to target
            r = math.sqrt(x * x + y * y)
            
            # Check if point is reachable
            if r > (self.link1 + self.link2):
                self.last_error = "Position out of reach"
                return None, None
                
            # Calculate theta2 using cosine law
            cos_theta2 = (r * r - self.link1 * self.link1 - self.link2 * self.link2) / (2 * self.link1 * self.link2)
            if abs(cos_theta2) > 1:
                self.last_error = "Position out of reach"
                return None, None
                
            theta2 = math.acos(cos_theta2)
            
            # Calculate theta1
            k1 = self.link1 + self.link2 * math.cos(theta2)
            k2 = self.link2 * math.sin(theta2)
            theta1 = math.atan2(y, x) - math.atan2(k2, k1)
            
            # Convert to degrees
            angle1 = math.degrees(theta1)
            angle2 = math.degrees(theta2)
            
            # Constrain angles
            angle1 = max(30, min(150, angle1))
            angle2 = max(30, min(150, angle2))
            
            return angle1, angle2
        except Exception as e:
            self.last_error = f"Angle calculation failed: {str(e)}"
            return None, None

    def move_to_angles(self, angle1, angle2):
        """Move to angles with error handling"""
        try:
            cmd = f"angles {angle1} {angle2}"
            response = self.send_command(cmd)
            
            if response is None:
                return {'success': False, 'message': self.last_error}
            
            # Calculate corresponding position
            x, y = self.calculate_position(angle1, angle2)
            
            return {
                'success': True,
                'message': response,
                'angles': {'angle1': angle1, 'angle2': angle2},
                'position': {'x': round(x, 2), 'y': round(y, 2)} if x is not None else None
            }
        except Exception as e:
            self.last_error = f"Angle movement failed: {str(e)}"
            return {'success': False, 'message': self.last_error}

    def move_to_position(self, x, y):
        """Move to position with error handling"""
        try:
            # Calculate required angles
            angle1, angle2 = self.calculate_angles(x, y)
            if angle1 is None or angle2 is None:
                return {'success': False, 'message': self.last_error}
            
            cmd = f"position {x} {y}"
            response = self.send_command(cmd)
            
            if response is None:
                return {'success': False, 'message': self.last_error}
            
            # Update current positions
            self.current_x = x
            self.current_y = y
            self.theta1 = angle1
            self.theta2 = angle2
            
            return {
                'success': True,
                'message': response,
                'position': {'x': round(x, 2), 'y': round(y, 2)},
                'angles': {'angle1': round(angle1, 2), 'angle2': round(angle2, 2)}
            }
        except Exception as e:
            self.last_error = f"Position movement failed: {str(e)}"
            return {'success': False, 'message': self.last_error}

    def toggle_gripper(self, should_open):
        """Open or close the gripper"""
        try:
            if should_open:
                response = self.send_command("go")  # Open gripper command
                self.is_gripper_open = True
            else:
                response = self.send_command("gc")  # Close gripper command
                self.is_gripper_open = False
            
            if response is None:
                return {'success': False, 'message': self.last_error}
            
            return {
                'success': True,
                'message': response,
                'is_gripper_open': self.is_gripper_open
            }
        except Exception as e:
            self.last_error = f"Gripper operation failed: {str(e)}"
            return {'success': False, 'message': self.last_error}

    def rotate_gripper(self, direction, degrees):
        """Rotate the gripper left or right by specified degrees"""
        try:
            # Validate input parameters
            if direction not in ['left', 'right']:
                return {'success': False, 'message': "Invalid direction. Use 'left' or 'right'"}
            if not isinstance(degrees, (int, float)) or degrees <= 0:
                return {'success': False, 'message': "Degrees must be a positive number"}
                
            # Calculate new rotation value
            new_rotation = self.gripper_rotation
            if direction == 'left':
                new_rotation = max(0, self.gripper_rotation - degrees)
                command = f"gl{int(degrees)}"
            else:  # right
                new_rotation = min(180, self.gripper_rotation + degrees)
                command = f"gr{int(degrees)}"
            
            # Send command and get response
            response = self.send_command(command)
            if response is None:
                return {'success': False, 'message': self.last_error or "Failed to send rotation command"}
            
            # Update gripper rotation only if command was successful
            self.gripper_rotation = new_rotation
            
            # Add to history for debugging
            self.add_to_history("System", f"Gripper rotated {direction} by {degrees} degrees. New position: {new_rotation}")
            
            return {
                'success': True,
                'message': response,
                'gripper_rotation': self.gripper_rotation,
                'direction': direction,
                'degrees': degrees
            }
        except Exception as e:
            error_msg = f"Gripper rotation failed: {str(e)}"
            self.add_to_history("Error", error_msg)
            self.last_error = error_msg
            return {'success': False, 'message': error_msg}

    def toggle_manual_mode(self, enable):
        """Toggle manual control mode"""
        try:
            if enable and not self.is_manual_mode:
                try:
                    self.manual_control = ManualControl(self)
                    self.manual_control.start()
                    self.is_manual_mode = True
                    self.add_to_history("System", "Manual control started")
                    return {'success': True, 'message': 'Manual control started'}
                except Exception as e:
                    self.last_error = f"Failed to start manual control: {str(e)}"
                    self.add_to_history("Error", self.last_error)
                    return {'success': False, 'message': self.last_error}
            elif not enable and self.is_manual_mode:
                try:
                    if self.manual_control:
                        self.manual_control.stop()
                        self.manual_control = None
                    self.is_manual_mode = False
                    self.add_to_history("System", "Manual control stopped")
                    return {'success': True, 'message': 'Manual control stopped'}
                except Exception as e:
                    self.last_error = f"Failed to stop manual control: {str(e)}"
                    self.add_to_history("Error", self.last_error)
                    return {'success': False, 'message': self.last_error}
            return {'success': True, 'message': 'No change in manual control state'}
        except Exception as e:
            self.last_error = f"Failed to toggle manual mode: {str(e)}"
            self.add_to_history("Error", self.last_error)
            return {'success': False, 'message': self.last_error}

    def get_current_position(self):
        """Get current position data"""
        try:
            return {
                'success': True,
                'position': {
                    'x': self.current_x,
                    'y': self.current_y,
                    'z': self.current_z,
                    'yaw': self.current_yaw
                },
                'angles': {
                    'angle1': self.theta1,
                    'angle2': self.theta2
                },
                'gripper': {
                    'is_open': self.is_gripper_open,
                    'rotation': self.gripper_rotation
                }
            }
        except Exception as e:
            return {'success': False, 'message': str(e)}

    def reset_to_home(self):
        """Reset robot to home position"""
        try:
            self.add_to_history("System", "Initiating reset to home sequence...")
            
            # First open gripper
            self.toggle_gripper(True)
            time.sleep(0.5)
            
            # Reset gripper rotation to center
            current_rotation = self.gripper_rotation
            if current_rotation > 90:
                self.rotate_gripper('left', current_rotation - 90)
            elif current_rotation < 90:
                self.rotate_gripper('right', 90 - current_rotation)
            time.sleep(0.5)
            
            # Move Z axis to zero position
            self.send_command("z")  # Calibrate Z-axis
            time.sleep(1)
            
            # Move to home X,Y position
            result = self.move_to_position(self.current_x, self.current_y)
            if not result.get('success', False):
                raise Exception("Failed to move to home position")
            
            # Reset yaw to center
            if self.current_yaw != 0:
                steps = -self.current_yaw
                self.move_yaw(steps)
            
            # Update all positions to home values
            self.current_x = -86.87
            self.current_y = 85.47
            self.current_z = 0
            self.current_yaw = 0
            self.theta1 = 90
            self.theta2 = 90
            self.gripper_rotation = 90
            self.is_gripper_open = True
            
            self.add_to_history("System", "Reset to home completed successfully")
            return {'success': True, 'message': 'Reset to home completed successfully'}
            
        except Exception as e:
            error_msg = f"Error during reset to home: {str(e)}"
            self.add_to_history("Error", error_msg)
            return {'success': False, 'message': error_msg}

class ManualControl:
    def __init__(self, robot):
        self.robot = robot
        self.running = False
        self.control_thread = None
        
        # Set initial X and Y positions
        self.current_x = -86.87  # Initial X position
        self.current_y = 85.47   # Initial Y position
        self.current_z = 0       # Initial Z position
        self.current_yaw = 0     # Initial Yaw position
        
        # Movement increment (in mm)
        self.move_increment = 10
        self.z_increment = 10
        self.yaw_increment = 10
        
        # Initialize Pygame
        pygame.init()
        pygame.joystick.init()
        
        # Check for controllers
        self.joystick_count = pygame.joystick.get_count()
        if self.joystick_count == 0:
            print("No joystick connected.")
            return
            
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        print(f"Joystick name: {self.joystick.get_name()}")
        
        # Threshold to avoid noise
        self.THRESHOLD = 0.5
        
        # Button mappings
        self.BUTTON_A = 0  # Z up
        self.BUTTON_B = 1  # Yaw right
        self.BUTTON_X = 2  # Z down
        self.BUTTON_Y = 3  # Yaw left
        self.BUTTON_LB = 4  # Rotate gripper left
        self.BUTTON_RB = 5  # Rotate gripper right
        self.BUTTON_BACK = 6
        self.BUTTON_START = 7
        self.BUTTON_RESTART = 8  # Restart connection
        self.BUTTON_GRIPPER = 9  # Toggle gripper
        
        # Axis mappings
        self.AXIS_LX = 0  # Left stick X
        self.AXIS_LY = 1  # Left stick Y
        self.AXIS_RX = 2  # Right stick X
        self.AXIS_RY = 3  # Right stick Y
        self.AXIS_LT = 4  # Left trigger
        self.AXIS_RT = 5  # Right trigger

    def restart_connection(self):
        """Restart serial connection"""
        try:
            # First disconnect and cleanup
            self.robot.add_to_history("System", "Initiating restart sequence...")
            
            # Ensure manual control is temporarily paused
            was_running = self.running
            self.running = False
            time.sleep(0.5)  # Give time for control loop to stop
            
            # First disconnect
            self.robot.disconnect_serial()
            time.sleep(2)  # Wait longer to ensure port is fully released
            
            # Reset all positions to default values
            self.current_x = -86.87  # Initial X position
            self.current_y = 85.47   # Initial Y position
            self.current_z = 0       # Initial Z position
            self.current_yaw = 0     # Initial Yaw position
            
            # Reset robot's positions
            self.robot.current_x = -86.87
            self.robot.current_y = 85.47
            self.robot.current_z = 0
            self.robot.current_yaw = 0
            self.robot.theta1 = 90
            self.robot.theta2 = 90
            self.robot.gripper_rotation = 90
            self.robot.is_gripper_open = True
            
            # Then reconnect
            success = self.robot.connect_serial()
            if success:
                time.sleep(1)  # Wait for connection to stabilize
                
                # Send reset commands
                self.robot.send_command("z")  # Calibrate Z-axis
                time.sleep(0.5)
                
                # Move to initial position
                result = self.robot.move_to_position(self.current_x, self.current_y)
                if not result.get('success', False):
                    raise Exception("Failed to move to initial position")
                    
                self.robot.add_to_history("System", "Connection restarted successfully and positions reset")
                
                # Resume manual control if it was running
                self.running = was_running
                return True
            else:
                self.robot.add_to_history("Error", "Failed to restart connection")
                return False
                
        except Exception as e:
            error_msg = f"Error during restart: {str(e)}"
            self.robot.add_to_history("Error", error_msg)
            traceback.print_exc()  # Print full traceback for debugging
            return False
        finally:
            # Ensure control loop is restored if needed
            self.running = was_running

    def start(self):
        if self.joystick_count == 0:
            print("Cannot start: No joystick connected")
            return
            
        # Move to initial X, Y position and calibrate Z
        self.robot.move_to_position(self.current_x, self.current_y)
        self.robot.send_command("z")  # Calibrate Z-axis
        
        self.running = True
        self.control_thread = threading.Thread(target=self._control_loop)
        self.control_thread.start()
        print("Manual control started")

    def stop(self):
        self.running = False
        if self.control_thread:
            self.control_thread.join()
        pygame.quit()
        print("Manual control stopped")

    def _control_loop(self):
        last_print_time = time.time()
        print_interval = 0.5  # Print values every 0.5 seconds
        last_move_time = time.time()
        move_interval = 0.2  # Minimum time between moves (seconds)
        
        while self.running:
            current_time = time.time()
            
            # Get current analog values
            lx = self.joystick.get_axis(self.AXIS_LX)
            ly = self.joystick.get_axis(self.AXIS_LY)
            
            # Print values periodically
            if current_time - last_print_time >= print_interval:
                print(f"Analog Values - LX: {lx:.2f}, LY: {ly:.2f}")
                print(f"Current Position - X: {self.current_x:.2f}, Y: {self.current_y:.2f}")
                print(f"Current Z: {self.current_z}, Current Yaw: {self.current_yaw}")
                last_print_time = current_time
            
            # Check if enough time has passed since last move
            if current_time - last_move_time >= move_interval:
                moved = False
                
                # Handle X-axis movement
                if abs(lx) > self.THRESHOLD:
                    x_increment = self.move_increment if lx > 0 else -self.move_increment
                    self.current_x += x_increment
                    moved = True
                
                # Handle Y-axis movement
                if abs(ly) > self.THRESHOLD:
                    y_increment = self.move_increment if ly > 0 else -self.move_increment
                    self.current_y += y_increment
                    moved = True
                
                # If either axis moved, update position
                if moved:
                    self.robot.move_to_position(self.current_x, self.current_y)
                    self.robot.send_command(f"position {self.current_x} {self.current_y}")
                    last_move_time = current_time
            
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
                    break
                    
                # Handle button presses
                if event.type == pygame.JOYBUTTONDOWN:
                    if event.button == self.BUTTON_A:  # Z up
                        self.current_z += 100  # Changed to 100 steps
                        self.robot.move_z(100)  # Changed to 100 steps
                        self.robot.send_command(f"u{10}")  # Keep small increments for smoother movement
                    elif event.button == self.BUTTON_X:  # Z down
                        self.current_z -= 100  # Changed to 100 steps
                        self.robot.move_z(-100)  # Changed to 100 steps
                        self.robot.send_command(f"d{10}")  # Keep small increments for smoother movement
                    elif event.button == self.BUTTON_B:  # Yaw right
                        self.current_yaw += self.yaw_increment
                        self.robot.move_yaw(self.yaw_increment)
                        self.robot.send_command(f"r{self.yaw_increment}")
                    elif event.button == self.BUTTON_Y:  # Yaw left
                        self.current_yaw -= self.yaw_increment
                        self.robot.move_yaw(-self.yaw_increment)
                        self.robot.send_command(f"l{self.yaw_increment}")
                    elif event.button == self.BUTTON_LB:  # Rotate gripper left
                        print("Rotating gripper left")
                        result = self.robot.rotate_gripper('left', 30)
                        if not result['success']:
                            print(f"Failed to rotate gripper left: {result['message']}")
                            self.robot.add_to_history("Error", f"Gripper rotation left failed: {result['message']}")
                        else:
                            print(f"Gripper rotated left to {result['gripper_rotation']} degrees")
                    elif event.button == self.BUTTON_RB:  # Rotate gripper right
                        print("Rotating gripper right")
                        result = self.robot.rotate_gripper('right', 30)
                        if not result['success']:
                            print(f"Failed to rotate gripper right: {result['message']}")
                            self.robot.add_to_history("Error", f"Gripper rotation right failed: {result['message']}")
                        else:
                            print(f"Gripper rotated right to {result['gripper_rotation']} degrees")
                    elif event.button == self.BUTTON_BACK:
                        self.running = False  # Stop manual control
                        break
                    elif event.button == self.BUTTON_RESTART:  # Button 8 for restart
                        print("Restart button pressed - initiating restart sequence")
                        if self.restart_connection():
                            print("Restart successful")
                            self.robot.add_to_history("System", "Restart completed successfully")
                        else:
                            print("Restart failed")
                            self.robot.add_to_history("Error", "Restart failed - please check connection")
                    elif event.button == self.BUTTON_GRIPPER:  # Button 9 for gripper
                        self.robot.toggle_gripper(not self.robot.is_gripper_open)
                        self.robot.send_command("g")  # Send gripper toggle command
            
            time.sleep(0.1)  # Small delay to prevent overwhelming the system

# Create robot instance
robot = ScaraRobot()

@app.route('/')
def home():
    ip_address = get_local_ip()
    return render_template('index.html', ip_address=ip_address)

@app.route('/disconnect_serial', methods=['POST'])
def disconnect_serial():
    success = robot.disconnect_serial()
    settings = robot.get_current_settings()
    return jsonify({
        'success': success,
        'status': robot.status,
        'settings': settings
    })

@app.route('/get_serial_settings')
def get_serial_settings():
    return jsonify(robot.get_current_settings())

@app.route('/update_serial_settings', methods=['POST'])
def update_serial_settings():
    data = request.get_json()
    success = robot.connect_serial(data.get('port'), data.get('baudrate'))
    settings = robot.get_current_settings()
    return jsonify({
        'success': success,
        'status': robot.status,
        'settings': settings
    })

@app.route('/get_serial_history')
def get_serial_history():
    return jsonify(robot.get_serial_history())

@app.route('/get_current_position')
def get_current_position():
    try:
        result = robot.get_current_position()
        return jsonify(result)
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)}), 400

@app.route('/move_z', methods=['POST'])
def move_z():
    try:
        data = request.get_json()
        steps = int(data['steps'])
        result = robot.move_z(steps)
        if result['success']:
            return jsonify(result)
        else:
            return jsonify(result), 400
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)}), 400

@app.route('/move_yaw', methods=['POST'])
def move_yaw():
    try:
        data = request.get_json()
        steps = int(data['steps'])
        result = robot.move_yaw(steps)
        if result['success']:
            return jsonify(result)
        else:
            return jsonify(result), 400
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)}), 400

@app.route('/update_position', methods=['POST'])
def update_position():
    try:
        data = request.get_json()
        x = float(data['x'])
        y = float(data['y'])
        result = robot.move_to_position(x, y)
        return jsonify(result)
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)}), 400

@app.route('/update_angles', methods=['POST'])
def update_angles():
    try:
        data = request.get_json()
        angle1 = float(data['angle1'])
        angle2 = float(data['angle2'])
        result = robot.move_to_angles(angle1, angle2)
        return jsonify(result)
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)}), 400

@app.route('/toggle_gripper', methods=['POST'])
def toggle_gripper():
    try:
        data = request.get_json()
        result = robot.toggle_gripper(data['is_open'])
        if result['success']:
            return jsonify(result)
        else:
            return jsonify(result), 400
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)}), 400

@app.route('/rotate_gripper', methods=['POST'])
def rotate_gripper():
    try:
        data = request.get_json()
        direction = data['direction']  # 'left' or 'right'
        degrees = int(data['degrees'])
        result = robot.rotate_gripper(direction, degrees)
        return jsonify(result)
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)}), 400

@app.route('/toggle_manual_mode', methods=['POST'])
def toggle_manual_mode():
    data = request.get_json()
    result = robot.toggle_manual_mode(data.get('enable', False))
    return jsonify(result)

@app.route('/reset_to_home', methods=['POST'])
def reset_to_home():
    try:
        result = robot.reset_to_home()
        return jsonify(result)
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)}), 400

if __name__ == '__main__':
    local_ip = get_local_ip()
    url = f"http://{local_ip}"
    print(f"Server running at: {url}")
    
    # Open Chromium browser after a short delay to ensure server is running
    def open_browser():
        time.sleep(1.5)  # Wait for server to start
        try:
            # Try to open with Chromium (Raspberry Pi default)
            webbrowser.get('chromium-browser').open(url)
        except webbrowser.Error:
            # Fall back to default browser if Chromium is not available
            print("Chromium not found, trying default browser")
            webbrowser.open(url)
    
    # Start browser in a separate thread
    threading.Thread(target=open_browser).start()
    
    # Run the Flask server
    app.run(host='0.0.0.0', port=5000, debug=True)  # Allow external access

# how to run the code
# python robot_control.py