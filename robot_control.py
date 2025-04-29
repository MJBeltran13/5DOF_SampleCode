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

app = Flask(__name__)

class ScaraRobot:
    def __init__(self):
        # Default arm configurations in mm
        self.link1 = 85.47  # Link 1 length (L1)
        self.link2 = 86.87  # Link 2 length (L2)
        
        # Current positions
        self.theta1 = 0  # Base rotation
        self.theta2 = 0  # First arm joint
        self.theta3 = 0  # Second arm joint
        self.z_pos = 0   # Z position
        self.yaw_pos = 0 # Yaw position
        
        # Gripper state
        self.is_gripper_open = True
        
        # Manual control state
        self.manual_control = None
        self.is_manual_mode = False
        
        # Serial communication history
        self.serial_history = deque(maxlen=100)  # Keep last 100 messages
        
        # Serial connection settings
        self.com_port = 'COM3'
        self.baudrate = 9600
        self.ser = None
        self.connect_serial()
        self.last_error = None

    def disconnect_serial(self):
        """Disconnect from serial port"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.ser = None
            self.status = f"Disconnected from {self.com_port}"
            self.add_to_history("System", self.status)
            return True
        return False

    def read_serial_response(self):
        """Read and decode serial response with multiple encoding attempts"""
        try:
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
        if port:
            self.com_port = port
        if baud:
            self.baudrate = baud

        # Close existing connection if any
        self.disconnect_serial()

        try:
            # Import serial here to ensure it's available
            import serial
            self.ser = serial.Serial(
                port=self.com_port,
                baudrate=self.baudrate,
                timeout=1,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            
            # Wait for Arduino to reset and send startup messages
            time.sleep(2)
            
            # Read any available startup messages
            start_time = time.time()
            while time.time() - start_time < 3:  # Wait up to 3 seconds for startup messages
                if self.ser.in_waiting:
                    response = self.read_serial_response()
                    if response:
                        self.add_to_history("← Startup", response)
                time.sleep(0.1)
                    
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
        for port in serial.tools.list_ports.comports():
            ports.append({
                'port': port.device,
                'description': port.description,
                'hwid': port.hwid
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
                return None

            self.add_to_history("→ Sent", cmd)
            self.ser.write(f"{cmd}\n".encode())
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
            if steps > 0:
                response = self.send_command(f"u{abs(steps)}")
            else:
                response = self.send_command(f"d{abs(steps)}")
            
            if response is None:
                return {'success': False, 'message': self.last_error}
            return {'success': True, 'message': response}
        except Exception as e:
            self.last_error = f"Z movement failed: {str(e)}"
            return {'success': False, 'message': self.last_error}

    def move_yaw(self, steps):
        """Move Yaw with error handling"""
        try:
            if steps > 0:
                response = self.send_command(f"r{abs(steps)}")
            else:
                response = self.send_command(f"l{abs(steps)}")
            
            if response is None:
                return {'success': False, 'message': self.last_error}
            return {'success': True, 'message': response}
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
        self.is_gripper_open = should_open
        # Add gripper control command here when implemented in Arduino
        return True

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

class ManualControl:
    def __init__(self, robot):
        self.robot = robot
        self.running = False
        self.control_thread = None
        
        # Set initial X and Y positions
        self.current_x = -86.87
        self.current_y = 85.47
        
        # Initialize Z and preserve current yaw
        self.current_z = 0
        self.current_yaw = robot.yaw_pos  # Preserve current yaw position
        
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
        self.BUTTON_LB = 4
        self.BUTTON_RB = 5  # Z calibration
        self.BUTTON_BACK = 6
        self.BUTTON_START = 7
        
        # Axis mappings
        self.AXIS_LX = 0  # Left stick X
        self.AXIS_LY = 1  # Left stick Y
        self.AXIS_RX = 2  # Right stick X
        self.AXIS_RY = 3  # Right stick Y
        self.AXIS_LT = 4  # Left trigger
        self.AXIS_RT = 5  # Right trigger

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
                        self.current_z += self.z_increment
                        self.robot.move_z(self.z_increment)
                        self.robot.send_command(f"u{self.z_increment}")
                    elif event.button == self.BUTTON_X:  # Z down
                        self.current_z -= self.z_increment
                        self.robot.move_z(-self.z_increment)
                        self.robot.send_command(f"d{self.z_increment}")
                    elif event.button == self.BUTTON_B:  # Yaw right
                        self.current_yaw += self.yaw_increment
                        self.robot.move_yaw(self.yaw_increment)
                        self.robot.send_command(f"r{self.yaw_increment}")
                    elif event.button == self.BUTTON_Y:  # Yaw left
                        self.current_yaw -= self.yaw_increment
                        self.robot.move_yaw(-self.yaw_increment)
                        self.robot.send_command(f"l{self.yaw_increment}")
                    elif event.button == self.BUTTON_RB:  # Z calibration
                        self.current_z = 0  # Reset Z position
                        self.robot.send_command("z")  # Send Z calibration command
                    elif event.button == self.BUTTON_BACK:
                        self.running = False  # Stop manual control
                        break
            
            time.sleep(0.1)  # Small delay to prevent overwhelming the system

# Create robot instance
robot = ScaraRobot()

@app.route('/')
def home():
    return render_template('index.html')

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

@app.route('/move_z', methods=['POST'])
def move_z():
    try:
        data = request.get_json()
        steps = int(data['steps'])
        result = robot.move_z(steps)
        return jsonify(result)
    except Exception as e:
        return jsonify({'success': False, 'message': str(e)}), 400

@app.route('/move_yaw', methods=['POST'])
def move_yaw():
    try:
        data = request.get_json()
        steps = int(data['steps'])
        result = robot.move_yaw(steps)
        return jsonify(result)
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
    data = request.get_json()
    success = robot.toggle_gripper(data['is_open'])
    return jsonify({
        'status': 'Gripper toggled',
        'is_gripper_open': robot.is_gripper_open
    })

@app.route('/toggle_manual_mode', methods=['POST'])
def toggle_manual_mode():
    data = request.get_json()
    result = robot.toggle_manual_mode(data.get('enable', False))
    return jsonify(result)

if __name__ == '__main__':
    app.run(debug=True) 

# how to run the code
# python robot_control.py