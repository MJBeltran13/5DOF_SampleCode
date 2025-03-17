import numpy as np
from flask import Flask, render_template, request, jsonify
import math

app = Flask(__name__)

class ScaraRobot:
    def __init__(self):
        # Default arm configurations in cm
        self.link1 = 20  # Base to first joint
        self.link2 = 15  # First joint to second joint
        self.link3 = 15  # Second joint to end effector
        self.link4 = 10  # Vertical link
        
        # Current joint angles
        self.theta1 = 0  # Base rotation
        self.theta2 = 0  # First arm joint
        self.theta3 = 0  # Second arm joint
        self.theta4 = 0  # End effector rotation
        self.d = 0      # Vertical position
        
        # Gripper state
        self.gripper_angle = 0  # Gripper rotation angle
        self.is_gripper_open = True  # Gripper state (True = open, False = closed)
        
        self.status = "Ready"

    def update_configuration(self, link1, link2, link3, link4):
        self.link1 = float(link1)
        self.link2 = float(link2)
        self.link3 = float(link3)
        self.link4 = float(link4)
        return True

    def update_gripper_rotation(self, angle):
        try:
            self.gripper_angle = float(angle)
            self.status = "Gripper rotation updated"
            return True
        except Exception as e:
            self.status = f"Error updating gripper rotation: {str(e)}"
            return False

    def toggle_gripper(self, should_open):
        try:
            self.is_gripper_open = should_open
            self.status = f"Gripper {'opened' if should_open else 'closed'}"
            return True
        except Exception as e:
            self.status = f"Error controlling gripper: {str(e)}"
            return False

    def update_angle(self, angle_id, value):
        try:
            value = float(value)
            if angle_id == 'theta1':
                self.theta1 = math.radians(value)
            elif angle_id == 'theta2':
                self.theta2 = math.radians(value)
            elif angle_id == 'theta3':
                self.theta3 = math.radians(value)
            elif angle_id == 'theta4':
                self.theta4 = math.radians(value)
            elif angle_id == 'd':
                self.d = value
            
            self.status = f"Angle {angle_id} updated successfully"
            return True
        except Exception as e:
            self.status = f"Error updating angle {angle_id}: {str(e)}"
            return False

    def inverse_kinematics(self, x, y, z):
        try:
            # Calculate base rotation (theta1)
            self.theta1 = math.atan2(y, x)
            
            # Transform to polar coordinates
            r = math.sqrt(x**2 + y**2)
            
            # Calculate theta2 and theta3 using cosine law
            cos_theta3 = (r**2 - self.link2**2 - self.link3**2) / (2 * self.link2 * self.link3)
            
            if abs(cos_theta3) > 1:
                raise ValueError("Position out of reach")
                
            self.theta3 = math.acos(cos_theta3)
            
            # Calculate theta2
            beta = math.atan2(self.link3 * math.sin(self.theta3),
                            self.link2 + self.link3 * math.cos(self.theta3))
            alpha = math.atan2(z, r)
            self.theta2 = alpha - beta
            
            # Vertical position
            self.d = z
            
            # Convert angles to degrees
            angles = {
                'theta1': math.degrees(self.theta1),
                'theta2': math.degrees(self.theta2),
                'theta3': math.degrees(self.theta3),
                'theta4': math.degrees(self.theta4),
                'd': self.d,
                'gripper_angle': self.gripper_angle,
                'is_gripper_open': self.is_gripper_open
            }
            
            self.status = "Success: Position calculated"
            return angles
            
        except Exception as e:
            self.status = f"Error: {str(e)}"
            return None

# Create robot instance
robot = ScaraRobot()

@app.route('/')
def home():
    return render_template('index.html')

@app.route('/update_position', methods=['POST'])
def update_position():
    data = request.get_json()
    x = float(data['x'])
    y = float(data['y'])
    z = float(data['z'])
    
    angles = robot.inverse_kinematics(x, y, z)
    if angles:
        return jsonify({
            'status': robot.status,
            'angles': angles
        })
    return jsonify({'status': robot.status}), 400

@app.route('/update_configuration', methods=['POST'])
def update_configuration():
    data = request.get_json()
    success = robot.update_configuration(
        data['link1'],
        data['link2'],
        data['link3'],
        data['link4']
    )
    return jsonify({'status': 'Configuration updated' if success else 'Update failed'})

@app.route('/update_gripper_rotation', methods=['POST'])
def update_gripper_rotation():
    data = request.get_json()
    success = robot.update_gripper_rotation(data['angle'])
    return jsonify({
        'status': robot.status,
        'gripper_angle': robot.gripper_angle
    })

@app.route('/toggle_gripper', methods=['POST'])
def toggle_gripper():
    data = request.get_json()
    success = robot.toggle_gripper(data['is_open'])
    return jsonify({
        'status': robot.status,
        'is_gripper_open': robot.is_gripper_open
    })

@app.route('/update_angle', methods=['POST'])
def update_angle():
    data = request.get_json()
    success = robot.update_angle(data['angle_id'], data['value'])
    return jsonify({
        'status': robot.status,
        'angles': {
            'theta1': math.degrees(robot.theta1),
            'theta2': math.degrees(robot.theta2),
            'theta3': math.degrees(robot.theta3),
            'theta4': math.degrees(robot.theta4),
            'd': robot.d,
            'gripper_angle': robot.gripper_angle,
            'is_gripper_open': robot.is_gripper_open
        }
    })

if __name__ == '__main__':
    app.run(debug=True) 