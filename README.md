# 5DOF SCARA Robot Control Interface

This is a web-based control interface for a 5DOF SCARA robot. It provides position control and arm configuration capabilities through an intuitive user interface.

## Features

- Position control (X, Y, Z coordinates)
- Real-time joint angle calculations
- Arm configuration settings
- Visual feedback for operation status
- Modern web interface

## Setup

1. Install Python 3.7 or higher
2. Install the required dependencies:
   ```bash
   pip install -r requirements.txt
   ```

## Running the Application

1. Start the server:
   ```bash
   python robot_control.py
   ```
2. Open a web browser and navigate to:
   ```
   http://localhost:5000
   ```

## Usage

### Position Control

- Enter the desired X, Y, Z coordinates in centimeters
- Click "Update Position" to calculate joint angles
- The status and joint angles will be displayed below

### Arm Configuration

- Modify the arm link lengths in centimeters
- Click "Update Configuration" to apply changes
- The status will be displayed below

## Technical Details

The application uses inverse kinematics to calculate the joint angles required to reach the desired position. The calculations take into account:

- Base rotation (θ1)
- First arm joint angle (θ2)
- Second arm joint angle (θ3)
- End effector rotation (θ4)
- Vertical position (D)

## Error Handling

The application includes error handling for:

- Out-of-reach positions
- Invalid configurations
- Input validation

## Note

This is a simulation interface. To connect to actual robot hardware, additional implementation of motor control would be required.
