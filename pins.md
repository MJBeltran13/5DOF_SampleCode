# 5DOF SCARA Robot Pin Configuration

## Servo Motors
- **Elbow Servo (Servo 1)**
  - Pin: 33
  - Function: Controls the elbow joint
  - Angle Range: 30° to 150°

- **Base Servo (Servo 2)**
  - Pin: 25
  - Function: Controls the base rotation
  - Angle Range: 30° to 150°

- **Gripper Servo (Servo 3)**
  - Pin: 26
  - Function: Controls the gripper (open/close)
  - Open Position: 0°
  - Closed Position: 90°
  - Commands: 
    - `go` - Opens the gripper
    - `gc` - Closes the gripper

- **Gripper Rotation Servo (Servo 4)**
  - Pin: 27
  - Function: Controls the rotation of the gripper
  - Angle Range: 0° to 180°
  - Commands:
    - `gl10` - Rotates left 10°
    - `gr10` - Rotates right 10°

## Stepper Motors

### Z-Axis Lift Stepper
- **Direction Pin (DIR_PIN_Z)**
  - Pin: 13
  - Function: Controls the direction of Z-axis movement

- **Step Pin (STEP_PIN_Z)**
  - Pin: 2
  - Function: Controls the step pulses for Z-axis movement

- **Limit Switch Pin (LIMIT_SWITCH_PIN_Z)**
  - Pin: 12
  - Function: Detects the lower limit of Z-axis movement

### Yaw Stepper
- **Direction Pin (DIR_PIN_YAW)**
  - Pin: 19
  - Function: Controls the direction of yaw rotation

- **Step Pin (STEP_PIN_YAW)**
  - Pin: 18
  - Function: Controls the step pulses for yaw rotation

## Stepper Motor Settings
- Steps per Revolution: 200
- Microsteps: 16
- Total Steps: 3200 (200 * 16)
- Z-Axis Step Delay: 1000 microseconds
- Yaw Step Delay: 500000 microseconds

## Z-Axis Position Limits
- Minimum Position: -1900 steps
- Maximum Position: 0 steps

## Serial Communication
- Baud Rate: 9600
- Data Bits: 8
- Parity: None
- Stop Bits: 1 