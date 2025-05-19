#include <ESP32Servo.h>
#include <math.h> // for sin(), cos()
#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"

// ESP32 FreeRTOS - these are automatically included with ESP32 core
// No need to explicitly include FreeRTOS headers as they are part of the ESP32 core

// RTOS settings
#define STACK_SIZE 4096         // Stack size for tasks
#define SERVO_TASK_PRIORITY 3   // Highest priority for servo control
#define STEPPER_TASK_PRIORITY 2 // Medium priority for stepper motors
#define MONITOR_TASK_PRIORITY 1 // Lower priority for position monitoring
#define SERIAL_TASK_PRIORITY 1  // Lower priority for serial communication

// Servo objects
Servo myservo1; // Elbow
Servo myservo2; // Base
Servo myservo3; // Gripper
Servo myservo4; // Gripper Rotation

// Task handles
TaskHandle_t serialTaskHandle = NULL;
TaskHandle_t servoTaskHandle = NULL;
TaskHandle_t zAxisTaskHandle = NULL;
TaskHandle_t yawTaskHandle = NULL;
TaskHandle_t monitorTaskHandle = NULL;

// Queue handles
QueueHandle_t commandQueue;
QueueHandle_t zCommandQueue;
QueueHandle_t positionQueue;

// Mutex for protecting shared resources
SemaphoreHandle_t positionMutex;
SemaphoreHandle_t serialMutex;

// Command structure
struct Command
{
  char type;
  float value1;
  float value2;
};

// Position structure
struct Position
{
  float x;
  float y;
  int z;
  int yaw;
  float angle1;
  float angle2;
  bool isGripperOpen;
  int gripperRotation;
};

// Global position state protected by mutex
Position currentPosition = {
    -86.87, // x
    85.47,  // y
    0,      // z
    0,      // yaw
    90,     // angle1
    90,     // angle2
    true,   // isGripperOpen
    90      // gripperRotation
};

const int servoPin1 = 33;
const int servoPin2 = 25;
const int servoPin3 = 26;
const int servoPin4 = 27; // Gripper Rotation Servo

// Link lengths in mm
const float L1 = 85.47;
const float L2 = 86.87;

// Servo angle limits
const int BASE_MIN_ANGLE = 30;
const int BASE_MAX_ANGLE = 150;
const int ELBOW_MIN_ANGLE = 30;
const int ELBOW_MAX_ANGLE = 150;
const int ROTATION_MIN_ANGLE = 0;   // Gripper rotation min angle
const int ROTATION_MAX_ANGLE = 180; // Gripper rotation max angle

// Gripper positions
const int GRIPPER_OPEN_ANGLE = 0;    // Gripper open position
const int GRIPPER_CLOSED_ANGLE = 90; // Gripper closed position

// Stepper motor (Z LIFT) pin definitions
const int DIR_PIN_Z = 13;
const int STEP_PIN_Z = 2;
const int LIMIT_SWITCH_PIN_Z = 12;

// Stepper motor (Z YAW) pin definitions
const int DIR_PIN_YAW = 19;
const int STEP_PIN_YAW = 18;

// Stepper motor settings
const int stepsPerRevolution = 200;
const int microsteps = 16;
const int totalSteps = stepsPerRevolution * microsteps;
const unsigned long MIN_STEP_DELAY = 500;      // Minimum delay between steps (max speed)
const unsigned long MAX_STEP_DELAY = 2000;     // Starting delay for acceleration
const unsigned long ACCEL_STEPS = 50;          // Number of steps for acceleration/deceleration
const unsigned long YAW_MIN_DELAY = 250000;    // Minimum delay for yaw movement
const unsigned long YAW_MAX_DELAY = 500000;    // Maximum delay for yaw movement

// Variables for Z control
int currentPositionZ = 0;
int currentPositionYaw = 0; // New for YAW motor

// Z-axis position limits
const int Z_MIN_POSITION = -1900; // Lower limit
const int Z_MAX_POSITION = 0;     // Upper limit

// Task function declarations
void serialTask(void *parameter);
void servoTask(void *parameter);
void zAxisTask(void *parameter);
void yawTask(void *parameter);
void monitorTask(void *parameter);

// Function to calculate step delay during acceleration/deceleration
unsigned long calculateStepDelay(int currentStep, int totalSteps) {
  // First phase: acceleration
  if (currentStep < ACCEL_STEPS) {
    return MAX_STEP_DELAY - (currentStep * (MAX_STEP_DELAY - MIN_STEP_DELAY) / ACCEL_STEPS);
  }
  // Last phase: deceleration
  else if (currentStep > totalSteps - ACCEL_STEPS) {
    int stepsFromEnd = totalSteps - currentStep;
    return MAX_STEP_DELAY - (stepsFromEnd * (MAX_STEP_DELAY - MIN_STEP_DELAY) / ACCEL_STEPS);
  }
  // Middle phase: constant speed
  else {
    return MIN_STEP_DELAY;
  }
}

void setup()
{
  Serial.begin(115200);  // Changed to match Python script's baud rate

  // Create mutex and queues
  positionMutex = xSemaphoreCreateMutex();
  serialMutex = xSemaphoreCreateMutex();
  commandQueue = xQueueCreate(10, sizeof(Command));
  zCommandQueue = xQueueCreate(5, sizeof(Command));
  positionQueue = xQueueCreate(5, sizeof(Position));

  // Setup for servos
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  myservo1.setPeriodHertz(50);
  myservo1.attach(servoPin1, 500, 2400);
  myservo1.write(90);

  myservo2.setPeriodHertz(50);
  myservo2.attach(servoPin2, 500, 2400);
  myservo2.write(90);

  myservo3.setPeriodHertz(50);
  myservo3.attach(servoPin3, 500, 2400);
  myservo3.write(90);

  myservo4.setPeriodHertz(50);
  myservo4.attach(servoPin4, 500, 2400);
  myservo4.write(90);

  Serial.println("Servos initialized.");

  // Setup for Z lift stepper motor
  pinMode(DIR_PIN_Z, OUTPUT);
  pinMode(STEP_PIN_Z, OUTPUT);
  pinMode(LIMIT_SWITCH_PIN_Z, INPUT_PULLUP);

  // Setup for Z yaw stepper motor
  pinMode(DIR_PIN_YAW, OUTPUT);
  pinMode(STEP_PIN_YAW, OUTPUT);

  Serial.println("Starting Z-axis calibration...");
  calibrateToZero();

  // Create tasks with proper stack size and priority
  xTaskCreate(
      serialTask,
      "SerialTask",
      STACK_SIZE,
      NULL,
      SERIAL_TASK_PRIORITY,
      &serialTaskHandle);

  xTaskCreate(
      servoTask,
      "ServoTask",
      STACK_SIZE,
      NULL,
      SERVO_TASK_PRIORITY,
      &servoTaskHandle);

  xTaskCreate(
      zAxisTask,
      "ZAxisTask",
      STACK_SIZE,
      NULL,
      STEPPER_TASK_PRIORITY,
      &zAxisTaskHandle);

  xTaskCreate(
      yawTask,
      "YawTask",
      STACK_SIZE,
      NULL,
      STEPPER_TASK_PRIORITY,
      &yawTaskHandle);

  xTaskCreate(
      monitorTask,
      "MonitorTask",
      STACK_SIZE,
      NULL,
      MONITOR_TASK_PRIORITY,
      &monitorTaskHandle);

  Serial.println("System ready.");
  printCommands();
}

void loop()
{
  // Empty loop - all work is done in tasks
  vTaskDelay(pdMS_TO_TICKS(1000));
}

// Print available commands
void printCommands()
{
  if (xSemaphoreTake(serialMutex, portMAX_DELAY))
  {
    Serial.println("Commands:");
    Serial.println("angles A1 A2    - move servos to angles");
    Serial.println("position X Y    - move servos to reach XY");
    Serial.println("u100            - move Z lift up 100 steps");
    Serial.println("d100            - move Z lift down 100 steps");
    Serial.println("l100            - rotate yaw left 100 steps");
    Serial.println("r100            - rotate yaw right 100 steps");
    Serial.println("gl10            - rotate gripper left 10 degrees");
    Serial.println("gr10            - rotate gripper right 10 degrees");
    Serial.println("go              - open gripper");
    Serial.println("gc              - close gripper");
    Serial.println("p               - print positions");
    xSemaphoreGive(serialMutex);
  }
}

// Serial communication task
void serialTask(void *parameter)
{
  String inputString = "";

  while (true)
  {
    if (Serial.available() > 0)
    {
      char inChar = (char)Serial.read();
      if (inChar == '\n')
      {
        Command cmd;
        parseCommand(inputString, &cmd);

        // Debug print before sending to queue
        if (xSemaphoreTake(serialMutex, portMAX_DELAY))
        {
          Serial.print("Sending command to queue - Type: ");
          Serial.print(cmd.type);
          Serial.print(", Value: ");
          Serial.println(cmd.value1);
          xSemaphoreGive(serialMutex);
        }

        // Route commands to appropriate queues
        if (cmd.type == 'Z')
        {
          xQueueSend(zCommandQueue, &cmd, portMAX_DELAY);
          if (xSemaphoreTake(serialMutex, portMAX_DELAY))
          {
            Serial.println("Z command sent to Z queue");
            xSemaphoreGive(serialMutex);
          }
        }
        else
        {
          // All other commands including yaw ('Y') go to the main command queue
          xQueueSend(commandQueue, &cmd, portMAX_DELAY);
          if (xSemaphoreTake(serialMutex, portMAX_DELAY))
          {
            if (cmd.type == 'Y')
            {
              Serial.println("Yaw command sent to command queue");
            }
            xSemaphoreGive(serialMutex);
          }
        }

        inputString = "";
      }
      else
      {
        inputString += inChar;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// Parse incoming command
void parseCommand(String input, Command *cmd)
{
  input.trim();

  // Handle IP address command from Python script
  if (input.startsWith("IP:"))
  {
    if (xSemaphoreTake(serialMutex, portMAX_DELAY))
    {
      Serial.println("OK"); // Send acknowledgment
      xSemaphoreGive(serialMutex);
    }
    return;
  }

  if (input == "reset" || input == "r" || input == "z")  // Allow "reset", "r", or "z" for reset
  {
    cmd->type = 'R';
    if (xSemaphoreTake(serialMutex, portMAX_DELAY))
    {
      Serial.println("Reset command received");
      xSemaphoreGive(serialMutex);
    }
    return;
  }
  else if (input == "go")
  {
    cmd->type = 'G';
    cmd->value1 = 1; // open
  }
  else if (input == "gc")
  {
    cmd->type = 'G';
    cmd->value1 = 0; // close
  }
  else if (input.startsWith("u"))
  {
    cmd->type = 'Z';
    cmd->value1 = input.substring(1).toFloat();
    if (xSemaphoreTake(serialMutex, portMAX_DELAY))
    {
      Serial.print("Up command received: ");
      Serial.println(cmd->value1);
      xSemaphoreGive(serialMutex);
    }
  }
  else if (input.startsWith("d"))
  {
    cmd->type = 'Z';
    cmd->value1 = -input.substring(1).toFloat();
    if (xSemaphoreTake(serialMutex, portMAX_DELAY))
    {
      Serial.print("Down command received: ");
      Serial.println(cmd->value1);
      xSemaphoreGive(serialMutex);
    }
  }
  else if (input.startsWith("l"))
  {
    cmd->type = 'Y';
    cmd->value1 = -input.substring(1).toFloat();
    if (xSemaphoreTake(serialMutex, portMAX_DELAY))
    {
      Serial.print("Left yaw command received: ");
      Serial.println(cmd->value1);
      xSemaphoreGive(serialMutex);
    }
  }
  else if (input.startsWith("r"))
  {
    cmd->type = 'Y';
    cmd->value1 = input.substring(1).toFloat();
    if (xSemaphoreTake(serialMutex, portMAX_DELAY))
    {
      Serial.print("Right yaw command received: ");
      Serial.println(cmd->value1);
      xSemaphoreGive(serialMutex);
    }
  }
  else if (input.startsWith("gl"))
  {
    cmd->type = 'T';
    cmd->value1 = -input.substring(2).toFloat();
  }
  else if (input.startsWith("gr"))
  {
    cmd->type = 'T';
    cmd->value1 = input.substring(2).toFloat();
  }
  else if (input == "p")
  {
    cmd->type = 'P';
  }
  else
  {
    int spaceIndex = input.indexOf(' ');
    if (spaceIndex != -1)
    {
      String mode = input.substring(0, spaceIndex);
      String data = input.substring(spaceIndex + 1);

      if (mode == "angles")
      {
        cmd->type = 'A';
        cmd->value1 = data.substring(0, data.indexOf(' ')).toFloat();
        cmd->value2 = data.substring(data.indexOf(' ') + 1).toFloat();
      }
      else if (mode == "position")
      {
        cmd->type = 'X';
        cmd->value1 = data.substring(0, data.indexOf(' ')).toFloat();
        cmd->value2 = data.substring(data.indexOf(' ') + 1).toFloat();
      }
    }
  }
}

// Servo control task
void servoTask(void *parameter)
{
  Command cmd;

  while (true)
  {
    if (xQueueReceive(commandQueue, &cmd, portMAX_DELAY))
    {
      switch (cmd.type)
      {
      case 'R': // Reset
        resetAll();
        break;
      case 'A': // Angles
        moveToAngles(cmd.value1, cmd.value2);
        break;
      case 'X': // XY Position
        moveToPosition(cmd.value1, cmd.value2);
        break;
      case 'G': // Gripper
        if (cmd.value1 == 1)
        {
          openGripper();
        }
        else
        {
          closeGripper();
        }
        break;
      case 'T': // Gripper rotation
        rotateGripper(cmd.value1);
        break;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// Z-axis control task
void zAxisTask(void *parameter)
{
  Command cmd;

  while (true)
  {
    if (xQueueReceive(zCommandQueue, &cmd, portMAX_DELAY))
    {
      if (xSemaphoreTake(serialMutex, portMAX_DELAY))
      {
        Serial.print("Z task received command - Type: ");
        Serial.print(cmd.type);
        Serial.print(", Value: ");
        Serial.println(cmd.value1);
        xSemaphoreGive(serialMutex);
      }

      if (cmd.type == 'Z')
      {
        if (xSemaphoreTake(serialMutex, portMAX_DELAY))
        {
          Serial.println("Processing Z movement command");
          xSemaphoreGive(serialMutex);
        }
        moveStepsZ(cmd.value1);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// Yaw control task
void yawTask(void *parameter)
{
  Command cmd;

  while (true)
  {
    if (xQueueReceive(commandQueue, &cmd, portMAX_DELAY))
    {
      if (cmd.type == 'Y')
      {
        if (xSemaphoreTake(serialMutex, portMAX_DELAY))
        {
          Serial.print("Yaw movement: ");
          Serial.print(cmd.value1);
          Serial.println(" steps");
          xSemaphoreGive(serialMutex);
        }

        moveStepsYaw(cmd.value1);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// Position monitoring task
void monitorTask(void *parameter)
{
  while (true)
  {
    if (xSemaphoreTake(positionMutex, portMAX_DELAY))
    {
      Position pos = currentPosition;
      xSemaphoreGive(positionMutex);

      xQueueSend(positionQueue, &pos, 0);
    }
    vTaskDelay(pdMS_TO_TICKS(100)); // Update position every 100ms
  }
}

// Update current position with mutex protection
void updatePosition(Position newPos)
{
  if (xSemaphoreTake(positionMutex, portMAX_DELAY))
  {
    currentPosition = newPos;
    xSemaphoreGive(positionMutex);
  }
}

// ============== Servo Functions (X, Y Control) ==============
void calculateKinematics()
{
  float angle1 = myservo1.read();
  float angle2 = myservo2.read();
  float gripperAngle = myservo3.read();
  float gripperRotationAngle = myservo4.read();

  float theta1 = radians(angle1);
  float theta2 = radians(angle2);

  float x = L1 * cos(theta1) + L2 * cos(theta1 + theta2);
  float y = L1 * sin(theta1) + L2 * sin(theta1 + theta2);

  Serial.print("Base Angle (Servo 2): ");
  Serial.print(angle2);
  Serial.print(" degrees, ");

  Serial.print("Elbow Angle (Servo 1): ");
  Serial.print(angle1);
  Serial.print(" degrees, ");

  Serial.print("Gripper Angle (Servo 3): ");
  Serial.print(gripperAngle);
  Serial.print(" degrees, ");

  Serial.print("Gripper Rotation Angle (Servo 4): ");
  Serial.print(gripperRotationAngle);
  Serial.println(" degrees");

  Serial.print("End Effector Position -> X: ");
  Serial.print(x);
  Serial.print(" mm, Y: ");
  Serial.println(y);
}

void calculateInverseKinematics(float targetX, float targetY)
{
  float r = sqrt(targetX * targetX + targetY * targetY);

  if (r > (L1 + L2))
  {
    Serial.println("Target is out of reach!");
    return;
  }

  float cosTheta2 = (r * r - L1 * L1 - L2 * L2) / (2 * L1 * L2);
  float theta2 = acos(cosTheta2);

  float k1 = L1 + L2 * cos(theta2);
  float k2 = L2 * sin(theta2);
  float theta1 = atan2(targetY, targetX) - atan2(k2, k1);

  float angle1 = degrees(theta1);
  float angle2 = degrees(theta2);

  angle1 = constrain(angle1, ELBOW_MIN_ANGLE, ELBOW_MAX_ANGLE);
  angle2 = constrain(angle2, BASE_MIN_ANGLE, BASE_MAX_ANGLE);

  myservo1.write(angle1);
  myservo2.write(angle2);

  Serial.print("Servo 1 Angle (Elbow): ");
  Serial.print(angle1);
  Serial.print(" degrees, ");

  Serial.print("Servo 2 Angle (Base): ");
  Serial.print(angle2);
  Serial.println(" degrees");

  Serial.print("Target Position -> X: ");
  Serial.print(targetX);
  Serial.print(" mm, Y: ");
  Serial.println(targetY);
}

void moveToAngles(float angle1, float angle2)
{
  // Constrain angles to valid ranges
  angle1 = constrain(angle1, ELBOW_MIN_ANGLE, ELBOW_MAX_ANGLE);
  angle2 = constrain(angle2, BASE_MIN_ANGLE, BASE_MAX_ANGLE);

  // Get current angles
  float currentAngle1 = myservo1.read();
  float currentAngle2 = myservo2.read();

  // Calculate number of steps needed
  const int STEPS = 20;  // Number of interpolation steps
  float angle1Step = (angle1 - currentAngle1) / STEPS;
  float angle2Step = (angle2 - currentAngle2) / STEPS;

  // Gradually move to target position
  for (int i = 0; i < STEPS; i++)
  {
    float nextAngle1 = currentAngle1 + angle1Step * (i + 1);
    float nextAngle2 = currentAngle2 + angle2Step * (i + 1);
    
    myservo1.write(nextAngle1);
    myservo2.write(nextAngle2);
    
    delay(20);  // Short delay between steps for smooth movement
  }

  // Ensure final position is exact
  myservo1.write(angle1);
  myservo2.write(angle2);

  // Update position with mutex protection
  if (xSemaphoreTake(positionMutex, portMAX_DELAY))
  {
    currentPosition.angle1 = angle1;
    currentPosition.angle2 = angle2;

    // Calculate new X,Y position
    float theta1 = radians(angle1);
    float theta2 = radians(angle2);
    currentPosition.x = L1 * cos(theta1) + L2 * cos(theta1 + theta2);
    currentPosition.y = L1 * sin(theta1) + L2 * sin(theta1 + theta2);

    if (xSemaphoreTake(serialMutex, portMAX_DELAY))
    {
      Serial.print("Position updated - X: ");
      Serial.print(currentPosition.x);
      Serial.print(", Y: ");
      Serial.print(currentPosition.y);
      Serial.print(", Angle1: ");
      Serial.print(angle1);
      Serial.print(", Angle2: ");
      Serial.println(angle2);
      xSemaphoreGive(serialMutex);
    }

    xSemaphoreGive(positionMutex);
  }
}

void moveToPosition(float x, float y)
{
  float r = sqrt(x * x + y * y);

  if (r > (L1 + L2))
  {
    if (xSemaphoreTake(serialMutex, portMAX_DELAY))
    {
      Serial.println("Error: Target position out of reach");
      xSemaphoreGive(serialMutex);
    }
    return;
  }

  float cosTheta2 = (r * r - L1 * L1 - L2 * L2) / (2 * L1 * L2);
  float theta2 = acos(cosTheta2);

  float k1 = L1 + L2 * cos(theta2);
  float k2 = L2 * sin(theta2);
  float theta1 = atan2(y, x) - atan2(k2, k1);

  float angle1 = degrees(theta1);
  float angle2 = degrees(theta2);

  moveToAngles(angle1, angle2);

  if (xSemaphoreTake(serialMutex, portMAX_DELAY))
  {
    Serial.print("Moving to position - X: ");
    Serial.print(x);
    Serial.print(", Y: ");
    Serial.println(y);
    xSemaphoreGive(serialMutex);
  }
}

void moveStepsZ(int steps)
{
  if (xSemaphoreTake(serialMutex, portMAX_DELAY))
  {
    Serial.print("Z movement: ");
    Serial.print(steps);
    Serial.println(" steps");
    xSemaphoreGive(serialMutex);
  }

  // Calculate new position and check limits
  if (xSemaphoreTake(positionMutex, portMAX_DELAY))
  {
    int newPosition = currentPosition.z + steps;
    if (newPosition < Z_MIN_POSITION || newPosition > Z_MAX_POSITION)
    {
      xSemaphoreGive(positionMutex);
      if (xSemaphoreTake(serialMutex, portMAX_DELAY))
      {
        Serial.println("Error: Z movement exceeds limits");
        xSemaphoreGive(serialMutex);
      }
      return;
    }
    xSemaphoreGive(positionMutex);
  }

  digitalWrite(DIR_PIN_Z, steps > 0 ? HIGH : LOW);
  int absSteps = abs(steps);

  // Move with acceleration and deceleration
  for (int i = 0; i < absSteps; i++)
  {
    // Check limit switch when moving up
    if (steps > 0 && currentPosition.z <= 0 && digitalRead(LIMIT_SWITCH_PIN_Z) == LOW)
    {
      if (xSemaphoreTake(serialMutex, portMAX_DELAY))
      {
        Serial.println("Z movement stopped: Limit switch triggered");
        xSemaphoreGive(serialMutex);
      }
      if (xSemaphoreTake(positionMutex, portMAX_DELAY))
      {
        currentPosition.z = 0;
        xSemaphoreGive(positionMutex);
      }
      return;
    }

    unsigned long stepDelay = calculateStepDelay(i, absSteps);
    
    digitalWrite(STEP_PIN_Z, HIGH);
    delayMicroseconds(stepDelay / 2);
    digitalWrite(STEP_PIN_Z, LOW);
    delayMicroseconds(stepDelay / 2);
  }

  // Update position
  if (xSemaphoreTake(positionMutex, portMAX_DELAY))
  {
    currentPosition.z += steps;
    if (xSemaphoreTake(serialMutex, portMAX_DELAY))
    {
      Serial.print("Z position: ");
      Serial.println(currentPosition.z);
      xSemaphoreGive(serialMutex);
    }
    xSemaphoreGive(positionMutex);
  }
}

// ============== Stepper Motor Functions (Z YAW) ==============
void moveStepsYaw(int steps)
{
  if (xSemaphoreTake(serialMutex, portMAX_DELAY))
  {
    Serial.print("Yaw movement: ");
    Serial.print(steps);
    Serial.println(" steps");
    xSemaphoreGive(serialMutex);
  }

  digitalWrite(DIR_PIN_YAW, steps > 0 ? HIGH : LOW);
  int absSteps = abs(steps);

  // Move with acceleration and deceleration
  for (int i = 0; i < absSteps; i++)
  {
    unsigned long stepDelay;
    if (i < ACCEL_STEPS) {
      // Acceleration
      stepDelay = YAW_MAX_DELAY - (i * (YAW_MAX_DELAY - YAW_MIN_DELAY) / ACCEL_STEPS);
    } else if (i > absSteps - ACCEL_STEPS) {
      // Deceleration
      int stepsFromEnd = absSteps - i;
      stepDelay = YAW_MAX_DELAY - (stepsFromEnd * (YAW_MAX_DELAY - YAW_MIN_DELAY) / ACCEL_STEPS);
    } else {
      // Constant speed
      stepDelay = YAW_MIN_DELAY;
    }

    digitalWrite(STEP_PIN_YAW, HIGH);
    delayMicroseconds(stepDelay / 2);
    digitalWrite(STEP_PIN_YAW, LOW);
    delayMicroseconds(stepDelay / 2);
  }

  if (xSemaphoreTake(positionMutex, portMAX_DELAY))
  {
    currentPosition.yaw += steps;
    if (xSemaphoreTake(serialMutex, portMAX_DELAY))
    {
      Serial.print("Yaw position: ");
      Serial.println(currentPosition.yaw);
      xSemaphoreGive(serialMutex);
    }
    xSemaphoreGive(positionMutex);
  }
}

// Add function for rotating the gripper
void rotateGripper(int degrees)
{
  if (xSemaphoreTake(positionMutex, portMAX_DELAY))
  {
    int currentAngle = currentPosition.gripperRotation;
    int targetAngle = constrain(currentAngle + degrees, ROTATION_MIN_ANGLE, ROTATION_MAX_ANGLE);
    
    // Calculate steps for smooth movement
    const int STEPS = 10;
    float angleStep = (targetAngle - currentAngle) / (float)STEPS;
    
    // Gradually move to target position
    for (int i = 0; i < STEPS; i++)
    {
      float nextAngle = currentAngle + angleStep * (i + 1);
      myservo4.write(nextAngle);
      delay(20);  // Short delay between steps
    }
    
    // Ensure final position is exact
    myservo4.write(targetAngle);
    currentPosition.gripperRotation = targetAngle;

    if (xSemaphoreTake(serialMutex, portMAX_DELAY))
    {
      Serial.print("Gripper rotation: ");
      Serial.print(targetAngle);
      Serial.println(" degrees");
      xSemaphoreGive(serialMutex);
    }

    xSemaphoreGive(positionMutex);
  }
}

// Function to open the gripper with smooth movement
void openGripper()
{
  const int STEPS = 10;
  float currentAngle = myservo3.read();
  float angleStep = (GRIPPER_OPEN_ANGLE - currentAngle) / STEPS;

  for (int i = 0; i < STEPS; i++)
  {
    float nextAngle = currentAngle + angleStep * (i + 1);
    myservo3.write(nextAngle);
    delay(20);
  }

  // Ensure final position
  myservo3.write(GRIPPER_OPEN_ANGLE);

  if (xSemaphoreTake(positionMutex, portMAX_DELAY))
  {
    currentPosition.isGripperOpen = true;
    xSemaphoreGive(positionMutex);
  }

  if (xSemaphoreTake(serialMutex, portMAX_DELAY))
  {
    Serial.println("Gripper status: open");
    xSemaphoreGive(serialMutex);
  }
}

// Function to close the gripper with smooth movement
void closeGripper()
{
  const int STEPS = 10;
  float currentAngle = myservo3.read();
  float angleStep = (GRIPPER_CLOSED_ANGLE - currentAngle) / STEPS;

  for (int i = 0; i < STEPS; i++)
  {
    float nextAngle = currentAngle + angleStep * (i + 1);
    myservo3.write(nextAngle);
    delay(20);
  }

  // Ensure final position
  myservo3.write(GRIPPER_CLOSED_ANGLE);

  if (xSemaphoreTake(positionMutex, portMAX_DELAY))
  {
    currentPosition.isGripperOpen = false;
    xSemaphoreGive(positionMutex);
  }

  if (xSemaphoreTake(serialMutex, portMAX_DELAY))
  {
    Serial.println("Gripper status: closed");
    xSemaphoreGive(serialMutex);
  }
}

void calibrateToZero()
{
  if (xSemaphoreTake(serialMutex, portMAX_DELAY))
  {
    Serial.println("Starting Z-axis calibration...");
    xSemaphoreGive(serialMutex);
  }

  digitalWrite(DIR_PIN_Z, HIGH); // set direction DOWN initially

  // Move down until limit switch is triggered
  while (digitalRead(LIMIT_SWITCH_PIN_Z) == HIGH)
  {
    digitalWrite(STEP_PIN_Z, HIGH);
    delayMicroseconds(MAX_STEP_DELAY / 2);
    digitalWrite(STEP_PIN_Z, LOW);
    delayMicroseconds(MAX_STEP_DELAY / 2);
  }

  // Move extra steps down
  const int extraSteps = 100;
  for (int i = 0; i < extraSteps; i++)
  {
    digitalWrite(STEP_PIN_Z, HIGH);
    delayMicroseconds(MAX_STEP_DELAY / 2);
    digitalWrite(STEP_PIN_Z, LOW);
    delayMicroseconds(MAX_STEP_DELAY / 2);
  }

  // Set zero position
  if (xSemaphoreTake(positionMutex, portMAX_DELAY))
  {
    currentPosition.z = 0;
    xSemaphoreGive(positionMutex);
  }

  if (xSemaphoreTake(serialMutex, portMAX_DELAY))
  {
    Serial.println("Z-axis Calibration complete - Position set to 0");
    Serial.print("Movement range: ");
    Serial.print(Z_MIN_POSITION);
    Serial.print(" to ");
    Serial.println(Z_MAX_POSITION);
    xSemaphoreGive(serialMutex);
  }
}

void resetAll()
{
  if (xSemaphoreTake(serialMutex, portMAX_DELAY))
  {
    Serial.println("Starting system reset...");
    xSemaphoreGive(serialMutex);
  }

  // First move servos to safe position
  myservo1.write(90);  // Elbow
  delay(500);
  myservo2.write(90);  // Base
  delay(500);
  myservo3.write(GRIPPER_OPEN_ANGLE);  // Gripper open
  delay(500);
  myservo4.write(90);  // Gripper rotation
  delay(500);

  // Reset position variables
  if (xSemaphoreTake(positionMutex, portMAX_DELAY))
  {
    currentPosition.x = -86.87;  // Initial X position
    currentPosition.y = 85.47;   // Initial Y position
    currentPosition.z = 0;       // Reset Z position
    currentPosition.yaw = 0;     // Reset Yaw position
    currentPosition.angle1 = 90; // Reset servo angles
    currentPosition.angle2 = 90;
    currentPosition.isGripperOpen = true;
    currentPosition.gripperRotation = 90;
    xSemaphoreGive(positionMutex);
  }

  // Calibrate Z axis
  if (xSemaphoreTake(serialMutex, portMAX_DELAY))
  {
    Serial.println("Starting Z-axis calibration...");
    xSemaphoreGive(serialMutex);
  }

  digitalWrite(DIR_PIN_Z, HIGH); // set direction DOWN initially

  // Move down until limit switch is triggered
  while (digitalRead(LIMIT_SWITCH_PIN_Z) == HIGH)
  {
    digitalWrite(STEP_PIN_Z, HIGH);
    delayMicroseconds(MAX_STEP_DELAY / 2);
    digitalWrite(STEP_PIN_Z, LOW);
    delayMicroseconds(MAX_STEP_DELAY / 2);
  }

  // Move extra steps down for safety
  const int extraSteps = 100;
  for (int i = 0; i < extraSteps; i++)
  {
    digitalWrite(STEP_PIN_Z, HIGH);
    delayMicroseconds(MAX_STEP_DELAY / 2);
    digitalWrite(STEP_PIN_Z, LOW);
    delayMicroseconds(MAX_STEP_DELAY / 2);
  }

  // Update Z position
  if (xSemaphoreTake(positionMutex, portMAX_DELAY))
  {
    currentPosition.z = 0;
    xSemaphoreGive(positionMutex);
  }

  // Reset Yaw to center position
  if (xSemaphoreTake(serialMutex, portMAX_DELAY))
  {
    Serial.println("Resetting Yaw position...");
    xSemaphoreGive(serialMutex);
  }

  // Move to initial XY position
  moveToPosition(currentPosition.x, currentPosition.y);

  if (xSemaphoreTake(serialMutex, portMAX_DELAY))
  {
    Serial.println("System reset complete");
    Serial.println("Current positions:");
    Serial.print("X: "); Serial.print(currentPosition.x);
    Serial.print(", Y: "); Serial.print(currentPosition.y);
    Serial.print(", Z: "); Serial.print(currentPosition.z);
    Serial.print(", Yaw: "); Serial.println(currentPosition.yaw);
    Serial.print("Angles - Elbow: "); Serial.print(currentPosition.angle1);
    Serial.print(", Base: "); Serial.println(currentPosition.angle2);
    Serial.print("Gripper - Position: "); Serial.print(currentPosition.isGripperOpen ? "Open" : "Closed");
    Serial.print(", Rotation: "); Serial.println(currentPosition.gripperRotation);
    xSemaphoreGive(serialMutex);
  }
}
