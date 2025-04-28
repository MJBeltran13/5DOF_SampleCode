#include <ESP32Servo.h>
#include <EEPROM.h>
#include <math.h> // for sin(), cos()

// Servo objects
Servo myservo1; // Elbow
Servo myservo2; // Base
Servo myservo3; // Gripper

const int servoPin1 = 33;
const int servoPin2 = 25;
const int servoPin3 = 26;

// Link lengths in mm
const float L1 = 85.47;
const float L2 = 86.87;

// Servo angle limits
const int BASE_MIN_ANGLE = 30;
const int BASE_MAX_ANGLE = 150;
const int ELBOW_MIN_ANGLE = 30;
const int ELBOW_MAX_ANGLE = 150;

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
const unsigned long stepDelay = 1000; // microseconds delay between steps
const unsigned long yawStepDelay = 500000; // microseconds delay for yaw movement (1 second per half step, 2 seconds total per step)

// EEPROM address for Z lift position
const int positionAddress = 0;

// Variables for Z control
int currentPositionZ = 0;
int currentPositionYaw = 0; // New for YAW motor

// Z-axis position limits
const int Z_MIN_POSITION = -1900;  // Lower limit
const int Z_MAX_POSITION = 0;      // Upper limit

void setup()
{
  Serial.begin(9600);

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

  Serial.println("System ready.");
  Serial.println("Commands:");
  Serial.println("angles A1 A2    - move servos to angles");
  Serial.println("position X Y    - move servos to reach XY");
  Serial.println("u100            - move Z lift up 100 steps");
  Serial.println("d100            - move Z lift down 100 steps");
  Serial.println("l100            - rotate yaw left 100 steps");
  Serial.println("r100            - rotate yaw right 100 steps");
  Serial.println("p               - print positions");
}

void loop()
{
  if (Serial.available() > 0)
  {
    String inputString = Serial.readStringUntil('\n');
    inputString.trim();

    // Single character commands first
    if (inputString.length() == 1 || inputString.startsWith("u") || inputString.startsWith("d") || inputString.startsWith("l") || inputString.startsWith("r"))
    {
      char command = inputString.charAt(0);

      int steps = inputString.substring(1).toInt();
      if (command == 'u')
      {
        moveStepsZ(steps);
      }
      else if (command == 'd')
      {
        moveStepsZ(-steps);
      }
      else if (command == 'l')
      {
        moveStepsYaw(-steps);
      }
      else if (command == 'r')
      {
        moveStepsYaw(steps);
      }

      if (command == 'u' || command == 'd')
      {
        Serial.print("Z Lift Position: ");
        Serial.println(currentPositionZ);
      }
      if (command == 'l' || command == 'r')
      {
        Serial.print("Yaw Position: ");
        Serial.println(currentPositionYaw);
      }
    }
    else if (inputString == "p")
    {
      Serial.print("Current Z Lift Position: ");
      Serial.println(currentPositionZ);
      Serial.print("Current Yaw Position: ");
      Serial.println(currentPositionYaw);
    }
    else
    {
      // Otherwise assume XY mode (angles/position)
      int spaceIndex = inputString.indexOf(' ');
      if (spaceIndex != -1)
      {
        String mode = inputString.substring(0, spaceIndex);
        String data = inputString.substring(spaceIndex + 1);

        if (mode == "angles")
        {
          float angle1 = data.substring(0, data.indexOf(' ')).toFloat();
          float angle2 = data.substring(data.indexOf(' ') + 1).toFloat();

          angle1 = constrain(angle1, ELBOW_MIN_ANGLE, ELBOW_MAX_ANGLE);
          angle2 = constrain(angle2, BASE_MIN_ANGLE, BASE_MAX_ANGLE);

          myservo1.write(angle1);
          myservo2.write(angle2);

          Serial.print("Servo 1 (Elbow) set to: ");
          Serial.println(angle1);
          Serial.print("Servo 2 (Base) set to: ");
          Serial.println(angle2);

          calculateKinematics();
        }
        else if (mode == "position")
        {
          float x = data.substring(0, data.indexOf(' ')).toFloat();
          float y = data.substring(data.indexOf(' ') + 1).toFloat();

          calculateInverseKinematics(x, y);
        }
        else
        {
          Serial.println("Invalid mode. Use 'angles' or 'position'.");
        }
      }
    }
  }

  delay(100); // small delay
}

// ============== Servo Functions (X, Y Control) ==============
void calculateKinematics()
{
  float angle1 = myservo1.read();
  float angle2 = myservo2.read();
  float gripperAngle = myservo3.read();

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

  Serial.print("Gripper Rotation Angle (Servo 3): ");
  Serial.print(gripperAngle);
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

void calibrateToZero()
{
  digitalWrite(DIR_PIN_Z, HIGH); // set direction DOWN initially

  // Move down until limit switch is triggered
  while (digitalRead(LIMIT_SWITCH_PIN_Z) == HIGH)
  {
    digitalWrite(STEP_PIN_Z, HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(STEP_PIN_Z, LOW);
    delayMicroseconds(stepDelay);
  }

  // Limit switch triggered! Now IGNORE it, move 100 more steps down
  const int extraSteps = 100;
  for (int i = 0; i < extraSteps; i++)
  {
    digitalWrite(STEP_PIN_Z, HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(STEP_PIN_Z, LOW);
    delayMicroseconds(stepDelay);
  }

  // After moving extra steps, set zero position
  currentPositionZ = 0;
  EEPROM.write(positionAddress, currentPositionZ);

  Serial.println("Z-axis Calibration complete - Position set to 0");
  Serial.print("Movement range: ");
  Serial.print(Z_MIN_POSITION);
  Serial.print(" to ");
  Serial.println(Z_MAX_POSITION);
}

void moveStepsZ(int steps)
{
  // Calculate new position before moving
  int newPosition = currentPositionZ + steps;

  // Check if movement would exceed limits
  if (newPosition < Z_MIN_POSITION) {
    Serial.println("Cannot move down: Would exceed lower limit");
    return;
  }
  if (newPosition > Z_MAX_POSITION) {
    Serial.println("Cannot move up: Would exceed upper limit");
    return;
  }

  if (steps > 0)
  {
    digitalWrite(DIR_PIN_Z, HIGH);
  }
  else
  {
    digitalWrite(DIR_PIN_Z, LOW);
  }

  for (int i = 0; i < abs(steps); i++)
  {
    // Only check limit switch when moving up AND current position is below or at zero
    if (steps > 0 && currentPositionZ <= 0 && digitalRead(LIMIT_SWITCH_PIN_Z) == LOW)
    {
      Serial.println("Limit switch triggered - Stopping upward movement");
      currentPositionZ = 0;
      EEPROM.write(positionAddress, currentPositionZ);
      return;
    }
    
    digitalWrite(STEP_PIN_Z, HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(STEP_PIN_Z, LOW);
    delayMicroseconds(stepDelay);
  }

  currentPositionZ += steps;
  EEPROM.write(positionAddress, currentPositionZ);
}

// ============== Stepper Motor Functions (Z YAW) ==============
void moveStepsYaw(int steps)
{
  if (steps > 0)
  {
    digitalWrite(DIR_PIN_YAW, HIGH);
  }
  else
  {
    digitalWrite(DIR_PIN_YAW, LOW);
  }

  for (int i = 0; i < abs(steps); i++)
  {
    digitalWrite(STEP_PIN_YAW, HIGH);
    delayMicroseconds(yawStepDelay);
    digitalWrite(STEP_PIN_YAW, LOW);
    delayMicroseconds(yawStepDelay);
  }

  currentPositionYaw += steps;
}
