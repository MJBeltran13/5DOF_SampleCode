#include <ESP32Servo.h>
#include <math.h> // for sin(), cos()

// Servo objects
Servo myservo1; // Elbow
Servo myservo2; // Base
Servo myservo3; // Gripper
Servo myservo4; // Gripper Rotation

// Position structure
struct Position {
    float x;
    float y;
    int z;
    int yaw;
    float angle1;
    float angle2;
    bool isGripperOpen;
    int gripperRotation;
};

// Global position state
Position currentPosition = {
    -86.87,  // x
    85.47,   // y
    0,       // z
    0,       // yaw
    90,      // angle1
    90,      // angle2
    true,    // isGripperOpen
    90       // gripperRotation
};

// Pin definitions
const int servoPin1 = 33;  // Elbow
const int servoPin2 = 25;  // Base
const int servoPin3 = 26;  // Gripper
const int servoPin4 = 27;  // Gripper Rotation

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
const unsigned long stepDelay = 1000; // microseconds delay between steps
const unsigned long yawStepDelay = stepDelay; // Set yaw delay same as Z-axis delay

// Z-axis position limits
const int Z_MIN_POSITION = -1900;  // Lower limit
const int Z_MAX_POSITION = 0;      // Upper limit

String inputString = ""; // For storing incoming serial data

void setup() {
    Serial.begin(9600);

    // Setup for servos
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);

    // Initialize servos
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

    Serial.println("System ready.");
    printCommands();
}

void loop() {
    // Handle serial input
    while (Serial.available() > 0) {
        char inChar = (char)Serial.read();
        if (inChar == '\n') {
            processCommand(inputString);
            inputString = "";
        } else {
            inputString += inChar;
        }
    }
}

void processCommand(String input) {
    input.trim();
    
    if (input == "reset") {
        resetAll();
    } else if (input == "go") {
        openGripper();
    } else if (input == "gc") {
        closeGripper();
    } else if (input.startsWith("u")) {
        int steps = input.substring(1).toInt();
        Serial.print("Moving up: ");
        Serial.println(steps);
        moveStepsZ(steps);
    } else if (input.startsWith("d")) {
        int steps = input.substring(1).toInt();
        Serial.print("Moving down: ");
        Serial.println(steps);
        moveStepsZ(-steps);
    } else if (input.startsWith("l")) {
        int steps = input.substring(1).toInt();
        Serial.print("Moving left: ");
        Serial.println(steps);
        moveStepsYaw(-steps);
    } else if (input.startsWith("r")) {
        int steps = input.substring(1).toInt();
        Serial.print("Moving right: ");
        Serial.println(steps);
        moveStepsYaw(steps);
    } else if (input.startsWith("gl")) {
        int degrees = input.substring(2).toInt();
        rotateGripper(-degrees);
    } else if (input.startsWith("gr")) {
        int degrees = input.substring(2).toInt();
        rotateGripper(degrees);
    } else if (input == "p") {
        printPosition();
    } else if (input.startsWith("angles ")) {
        String data = input.substring(7);
        float angle1 = data.substring(0, data.indexOf(' ')).toFloat();
        float angle2 = data.substring(data.indexOf(' ') + 1).toFloat();
        moveToAngles(angle1, angle2);
    } else if (input.startsWith("position ")) {
        String data = input.substring(9);
        float x = data.substring(0, data.indexOf(' ')).toFloat();
        float y = data.substring(data.indexOf(' ') + 1).toFloat();
        moveToPosition(x, y);
    }
}

void printCommands() {
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
}

void printPosition() {
    Serial.println("Current Position:");
    Serial.print("X: "); Serial.print(currentPosition.x);
    Serial.print(", Y: "); Serial.println(currentPosition.y);
    Serial.print("Z: "); Serial.println(currentPosition.z);
    Serial.print("Yaw: "); Serial.println(currentPosition.yaw);
    Serial.print("Angle1: "); Serial.println(currentPosition.angle1);
    Serial.print("Angle2: "); Serial.println(currentPosition.angle2);
    Serial.print("Gripper: "); Serial.println(currentPosition.isGripperOpen ? "Open" : "Closed");
    Serial.print("Gripper Rotation: "); Serial.println(currentPosition.gripperRotation);
}

void moveToAngles(float angle1, float angle2) {
    // Constrain angles to valid ranges
    angle1 = constrain(angle1, ELBOW_MIN_ANGLE, ELBOW_MAX_ANGLE);
    angle2 = constrain(angle2, BASE_MIN_ANGLE, BASE_MAX_ANGLE);

    // Move servos
    myservo1.write(angle1);
    myservo2.write(angle2);

    // Update position
    currentPosition.angle1 = angle1;
    currentPosition.angle2 = angle2;
    
    // Calculate new X,Y position
    float theta1 = radians(angle1);
    float theta2 = radians(angle2);
    currentPosition.x = L1 * cos(theta1) + L2 * cos(theta1 + theta2);
    currentPosition.y = L1 * sin(theta1) + L2 * sin(theta1 + theta2);

    Serial.print("Servo 1 (Elbow) set to: ");
    Serial.println(angle1);
    Serial.print("Servo 2 (Base) set to: ");
    Serial.println(angle2);
}

void moveToPosition(float x, float y) {
    float r = sqrt(x * x + y * y);

    if (r > (L1 + L2)) {
        Serial.println("Target is out of reach!");
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

    Serial.print("Target Position -> X: ");
    Serial.print(x);
    Serial.print(" mm, Y: ");
    Serial.println(y);
}

void moveStepsZ(int steps) {
    // Calculate new position before moving
    int newPosition = currentPosition.z + steps;
    
    // Check if movement would exceed limits
    if (newPosition < Z_MIN_POSITION) {
        Serial.println("Cannot move down: Would exceed lower limit");
        return;
    }
    if (newPosition > Z_MAX_POSITION) {
        Serial.println("Cannot move up: Would exceed upper limit");
        return;
    }

    // Set direction
    digitalWrite(DIR_PIN_Z, steps > 0 ? HIGH : LOW);
    
    // Move stepper
    for (int i = 0; i < abs(steps); i++) {
        // Check limit switch when moving up
        if (steps > 0 && currentPosition.z <= 0 && digitalRead(LIMIT_SWITCH_PIN_Z) == LOW) {
            Serial.println("Limit switch triggered - Stopping upward movement");
            currentPosition.z = 0;
            return;
        }
        
        digitalWrite(STEP_PIN_Z, HIGH);
        delayMicroseconds(stepDelay);
        digitalWrite(STEP_PIN_Z, LOW);
        delayMicroseconds(stepDelay);
        
        if (i % 10 == 0) {
            Serial.print("Step ");
            Serial.print(i + 1);
            Serial.print(" of ");
            Serial.println(abs(steps));
        }
    }

    // Update position
    currentPosition.z += steps;
    Serial.print("Z Lift Position updated to: ");
    Serial.println(currentPosition.z);
}

void moveStepsYaw(int steps) {
    digitalWrite(DIR_PIN_YAW, steps > 0 ? HIGH : LOW);
    
    Serial.print("Moving yaw ");
    Serial.print(steps > 0 ? "right" : "left");
    Serial.print(" by ");
    Serial.print(abs(steps));
    Serial.println(" steps");

    for (int i = 0; i < abs(steps); i++) {
        digitalWrite(STEP_PIN_YAW, HIGH);
        delayMicroseconds(yawStepDelay);
        digitalWrite(STEP_PIN_YAW, LOW);
        delayMicroseconds(yawStepDelay);
        
        if (i % 10 == 0) {
            Serial.print("Step ");
            Serial.print(i + 1);
            Serial.print(" of ");
            Serial.println(abs(steps));
        }
    }

    currentPosition.yaw += steps;
    Serial.print("Yaw Position: ");
    Serial.println(currentPosition.yaw);
}

void rotateGripper(int degrees) {
    int currentAngle = currentPosition.gripperRotation;
    int newAngle = currentAngle + degrees;
    
    // Constrain to valid range
    newAngle = constrain(newAngle, ROTATION_MIN_ANGLE, ROTATION_MAX_ANGLE);
    
    // Move to new position
    myservo4.write(newAngle);
    currentPosition.gripperRotation = newAngle;
    
    Serial.print("Gripper Rotation: ");
    Serial.print(currentAngle);
    Serial.print(" -> ");
    Serial.print(newAngle);
    Serial.println(" degrees");
}

void openGripper() {
    myservo3.write(GRIPPER_OPEN_ANGLE);
    currentPosition.isGripperOpen = true;
    Serial.println("Gripper opened");
}

void closeGripper() {
    myservo3.write(GRIPPER_CLOSED_ANGLE);
    currentPosition.isGripperOpen = false;
    Serial.println("Gripper closed");
}

void calibrateToZero() {
    Serial.println("Starting Z-axis calibration...");

    digitalWrite(DIR_PIN_Z, HIGH); // set direction DOWN initially

    // Move down until limit switch is triggered
    while (digitalRead(LIMIT_SWITCH_PIN_Z) == HIGH) {
        digitalWrite(STEP_PIN_Z, HIGH);
        delayMicroseconds(stepDelay);
        digitalWrite(STEP_PIN_Z, LOW);
        delayMicroseconds(stepDelay);
    }

    // Move extra steps down
    const int extraSteps = 100;
    for (int i = 0; i < extraSteps; i++) {
        digitalWrite(STEP_PIN_Z, HIGH);
        delayMicroseconds(stepDelay);
        digitalWrite(STEP_PIN_Z, LOW);
        delayMicroseconds(stepDelay);
    }

    // Set zero position
    currentPosition.z = 0;

    Serial.println("Z-axis Calibration complete - Position set to 0");
    Serial.print("Movement range: ");
    Serial.print(Z_MIN_POSITION);
    Serial.print(" to ");
    Serial.println(Z_MAX_POSITION);
}

void resetAll() {
    // Reset positions
    currentPosition.z = 0;
    currentPosition.yaw = 0;
    currentPosition.angle1 = 90;
    currentPosition.angle2 = 90;
    currentPosition.isGripperOpen = true;
    currentPosition.gripperRotation = 90;
    
    // Reset servos to default positions
    myservo1.write(90);  // Elbow
    myservo2.write(90);  // Base
    myservo3.write(GRIPPER_OPEN_ANGLE);  // Gripper open
    myservo4.write(90);  // Gripper rotation
    
    // Calibrate Z axis
    calibrateToZero();
    
    Serial.println("System reset complete - All positions set to default");
} 