// Joystick 1 Pins (Left/Right, Forward/Backward, and Button)
const int VRX_PIN = 34; // X-axis (left/right)
const int VRY_PIN = 35; // Y-axis (forward/backward)
const int SW_PIN  = 5;  // Button

// Calibrated neutral values for Joystick 1 (adjust based on your hardware)
const int centerX = 3030;
const int centerY = 2928;

// Thresholds for Joystick 1 movement detection
const int thresholdX = 200;
const int thresholdY = 200;

// Joystick 2 Pins (Gripper and Up/Down)
const int VRX2_PIN = 32; // X-axis (gripper open/close)
const int VRY2_PIN = 33; // Y-axis (up/down)

// Calibrated neutral values for Joystick 2 (adjust if needed)
const int centerX2 = 3030;
const int centerY2 = 2928;

// Thresholds for Joystick 2 movement detection
const int thresholdX2 = 200;
const int thresholdY2 = 200;

// Determines the dominant direction for Joystick 1.
String getDirection(int xValue, int yValue) {
  int diffX = xValue - centerX;
  int diffY = yValue - centerY;
  
  // If both axes are within threshold, return "Neutral".
  if (abs(diffX) < thresholdX && abs(diffY) < thresholdY) {
    return "Neutral";
  }
  
  // Choose the axis with the greater deviation.
  if (abs(diffX) >= abs(diffY)) {
    return (diffX > 0) ? "Right" : "Left";
  } else {
    return (diffY > 0) ? "Backward" : "Forward";
  }
}

// Determines the dominant direction for Joystick 2.
// Up/Down control is tied to the Y-axis, and gripper control to the X-axis.
String getDirection2(int xValue, int yValue) {
  int diffX = xValue - centerX2;
  int diffY = yValue - centerY2;
  
  // If both axes are within threshold, return "Neutral".
  if (abs(diffX) < thresholdX2 && abs(diffY) < thresholdY2) {
    return "Neutral";
  }
  
  // If vertical (Y) movement is dominant, control up/down.
  if (abs(diffY) >= abs(diffX)) {
    return (diffY < 0) ? "Down" : "Up";
  } 
  // Otherwise, control gripper: push right for open, left for close.
  else {
    return (diffX > 0) ? "Gripper Open" : "Gripper Close";
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(SW_PIN, INPUT_PULLUP); // Enable internal pull-up for Joystick 1 button
}

void loop() {
  // Read Joystick 1 values
  int xValue = analogRead(VRX_PIN);
  int yValue = analogRead(VRY_PIN);
  int swState = digitalRead(SW_PIN);
  String direction = getDirection(xValue, yValue);
  
  // Read Joystick 2 values
  int xValue2 = analogRead(VRX2_PIN);
  int yValue2 = analogRead(VRY2_PIN);
  String direction2 = getDirection2(xValue2, yValue2);
  
  // Print Joystick 1 info
  Serial.print("Joystick1 - X: ");
  Serial.print(xValue);
  Serial.print(" | Y: ");
  Serial.print(yValue);
  Serial.print(" | Button: ");
  Serial.print(swState == LOW ? "Pressed" : "Released");
  Serial.print(" | Direction: ");
  Serial.print(direction);
  
  // Print Joystick 2 info
  Serial.print(" || Joystick2 - X: ");
  Serial.print(xValue2);
  Serial.print(" | Y: ");
  Serial.print(yValue2);
  Serial.print(" | Direction: ");
  Serial.println(direction2);
  
  delay(100); // Delay to avoid spamming the output
}
