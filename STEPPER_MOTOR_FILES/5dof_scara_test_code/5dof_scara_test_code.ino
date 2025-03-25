// Stepper motor pins
const int STEP_PIN = 2;  // Step pin
const int DIR_PIN = 4;   // Direction pin

// Microstepping pins
const int MS1_PIN = 35;  // MS1 pin for microstepping
const int MS2_PIN = 32;  // MS2 pin for microstepping
const int MS3_PIN = 33;  // MS3 pin for microstepping

// Motor parameters
const int STEP_DELAY = 6000;  // Microseconds between steps (higher number = slower rotation)
const bool DIRECTION = false;   // true = clockwise, false = counterclockwise

// Monitoring variables
unsigned long stepCount = 0;
unsigned long lastReport = 0;
const unsigned long REPORT_INTERVAL = 1000; // Report every 1000ms

void setup() {
  // Initialize Serial communication
  Serial.begin(115200);
  
  // Wait for serial connection to be established
  while(!Serial) {
    delay(100);
  }
  
  Serial.println("\n=== ESP32-S3 SCARA Robot Initializing ===");
  
  // Configure stepper motor pins
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  
  // Configure microstepping pins
  pinMode(MS1_PIN, OUTPUT);
  pinMode(MS2_PIN, OUTPUT);
  pinMode(MS3_PIN, OUTPUT);
  
  // Set microstepping pins to high for the desired microstepping mode
  digitalWrite(MS1_PIN, HIGH);  // Set MS1 to HIGH
  digitalWrite(MS2_PIN, HIGH);  // Set MS2 to HIGH
  digitalWrite(MS3_PIN, HIGH);  // Set MS3 to HIGH
  
  // Set initial direction
  digitalWrite(DIR_PIN, DIRECTION);
  Serial.print("Motor direction set to: ");
  Serial.println(DIRECTION ? "CLOCKWISE" : "COUNTERCLOCKWISE");
  Serial.print("Step delay set to: ");
  Serial.print(STEP_DELAY);
  Serial.println(" microseconds");
  
  Serial.println("Setup complete! Starting motor...\n");
}

void loop() {
  // Generate one step pulse
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(STEP_DELAY);
  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(STEP_DELAY);
  
  // Increment step counter
  stepCount++;
  
  // Report status every REPORT_INTERVAL milliseconds
  if (millis() - lastReport >= REPORT_INTERVAL) {
    float stepsPerSecond = stepCount * (1000.0 / REPORT_INTERVAL);
    float rpm = (stepsPerSecond * 60.0) / 200.0; // Assuming 200 steps/revolution
    
    Serial.print("Steps taken: ");
    Serial.print(stepCount);
    Serial.print(" | Steps/sec: ");
    Serial.print(stepsPerSecond, 1);
    Serial.print(" | Approx RPM: ");
    Serial.println(rpm, 1);
    
    // Reset counter and update last report time
    stepCount = 0;
    lastReport = millis();
  }
}
