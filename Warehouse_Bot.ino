#include <Arduino.h>

// ================= PIN DEFINITIONS =================

// --- L298N Motor Driver ---
#define ENA 5  // Left Motor PWM

#define IN1 18
#define IN2 23
#define ENB 4   // Right Motor PWM
#define IN3 18
#define IN4 5

// --- IR Array (5 sensors) ---
#define NUM_SENSORS 5
int sensors[NUM_SENSORS] = {26, 25, 33, 32, 35};  // S1 → S5 connections

// --- HC-SR04 Ultrasonic Sensor ---
#define TRIG_PIN 14
#define ECHO_PIN 27

// --- LED indicator ---
#define LED 2  // Built-in LED or external

// --- Piston Control Pins ---
#define PISTON_1 13   // First piston control
#define PISTON_2 12   // Second piston control

// ================= ULTRASONIC SENSOR SETTINGS =================
#define SLOT_DISTANCE 20.0  // Slot detection at 15cm distance
#define DISTANCE_TOLERANCE 2.0  // ±2cm tolerance for slots

// ================= OPTIMIZED PID VARIABLES =================
float Kp = 12.0;   // Increased for stronger correction
float Ki = 0.0;
float Kd = 3.0;    // Increased derivative for better damping

float error = 0, previous_error = 0, integral = 0;
float PID_Value = 0;
int base_speed = 175;  // Reduced for better control

// Optimized sensor weights for center-line tracking
int weights[NUM_SENSORS] = {-6, -3, 0, 3, 6};  // Balanced weights

// Global variables for advanced turn handling
bool lineDetected = true;
unsigned long lineLostTime = 0;
const unsigned long LINE_LOST_TIMEOUT = 2000;
bool inSharpTurn = false;
int consecutiveSharpTurns = 0;
int searchDirection = 0;
int searchIntensity = 0;

// ================= SLOT DETECTION MANAGEMENT =================
bool pistonSequenceActive = false;
int pistonSequenceStep = 0;
unsigned long pistonStepStartTime = 0;
unsigned long lastSlotDetection = 0;
const unsigned long SLOT_COOLDOWN = 10000; // 10 seconds between slot detections

// Ultrasonic debugging
unsigned long lastUltrasonicPrint = 0;
const unsigned long ULTRASONIC_PRINT_INTERVAL = 500; // Print every 500ms

// ================= SETUP =================
void setup() {
  Serial.begin(115200);

  // --- Motor driver pins ---
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // --- LED indicator ---
  pinMode(LED, OUTPUT);

  // --- IR sensors ---
  for (int i = 0; i < NUM_SENSORS; i++)
    pinMode(sensors[i], INPUT);

  // --- Ultrasonic sensor pins ---
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  // --- Piston control pins ---
  pinMode(PISTON_1, OUTPUT);
  pinMode(PISTON_2, OUTPUT);
  
  // Initialize pistons to OFF state
  digitalWrite(PISTON_1, LOW);
  digitalWrite(PISTON_2, LOW);

  // --- PWM setup (new API) ---
  ledcAttach(ENA, 5000, 8);
  ledcAttach(ENB, 5000, 8);

  Serial.println("=== ROBOT SYSTEM INITIALIZED ===");
  Serial.println("Line Following + MULTIPLE SLOT DETECTION");
  Serial.print("Slot Detection: ");
  Serial.print(SLOT_DISTANCE);
  Serial.println(" cm range");
  Serial.println("================================");

  digitalWrite(LED, HIGH);
  delay(10000);  // Wait 10 seconds before start
  digitalWrite(LED, LOW);
  
  Serial.println("STARTING LINE FOLLOWING...");
}

// ================= ULTRASONIC SENSOR FUNCTIONS =================
float getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  
  if (duration == 0) {
    return -1;
  }
  
  float distance = (duration * 0.0343) / 2.0;
  return distance;
}

bool checkSlotDetection() {
  // Don't check for new slots while piston sequence is active
  if (pistonSequenceActive) {
    return false;
  }
  
  // Cooldown period to prevent multiple rapid detections
  unsigned long currentTime = millis();
  if (currentTime - lastSlotDetection < SLOT_COOLDOWN) {
    return false;
  }
  
  float distance = getDistance();
  
  // Check for valid distance reading
  if (distance <= 0 || distance > 100) {
    return false;
  }
  
  // Print distance status periodically
  if (currentTime - lastUltrasonicPrint > ULTRASONIC_PRINT_INTERVAL) {
    Serial.print("ULTRASONIC: Distance = ");
    Serial.print(distance);
    Serial.print(" cm | Slot Range = ");
    Serial.print(SLOT_DISTANCE);
    Serial.print("±");
    Serial.print(DISTANCE_TOLERANCE);
    Serial.print(" cm | ");
    
    if (abs(distance - SLOT_DISTANCE) <= DISTANCE_TOLERANCE) {
      Serial.println("🎯 SLOT DETECTED!");
    } else {
      Serial.println("Searching for slots...");
    }
    
    lastUltrasonicPrint = currentTime;
  }
  
  // Check for slot detection: 15cm ± tolerance
  if (abs(distance - SLOT_DISTANCE) <= DISTANCE_TOLERANCE) {
    Serial.println("=== SLOT DETECTED ===");
    Serial.print("🎯 Slot at: ");
    Serial.print(distance);
    Serial.print(" cm (Target: ");
    Serial.print(SLOT_DISTANCE);
    Serial.println(" cm)");
    lastSlotDetection = currentTime;
    return true;
  }
  
  return false;
}

// ================= PISTON SEQUENCE MANAGEMENT =================
void startPistonSequence() {
  pistonSequenceActive = true;
  pistonSequenceStep = 0;
  pistonStepStartTime = millis();
  
  Serial.println("=== STARTING PISTON SEQUENCE ===");
  stopMotors(); // Stop when slot is detected
  digitalWrite(LED, HIGH);
}

void executePistonSequence() {
  unsigned long currentTime = millis();
  unsigned long stepElapsedTime = currentTime - pistonStepStartTime;
  
  switch (pistonSequenceStep) {
    case 0: // Step 0: Wait 2 seconds then activate Piston 1
      if (stepElapsedTime >= 2000) {
        digitalWrite(PISTON_1, HIGH);
        Serial.println("PISTON: Piston 1 ACTIVATED (Pin 13 HIGH)");
        pistonSequenceStep = 1;
        pistonStepStartTime = currentTime;
      }
      break;
      
    case 1: // Step 1: Wait 2 seconds then activate Piston 2
      if (stepElapsedTime >= 2000) {
        digitalWrite(PISTON_2, HIGH);
        Serial.println("PISTON: Piston 2 ACTIVATED (Pin 12 HIGH)");
        pistonSequenceStep = 2;
        pistonStepStartTime = currentTime;
      }
      break;
      
    case 2: // Step 2: Wait 2 seconds then deactivate Piston 2
      if (stepElapsedTime >= 2000) {
        digitalWrite(PISTON_2, LOW);
        Serial.println("PISTON: Piston 2 DEACTIVATED (Pin 12 LOW)");
        pistonSequenceStep = 3;
        pistonStepStartTime = currentTime;
      }
      break;
      
    case 3: // Step 3: Wait 2 seconds then deactivate Piston 1
      if (stepElapsedTime >= 2000) {
        digitalWrite(PISTON_1, LOW);
        Serial.println("PISTON: Piston 1 DEACTIVATED (Pin 13 LOW)");
        pistonSequenceStep = 4;
        pistonStepStartTime = currentTime;
      }
      break;
      
    case 4: // Step 4: Sequence completed - resume line following
      pistonSequenceActive = false;
      digitalWrite(LED, LOW);
      
      Serial.println("=== PISTON SEQUENCE COMPLETED ===");
      Serial.println("Resuming line following...");
      break;
  }
}

// ================= LINE FOLLOWING FUNCTIONS =================
float computeLineError() {
  int sum = 0, activeCount = 0;
  int sensorReadings[NUM_SENSORS] = {0};

  for (int i = 0; i < NUM_SENSORS; i++) {
    int val = digitalRead(sensors[i]);
    val = !val; // Invert: black = 1, white = 0
    sensorReadings[i] = val;
    sum += val * weights[i];
    activeCount += val;
  }

  // Prioritize center sensor for stability
  if (sensorReadings[2] == 1 && activeCount > 1) {
    sum = sum * 0.7; // Smoothing
  }

  if (activeCount == 0) {
    lineDetected = false;
    if (lineLostTime == 0) {
      lineLostTime = millis();
    }
    return 0;
  } else {
    lineDetected = true;
    lineLostTime = 0;
    searchDirection = 0;
    searchIntensity = 0;
  }

  return (float)sum / activeCount;
}

void performLineSearch() {
  int sensorReadings[NUM_SENSORS] = {0};
  int leftSensors = 0, rightSensors = 0;
  int centerSensor = 0;
  
  for (int i = 0; i < NUM_SENSORS; i++) {
    int val = digitalRead(sensors[i]);
    sensorReadings[i] = !val;
  }
  
  leftSensors = sensorReadings[0] + sensorReadings[1];
  rightSensors = sensorReadings[3] + sensorReadings[4];
  centerSensor = sensorReadings[2];
  
  if (centerSensor == 1) {
    setMotorSpeeds(100, 100);
    searchDirection = 0;
    return;
  }
  
  if (leftSensors > 0 && rightSensors == 0) {
    setMotorSpeeds(-150, 200);
    searchDirection = -1;
    searchIntensity++;
  }
  else if (rightSensors > 0 && leftSensors == 0) {
    setMotorSpeeds(200, -150);
    searchDirection = 1;
    searchIntensity++;
  }
  else if (leftSensors > 0 && rightSensors > 0) {
    setMotorSpeeds(80, 80);
    searchDirection = 0;
  }
  else {
    if (searchDirection == 0) {
      if (previous_error < 0) {
        searchDirection = -1;
      } else {
        searchDirection = 1;
      }
    }
    
    if (searchIntensity < 3) {
      if (searchDirection == -1) {
        setMotorSpeeds(-150, 180);
      } else {
        setMotorSpeeds(180, -150);
      }
    } else if (searchIntensity < 6) {
      if (searchDirection == -1) {
        setMotorSpeeds(-160, 190);
      } else {
        setMotorSpeeds(190, -160);
      }
    } else {
      if (searchDirection == -1) {
        setMotorSpeeds(-170, 200);
      } else {
        setMotorSpeeds(200, -170);
      }
    }
    searchIntensity++;
  }
}

void followLinePID() {
  float lineError = computeLineError();

  if (!lineDetected) {
    if (lineLostTime > 0 && (millis() - lineLostTime) > LINE_LOST_TIMEOUT) {
      Serial.println("LINE FOLLOWING: TIMEOUT - STOPPING MOTORS!");
      stopMotors();
      digitalWrite(LED, HIGH);
      return;
    }

    performLineSearch();
    delay(100);
    return;
  }

  digitalWrite(LED, LOW);
  error = lineError;

  bool centerOnLine = !digitalRead(sensors[2]);
  
  if (abs(error) > 2.0) {
    inSharpTurn = true;
    consecutiveSharpTurns++;
    digitalWrite(LED, HIGH);
  } else {
    inSharpTurn = false;
    consecutiveSharpTurns = 0;
    digitalWrite(LED, LOW);
  }

  integral += error;
  integral = constrain(integral, -30, 30);
  
  float derivative = error - previous_error;
  PID_Value = (Kp * error) + (Ki * integral) + (Kd * derivative);
  previous_error = error;

  int leftSpeed, rightSpeed;

  if (centerOnLine && abs(error) < 1.0) {
    leftSpeed = base_speed + (PID_Value * 0.5);
    rightSpeed = base_speed - (PID_Value * 0.5);
  }
  else if (inSharpTurn && consecutiveSharpTurns > 1) {
    if (error > 0) {
      leftSpeed = 200;
      rightSpeed = -160;
    } else {
      leftSpeed = -160;
      rightSpeed = 200;
    }
  }
  else if (inSharpTurn) {
    float boostFactor = 2.0;
    int turn_base_speed = base_speed * 0.8;
    leftSpeed = turn_base_speed + (PID_Value * boostFactor);
    rightSpeed = turn_base_speed - (PID_Value * boostFactor);
  }
  else {
    leftSpeed = base_speed + PID_Value;
    rightSpeed = base_speed - PID_Value;
  }

  if (abs(error) > 1.5) {
    float speedReduction = 0.8;
    leftSpeed = leftSpeed * speedReduction;
    rightSpeed = rightSpeed * speedReduction;
  }

  if (abs(leftSpeed) < 50 && leftSpeed != 0) leftSpeed = (leftSpeed > 0) ? 50 : -50;
  if (abs(rightSpeed) < 50 && rightSpeed != 0) rightSpeed = (rightSpeed > 0) ? 50 : -50;

  leftSpeed = constrain(leftSpeed, -255, 255);
  rightSpeed = constrain(rightSpeed, -255, 255);

  setMotorSpeeds(leftSpeed, rightSpeed);
}

// ================= MOTOR CONTROL =================
void setMotorSpeeds(int leftSpeed, int rightSpeed) {
  if (leftSpeed >= 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    leftSpeed = -leftSpeed;
  }
  ledcWrite(ENA, constrain(leftSpeed, 0, 255));

  if (rightSpeed >= 0) {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    rightSpeed = -rightSpeed;
  }
  ledcWrite(ENB, constrain(rightSpeed, 0, 255));
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  ledcWrite(ENA, 0);
  ledcWrite(ENB, 0);
}

// ================= MAIN LOOP =================
void loop() {
  // If piston sequence is active, handle that first
  if (pistonSequenceActive) {
    executePistonSequence();
    delay(100);
    return;
  }
  
  // Check for slot detection if not in piston sequence
  if (checkSlotDetection()) {
    Serial.println("=== PROCESSING SLOT ===");
    // Start the piston sequence
    startPistonSequence();
  } else {
    // No slot detected - follow line normally
    followLinePID();
  }
  
  delay(10);
}