/*
 * Project: Semi-Autonomous Mobile Platform (SAMP) v1.0
 * Description: FOSSEE Task 1 Submission.
 * Features: 
 * - Differential Drive Control via Bluetooth (UART).
 * - Real-time Obstacle Avoidance (IR Interrupt).
 * - Signal Loss Failsafe (Software Watchdog with Lockout).
 */

#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>

// --- PIN DEFINITIONS ---
SoftwareSerial myBluetooth(12, 13); // RX, TX

const int LeftMotor_Pin1 = 2;
const int LeftMotor_Pin2 = 3;
const int RightMotor_Pin1 = 5;
const int RightMotor_Pin2 = 6;
const int irSensorPin = 8;

// Watchdog Settings
const long WATCHDOG_TIMEOUT = 2000; // 2 Seconds
unsigned long lastCommandTime = 0; 
bool isLost = false; // Failsafe State Flag

// LCD Setup (Address 0x27 or 0x3F)
LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup() {
  pinMode(LeftMotor_Pin1, OUTPUT);
  pinMode(LeftMotor_Pin2, OUTPUT);
  pinMode(RightMotor_Pin1, OUTPUT);
  pinMode(RightMotor_Pin2, OUTPUT);
  pinMode(irSensorPin, INPUT);

  Serial.begin(9600);
  myBluetooth.begin(9600);

  lcd.init();       
  lcd.backlight();  
  
  updateScreen("System Booting...", "Sensors Active");
  delay(1000);
  updateScreen("Mode: Bluetooth", "Waiting for App");
  
  lastCommandTime = millis();
}

void loop() {
  // --- 1. SAFETY CHECK (IR SENSOR) ---
  // Active LOW check (Standard for IR)
  if (digitalRead(irSensorPin) == LOW) {
    stopRobot();
    updateScreen("WARNING!", "OBSTACLE DETECTED");
    lastCommandTime = millis(); // Keep watchdog alive while blocked
    delay(200); 
    return; // Block movement
  }

  // --- 2. WATCHDOG & FAILSAFE LOGIC ---
  // If no command received for 2 seconds...
  if (millis() - lastCommandTime > WATCHDOG_TIMEOUT) {
    
    // ACTION: Hard Stop & Lockout
    stopRobot();
    
    // Only update screen ONCE to prevent flickering
    if (!isLost) {
      lcd.clear();
      lcd.setCursor(0,0); lcd.print("!! FAILSAFE !!");
      lcd.setCursor(0,1); lcd.print("BT SIGNAL LOST");
      isLost = true; // Enter Failsafe Mode
    }
  }

  // --- 3. BLUETOOTH COMMANDS ---
  if (myBluetooth.available()) {
    char command = myBluetooth.read(); 
    
    // If we were in Failsafe, wake up!
    if (isLost) {
      lcd.clear();
      lcd.setCursor(0,0); lcd.print("RECONNECTED");
      lcd.setCursor(0,1); lcd.print("System Active");
      delay(500); 
      isLost = false; // Exit Failsafe Mode
    }

    // Reset Timer
    lastCommandTime = millis(); 
    
    // Execute Command
    if (command == 'F') {
      moveForward();
      updateScreen("Status:", "Moving Forward");
    }
    else if (command == 'B') {
      moveBackward();
      updateScreen("Status:", "Moving Backward");
    }
    else if (command == 'L') {
      turnLeft();
      updateScreen("Status:", "Turning Left");
    }
    else if (command == 'R') {
      turnRight();
      updateScreen("Status:", "Turning Right");
    }
    else if (command == 'S') {
      stopRobot();
      updateScreen("Status:", "Stopped");
    }
  }
}

// --- MOVEMENT FUNCTIONS (LOGIC PRESERVED) ---

void moveForward() {
  // Right Motor Forward
  digitalWrite(RightMotor_Pin1, LOW);
  digitalWrite(RightMotor_Pin2, HIGH);
  // Left Motor Forward
  digitalWrite(LeftMotor_Pin1, HIGH);
  digitalWrite(LeftMotor_Pin2, LOW);
}

void moveBackward() {
  // Right Motor Backward
  digitalWrite(RightMotor_Pin1, HIGH);
  digitalWrite(RightMotor_Pin2, LOW);
  // Left Motor Backward
  digitalWrite(LeftMotor_Pin1, LOW);
  digitalWrite(LeftMotor_Pin2, HIGH);
}

void turnLeft() {
  // Left FWD, Right BACK
  digitalWrite(LeftMotor_Pin1, HIGH);
  digitalWrite(LeftMotor_Pin2, LOW);
  digitalWrite(RightMotor_Pin1, HIGH);
  digitalWrite(RightMotor_Pin2, LOW);
}

void turnRight() {
  // Right FWD, Left BACK
  digitalWrite(RightMotor_Pin1, LOW);
  digitalWrite(RightMotor_Pin2, HIGH);
  digitalWrite(LeftMotor_Pin1, LOW);
  digitalWrite(LeftMotor_Pin2, HIGH);
}

void stopRobot() {
  digitalWrite(RightMotor_Pin1, LOW);
  digitalWrite(RightMotor_Pin2, LOW);
  digitalWrite(LeftMotor_Pin1, LOW);
  digitalWrite(LeftMotor_Pin2, LOW);
}

void updateScreen(String line1, String line2) {
  lcd.clear();
  lcd.setCursor(0,0); lcd.print(line1);
  lcd.setCursor(0,1); lcd.print(line2);
}
