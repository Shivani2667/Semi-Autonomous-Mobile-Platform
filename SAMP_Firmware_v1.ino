/*
 * Project: Semi-Autonomous Mobile Platform (SAMP) v1.0
 * Description: Firmware for FOSSEE Task 1 Submission.
 * * Overview:
 * This firmware controls a differential-drive robot with a focus on safety.
 * It integrates manual Bluetooth control with an autonomous safety override system.
 * The logic prioritizes sensor data (IR obstacle detection) over user input to prevent collisions.
 * Additionally, a software watchdog ensures the robot stops if the signal is lost.
 *
 * Hardware Config:
 * - Controller: Arduino Uno (ATmega328p)
 * - Driver: L298N H-Bridge
 * - Communication: HC-05 Bluetooth Module (SoftwareSerial)
 * - Sensors: IR Proximity Sensor (Active LOW)
 * - Display: I2C LCD (16x2)
 */

#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <SoftwareSerial.h>

// --- PIN DEFINITIONS ---

// Bluetooth Communication
// Using SoftwareSerial on Pins 12 & 13 to keep the hardware serial port (0 & 1) free for USB debugging.
// This avoids the need to unplug the Bluetooth module every time we upload code.
SoftwareSerial myBluetooth(12, 13); // RX, TX

// Motor Driver Connections (L298N)
// Note: During assembly, the Left Motor wiring resulted in inverted rotation.
// Instead of re-soldering, I handled this inversion in the software logic below.
const int LeftMotor_Pin1 = 2;
const int LeftMotor_Pin2 = 3;
const int RightMotor_Pin1 = 5;
const int RightMotor_Pin2 = 6;

// IR Sensor Input
// The sensor output is Active LOW (0V when obstacle detected).
const int irSensorPin = 8;

// Watchdog Safety Timer
// If no command is received for 2000ms, we assume connection loss and auto-brake.
const long WATCHDOG_TIMEOUT = 2000; 
unsigned long lastCommandTime = 0; 
bool isLost = false; // Flag to track if we are currently in Failsafe mode

// LCD Setup
// Standard I2C address is 0x27. 
LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup() {
  // Configure Motor Pins as Outputs
  pinMode(LeftMotor_Pin1, OUTPUT);
  pinMode(LeftMotor_Pin2, OUTPUT);
  pinMode(RightMotor_Pin1, OUTPUT);
  pinMode(RightMotor_Pin2, OUTPUT);
  
  // Configure Sensor Pin
  pinMode(irSensorPin, INPUT);

  // Initialize Communications
  Serial.begin(9600);      // For monitoring via laptop
  myBluetooth.begin(9600); // For receiving app commands

  // Initialize LCD Interface
  lcd.init();       
  lcd.backlight();  
  
  // Display Startup Sequence
  // Provides visual confirmation that the system has booted successfully.
  updateScreen("System Booting...", "Sensors Active");
  delay(1000);
  updateScreen("Mode: Bluetooth", "Waiting for App");
  
  // Start the watchdog timer
  lastCommandTime = millis();
}

void loop() {
  // --- SAFETY LAYER 1: OBSTACLE AVOIDANCE ---
  // Priority: HIGH. This check happens before reading any bluetooth commands.
  // If an obstacle is detected (LOW signal), we force a stop immediately.
  if (digitalRead(irSensorPin) == LOW) {
    stopRobot();
    updateScreen("WARNING!", "OBSTACLE DETECTED");
    
    // Reset watchdog so we don't trigger a "Signal Lost" error while waiting for the obstacle to move.
    lastCommandTime = millis(); 
    delay(200); // Small delay to prevent jitter
    return; // Exit loop early to ignore any movement commands
  }

  // --- SAFETY LAYER 2: CONNECTION WATCHDOG ---
  // Priority: MEDIUM. Checks if the controller has disconnected.
  if (millis() - lastCommandTime > WATCHDOG_TIMEOUT) {
    
    // ACTION: Emergency Stop & Lockout
    stopRobot();
    
    // Update screen only once to prevent flickering
    if (!isLost) {
      lcd.clear();
      lcd.setCursor(0,0); lcd.print("!! FAILSAFE !!");
      lcd.setCursor(0,1); lcd.print("BT SIGNAL LOST");
      isLost = true; // Lock the system state
    }
  }

  // --- CONTROL LAYER: BLUETOOTH INPUT ---
  // Only runs if safety checks pass.
  if (myBluetooth.available()) {
    char command = myBluetooth.read(); 
    
    // If we were in Failsafe mode, notify user that we are back online
    if (isLost) {
      lcd.clear();
      lcd.setCursor(0,0); lcd.print("RECONNECTED");
      lcd.setCursor(0,1); lcd.print("System Active");
      delay(500); 
      isLost = false; // Unlock the system
    }

    // Reset Watchdog Timer (Heartbeat)
    lastCommandTime = millis(); 
    
    // Execute Motion Command
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

// --- KINEMATICS CONTROL FUNCTIONS ---
// These functions abstract the low-level pin writing for cleaner logic above.

void moveForward() {
  // Right Motor: Standard Polarity (LOW/HIGH)
  digitalWrite(RightMotor_Pin1, LOW);
  digitalWrite(RightMotor_Pin2, HIGH);
  
  // Left Motor: Inverted Polarity (HIGH/LOW) due to wiring orientation
  digitalWrite(LeftMotor_Pin1, HIGH);
  digitalWrite(LeftMotor_Pin2, LOW);
}

void moveBackward() {
  // Right Motor: Standard Polarity (HIGH/LOW)
  digitalWrite(RightMotor_Pin1, HIGH);
  digitalWrite(RightMotor_Pin2, LOW);
  
  // Left Motor: Inverted Polarity (LOW/HIGH)
  digitalWrite(LeftMotor_Pin1, LOW);
  digitalWrite(LeftMotor_Pin2, HIGH);
}

void turnLeft() {
  // Differential Drive: Right FWD, Left BACK
  digitalWrite(RightMotor_Pin1, LOW);
  digitalWrite(RightMotor_Pin2, HIGH);
  digitalWrite(LeftMotor_Pin1, LOW);
  digitalWrite(LeftMotor_Pin2, HIGH);
}

void turnRight() {
  // Differential Drive: Right BACK, Left FWD
  digitalWrite(RightMotor_Pin1, HIGH);
  digitalWrite(RightMotor_Pin2, LOW);
  digitalWrite(LeftMotor_Pin1, HIGH);
  digitalWrite(LeftMotor_Pin2, LOW);
}

void stopRobot() {
  // Force all pins LOW to coast to a stop
  digitalWrite(RightMotor_Pin1, LOW);
  digitalWrite(RightMotor_Pin2, LOW);
  digitalWrite(LeftMotor_Pin1, LOW);
  digitalWrite(LeftMotor_Pin2, LOW);
}

// Helper function to manage LCD updates cleanly
void updateScreen(String line1, String line2) {
  lcd.clear();
  lcd.setCursor(0,0); lcd.print(line1);
  lcd.setCursor(0,1); lcd.print(line2);
}
