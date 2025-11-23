
# Semi-Autonomous Mobile Platform (SAMP)
**FOSSEE Screening Task 1 Submission**

## 🎥 Video Demonstration
### [Click Here to Watch the Project Demo on YouTube](https://youtu.be/VFBYiy64r78)


<img width="701" height="655" alt="image" src="https://github.com/user-attachments/assets/3be70df8-6deb-48da-ba4e-8ccda00e80d3" />


## Abstract
This project implements a differential-drive mobile robot designed with a focus on **operational safety** and **system reliability**. Unlike standard remote-controlled vehicles, the SAMP integrates a **priority-based control hierarchy**. The system operates in a semi-autonomous mode where sensor data (Obstacle Detection) and system health metrics (Connection Watchdog) take precedence over user input to prevent accidents and runaway behavior.

## Key Features
* **Hybrid Control Architecture:** Manual navigation via Bluetooth (UART) with autonomous safety overrides.
* **Active Collision Avoidance:** A non-blocking IR interrupt system instantly cuts motor power upon detecting an obstacle, regardless of user commands.
* **Connection Failsafe (Watchdog):** A software watchdog timer monitors the data stream. If signal loss exceeds 2000ms, the system enters a **"Failsafe Lockout"** state to prevent uncontrolled motion.
* **Real-Time Telemetry:** An I2C LCD interface provides live status updates on system state (e.g., "Moving", "Obstacle Detected", "Signal Lost").

# System Architecture
The system is built on the **ATmega328p (Arduino Uno)** microcontroller.
* **Power Topology:** Single-source 7.4V Li-ion array split into a high-current rail (L298N Driver) and a regulated logic rail (Arduino VIN).
* **Communication:** HC-05 Bluetooth module utilizing SoftwareSerial (Pins 12/13) to keep hardware UART open for debugging.
* **Actuation:** L298N H-Bridge driving two DC Gear Motors in a differential configuration.

## Circuit Design
The circuit schematic was designed and validated using **FOSSEE's eSim software**.

<img width="1372" height="945" alt="image" src="https://github.com/user-attachments/assets/e2f0dadb-abb8-4f1b-9200-050746c7b958" />


*The full project file and PDF export are available in the `/schematics` folder of this repository.*

## Software Logic
The firmware implements a continuous loop with the following priority logic:
1.  **Check Sensors:** Is the path blocked? (If YES $\rightarrow$ **Emergency Stop**).
2.  **Check Watchdog:** Has the connection timed out? (If YES $\rightarrow$ **Failsafe Lockout**).
3.  **Check User Input:** Is there a valid Bluetooth command? (If YES $\rightarrow$ **Execute Motion**).

## Components List
1.  Arduino Uno R3
2.  L298N Motor Driver Module
3.  HC-05 Bluetooth Module
4.  I2C LCD Display (16x2)
5.  IR Proximity Sensor
6.  Li-ion Battery Pack (7.4V)
7.  Chassis & DC Gear Motors

#🔮 Future Scope
* **Closed-Loop Control:** Implementation of rotary encoders for PID speed stabilization.
* **PCB Migration:** Transitioning from breadboard prototype to a custom PCB designed in eSim.
* **Mapping:** Integration of ROS (Robot Operating System) for SLAM capabilities.

Challenges Faced:
Power Brownouts: Initially, the Arduino reset when motors accelerated. I resolved this by isolating the power rails (7.4V for motors, Regulated 5V for logic) and adding a common ground.

Motor Inversion: The left motor logic was physically inverted relative to the chassis orientation. I implemented a software fix in the moveForward() function rather than rewiring the hardware.

Signal Noise: The IR sensor triggered false positives. I resolved this by tuning the potentiometer sensitivity and adding a software debounce delay.

---
**Author:** Shivani Sarangi
**Date:** November 2025
**License:** MIT License
