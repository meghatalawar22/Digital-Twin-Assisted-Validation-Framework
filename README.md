# Digital Twin Assisted Validation Framework for Autonomous Vehicles

## 1. Project Overview

This project focuses on developing a **Digital Twin Assisted Validation Framework** using an Arduino-based autonomous vehicle prototype.

The system consists of a physical robotic vehicle controlled by an Arduino Mega 2560 and a MATLAB-based Digital Twin that replicates the vehicle's control logic.

The physical vehicle uses IR sensors for lane detection and an ultrasonic sensor for obstacle detection. Based on sensor readings, the Arduino controls the motors, buzzer, and vehicle movement.

The Arduino transmits real-time telemetry data to MATLAB through an HC-05 Bluetooth module. MATLAB records the hardware data, simulates the corresponding Digital Twin behaviour, and compares both results to validate the control logic.

The objective is to evaluate the consistency between the physical hardware and its digital representation.

## 2. Problem Statement

Testing autonomous vehicle control systems requires evaluating their behaviour under different driving conditions, including lane detection, obstacle detection, stopping, and restarting.

Testing only on physical hardware can make it difficult to repeatedly analyse internal control decisions and compare expected behaviour with actual hardware responses.

Therefore, this project proposes a Digital Twin Assisted Validation Framework in which a MATLAB-based digital representation models the Arduino control logic and compares its expected outputs with actual hardware telemetry.

This approach helps identify mismatches in control decisions, motor commands, obstacle response, and stopping behaviour.

## 3. Objectives

- To develop an Arduino Mega 2560-based autonomous vehicle prototype.
- To implement lane-following functionality using two IR sensors.
- To detect obstacles using an HC-SR04 ultrasonic sensor.
- To implement automatic stopping and restarting based on obstacle distance.
- To provide buzzer-based obstacle warnings.
- To transmit real-time hardware telemetry through Bluetooth.
- To develop a MATLAB-based Digital Twin that replicates the Arduino control logic.
- To compare hardware behaviour with Digital Twin predictions.
- To analyse mismatches and generate validation results and plots.

## 4. System Architecture

The proposed system consists of two main parts:

### A. Physical Hardware System

The Arduino Mega 2560 receives input from the IR sensors and ultrasonic sensor.

Based on these readings, it determines the vehicle's movement and controls the motors through the L298N motor driver.

The system also controls the buzzer and maintains the obstacle-stop and restart states.

The HC-05 Bluetooth module transmits the vehicle's telemetry data to MATLAB.

### B. Digital Twin and Validation System

MATLAB is used to develop the Digital Twin and validation framework.

The Digital Twin reproduces the Arduino control decisions using the same sensor inputs and control parameters.

A separate MATLAB logging program receives actual hardware telemetry through Bluetooth and stores it in a MAT file.

The validation program compares the hardware records with the Digital Twin's predicted outputs and calculates matching percentages.

## 5. Hardware Components

| Component | Purpose |
|---|---|
| Arduino Mega 2560 | Main controller |
| IR Sensor (Left) | Left-side lane detection |
| IR Sensor (Right) | Right-side lane detection |
| HC-SR04 Ultrasonic Sensor | Front obstacle detection |
| L298N Motor Driver | Controls DC motors |
| Four DC Motors | Vehicle movement |
| HC-05 Bluetooth Module | Wireless telemetry transmission |
| Buzzer | Obstacle warning |
| SG90 Servo Motor | Holds the ultrasonic sensor facing forward |
| 12V Batteries | Power supply |
| Buck Converter | Voltage regulation |

## 6. Software and Tools

- **Arduino IDE:** Arduino firmware development and uploading.
- **MATLAB:** Digital Twin simulation, hardware data logging, validation, and plotting.
- **C/C++:** Arduino programming.
- **Bluetooth Serial Communication:** Transfers hardware telemetry to MATLAB.
- **GitHub:** Source code management and project documentation.

## 7. Arduino Pin Connections

The project uses an Arduino Mega 2560.

| Component / Signal | Arduino Mega Pin |
|---|---|
| Left IR Sensor OUT | A3 |
| Right IR Sensor OUT | A2 |
| Ultrasonic TRIG | D3 |
| Ultrasonic ECHO | D2 |
| Buzzer | D4 |
| Servo Signal | D5 |
| L298N ENA | D11 |
| L298N IN1 | D10 |
| L298N IN2 | D9 |
| L298N IN3 | D8 |
| L298N IN4 | D7 |
| L298N ENB | D6 |
| HC-05 TXD | D19 (RX1) |
| HC-05 RXD | D18 (TX1), through voltage divider |

**Power connections:**
- A regulated 9V supply is connected to the Arduino VIN.
- A separate 12V supply powers the motors through the L298N motor driver.
- All grounds are connected together.
- A voltage divider is used between Arduino TX1 and HC-05 RXD.

## 8. Working Methodology

### Step 1: Sensor Data Acquisition

The two IR sensors detect the lane markings, while the HC-SR04 ultrasonic sensor measures the distance to an obstacle.

### Step 2: Arduino-Based Vehicle Control

The Arduino processes sensor readings and selects the appropriate movement:

- Forward movement.
- Right turn.
- Left turn.
- Stop when both IR sensors detect black.
- Stop when an obstacle is detected.

### Step 3: Obstacle Detection and Restart

The vehicle stops when the obstacle distance is less than or equal to 20 cm.

The buzzer provides warnings when an obstacle enters the warning zone.

The vehicle restarts only after the path remains clear for six consecutive readings above 35 cm.

A short kick-start command is used to restart the motors. The ultrasonic sensor checks for an obstacle during this movement.

### Step 4: Bluetooth Telemetry

The Arduino transmits information through the HC-05 Bluetooth module.

The telemetry contains:

- Sequence number.
- Time in milliseconds.
- Left and right IR readings.
- Ultrasonic distance.
- Previous stop state.
- Previous clear counter.
- Control mode.
- Left and right motor commands.
- Buzzer state.
- Current stop state.
- Current clear counter.

### Step 5: MATLAB Digital Twin

MATLAB implements the same control decision logic as the Arduino sketch.

It processes the sensor readings and predicts:

- Vehicle control mode.
- Motor PWM commands.
- Stop-latch state.
- Clear counter.
- Kick-start decision.

### Step 6: Hardware Data Logging

MATLAB receives Bluetooth telemetry and stores the collected records in a scenario-specific MAT file.

Example:

`hw_run_1.mat`

### Step 7: Hardware vs Digital Twin Validation

The validation script loads the recorded hardware data and compares it with the Digital Twin predictions.

It evaluates:

- Control mode matching.
- Motor PWM matching.
- Stop-latch matching.
- Clear-counter matching.
- Buzzer matching.
- Overall record matching.

The MATLAB program also generates comparison plots and identifies the first mismatching records.

## 9. Control Logic and Thresholds

| Parameter | Value |
|---|---:|
| Normal motor speed A | 60 PWM |
| Normal motor speed B | 60 PWM |
| Turning speed | 90 PWM |
| Kick-start speed | 130 PWM |
| Kick-start duration | 150 ms |
| Obstacle stop distance | 20 cm |
| Warning distance | 30 cm |
| Restart distance | Above 35 cm |
| Consecutive clear readings | 6 |
| Slow buzzer interval | 600 ms |
| Fast buzzer interval | 100 ms |
| Buzzer ON duration | 60 ms |

## 10. Control Modes

| Mode | Description |
|---:|---|
| 0 | Forward |
| 1 | Turn Right |
| 2 | Turn Left |
| 3 | Both IR Sensors Detect Black - Stop |
| 4 | Obstacle Stop |
| 5 | Latched Stop |
| 6 | Kick Start Successful |
| 7 | Kick Start Aborted |

## 11. Test Scenarios

The MATLAB Digital Twin supports four scenarios:

| Scenario | Description |
|---|---|
| Scenario 1 | No obstacle and normal lane following |
| Scenario 2 | Obstacle is removed after 14 seconds |
| Scenario 3 | Obstacle remains in front of the vehicle |
| Scenario 4 | IR sensor truth-table demonstration |

The hardware validation program can record four corresponding test runs for comparison.

## 12. Validation Metrics

The MATLAB validation program calculates the following metrics:

1. **Mode Match:** Percentage of records where hardware and Digital Twin control modes agree.
2. **PWM Match:** Percentage of records where motor PWM commands agree.
3. **Stop-Latch Match:** Agreement between hardware and Digital Twin stopping states.
4. **Clear-Counter Match:** Agreement between the restart counter values.
5. **Buzzer Match:** Agreement in buzzer behaviour, accounting for timing variations in the warning zone.
6. **Overall Match:** Percentage of records satisfying all five validation checks.

The validation program also calculates the hardware loop period, obstacle stopping distance, and restart delay.

## 13. Repository Structure

```text
Digital-Twin-Autonomous-Vehicle/
│
├── README.md
│
├── Arduino/
│   └── autonomous_vehicle.ino
│
├── MATLAB/
│   ├── digital_twin.m
│   ├── bluetooth_logger.m
│   ├── validation.m
│   └── twinController.m
│
├── Hardware/
│   └── wiring_diagram.png
│
├── Results/
│   ├── plots/
│   └── validation_reports/
│
└── Documentation/
    └── project_details.md
```

*Note: The filenames and folder structure above are suggested. Adjust them to match the actual files uploaded to GitHub.*

## 14. Current Project Progress

- Arduino Mega 2560 hardware architecture identified.
- Hardware wiring and component connections prepared.
- Arduino firmware developed for lane following and obstacle detection.
- Automatic obstacle stopping and restart logic implemented.
- Bluetooth telemetry transmission implemented using HC-05.
- MATLAB-based Digital Twin control logic developed.
- MATLAB Bluetooth data logging program prepared.
- Hardware and Digital Twin comparison program developed.
- Validation metrics and graphical comparison implemented.
- Physical hardware testing and validation results are being completed/documented.

## 15. Expected Outcome

The expected outcome is a working Arduino-based autonomous vehicle prototype with a MATLAB Digital Twin-assisted validation system.

The framework will record real hardware behaviour, reproduce the corresponding control decisions in MATLAB, compare both outputs, identify mismatches, and provide quantitative validation results.

This demonstrates how a Digital Twin can support the testing and validation of autonomous vehicle control systems.

## 16. Project Status



