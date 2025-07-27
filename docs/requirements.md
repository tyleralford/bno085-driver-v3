Of course. Here is the comprehensive Product Requirements Document (PRD) compiled from our detailed discussion.

---

# **PRD: BNO085 micro-ROS IMU Publisher**

| | |
|---|---|
| **Project Title** | BNO085 micro-ROS IMU Publisher |
| **Target ROS 2 Distro** | Jazzy Jalisco |
| **Version** | 1.0 |

## **1. Project Overview**

This document outlines the product and technical requirements for an embedded firmware application that turns a Raspberry Pi Pico and an Adafruit BNO085 IMU into a robust, high-performance ROS 2 sensor node.

The primary objective is to provide a reliable stream of IMU data for robot localization algorithms. The system is designed to be tightly integrated with the ROS 2 ecosystem, featuring remote configuration, quality-of-service reporting through dynamic covariance, and robust error handling. The firmware will be developed within the PlatformIO framework for ease of building and deployment.

## **2. Core Requirements**

### **Data & Publishing**
*   **REQ-01:** The system must acquire the Game Rotation Vector, Gyroscope, and Accelerometer data from the BNO085 sensor.
*   **REQ-02:** All three data points must be consolidated and published as a single `sensor_msgs/Imu` message on the `/imu/data` topic.
*   **REQ-03:** The default publishing rate is **60Hz** (optimized for BNO085 sensor data availability and performance).

### **Time Management**
*   **REQ-04:** All published messages must be timestamped using the synchronized ROS 2 host time, with an accuracy of tens of milliseconds or better.
*   **REQ-05:** The firmware must halt data publishing if time synchronization with the host has not been established or has been lost.
*   **REQ-06:** The system will attempt to re-synchronize time every **5 seconds**.
*   **REQ-07:** Time synchronization is considered "lost" after **3 consecutive** failed re-sync attempts.

### **Configuration**
*   **REQ-08:** The system must be configurable remotely via the ROS 2 Parameter Server. This includes data rate, time-sync policies, and all covariance values.
*   **REQ-09:** In the event that ROS parameters cannot be fetched on startup, the system will load a set of safe, pre-compiled default values and issue a warning to the ROS console.

### **Data Quality & Covariance**
*   **REQ-10:** The firmware must continuously monitor the BNO085's internal calibration status (levels 0-3).
*   **REQ-11:** The `covariance` matrices in the published `sensor_msgs/Imu` message must be dynamically scaled based on the real-time calibration status to reflect data quality.

### **Hardware & Physical**
*   **REQ-12:** The firmware must provide clear operational status via onboard LEDs (Blue, Green, Red).
*   **REQ-13:** The firmware assumes the BNO085 sensor is mounted according to the ROS REP-103 standard (X-forward, Y-left, Z-up).

## **3. Core Features**

*   **High-Frequency Data Streaming:** Provides a 200Hz stream of orientation, angular velocity, and linear acceleration data for demanding localization tasks.
*   **ROS 2 Time Synchronization:** Ensures all data is correctly timestamped within the broader ROS ecosystem, critical for sensor fusion.
*   **Dynamic Covariance:** Intelligently adjusts the reported data uncertainty based on the IMU's real-time calibration status, improving the performance of downstream localization filters (e.g., EKF).
*   **Persistent Calibration:** Allows the user to perform a one-time calibration and save the results, eliminating the need to re-calibrate on every power cycle.
*   **Remote Configuration:** Enables tuning of all key performance parameters (rate, timing, covariance) from a host-side launch file without re-flashing the firmware.
*   **Visual Status Indicators:** Provides at-a-glance diagnostics via multi-colored LEDs, simplifying debugging and operational checks.
*   **Robust State Management:** Gracefully handles common failure modes such as loss of communication with the ROS host or sensor.

## **4. Core Components**

### **Hardware**
*   **Microcontroller:** Raspberry Pi Pico (RP2040)
*   **IMU Sensor:** Adafruit BNO085 IMU Sensor Breakout
*   **Status LEDs:**
    *   Blue: GPIO25
    *   Green: GPIO16
    *   Red: GPIO17
*   **I2C Connection:** Pico `i2c1` port to BNO085 at address `0x4A`.

### **Software**
*   **Firmware:** Custom C++ application.
*   **ROS Communication:** `micro-ROS` library for Pico.
*   **Sensor Interface:** `SparkFun BNO08x` Arduino Library.

## **5. App/User Flow**

### **State Machine & LED Indicators**

| State                 | Condition                                                              | LED Indicator                             | Actions                                                                                               |
| --------------------- | ---------------------------------------------------------------------- | ----------------------------------------- | ----------------------------------------------------------------------------------------------------- |
| **Initializing**      | Power-on until sensor and ROS parameters are configured.               | **Blue LED** (slow pulse)                 | Init I2C. Connect to BNO085. Load calibration from flash. Fetch ROS parameters.                       |
| **Awaiting Time Sync**| Initialization complete, but no valid time from ROS host. Also on re-sync. | **Blue LED** (rapid blink)                | Periodically request time from the ROS agent. Halt all data publishing.                               |
| **Operational**       | Time is synchronized, and the sensor is providing data.                | **Solid Green LED**                       | Read sensor, check calibration, update covariance, and publish `sensor_msgs/Imu` message at 200Hz. |
| **Critical Fault**    | Lost I2C communication with the BNO085 sensor.                         | **Solid Red LED**                         | Halt all publishing and processing. Attempt to reconnect to the sensor periodically.                  |

### **User Interactions**

1.  **Tuning:** The user can modify a `YAML` or launch file on the ROS 2 host computer to adjust parameters like data rate or baseline covariance. These changes are applied on the next startup of the Pico firmware.
2.  **Calibration:** The user moves the robot/sensor to achieve full calibration (indicated by the `dynamic_covariance` logic settling on low values). The sensor will automatically maintain its calibration during operation.

## **6. Tech Stack**

| Category          | Technology / Specification           |
| ----------------- | ------------------------------------ |
| **Build System**  | PlatformIO                           |
| **Firmware Language** | C++ (Arduino Framework)              |
| **ROS Middleware**| micro-ROS for Raspberry Pi Pico      |
| **ROS 2 Target**  | Jazzy Jalisco                        |
| **Host OS**       | Linux (Ubuntu 24.04 recommended)     |
| **Libraries**     | `micro_ros_platformio`, `SparkFun BNO08x` |

## **7. Implementation Plan**

This project will be implemented in logical, iterative steps.

1.  **Phase 1: Hardware Bring-up & Validation**
    *   Task: Establish basic I2C communication between the Pico and the BNO085.
    *   Goal: Use the SparkFun library to read the accelerometer, gyroscope, and game rotation vector and print the raw values to the USB serial console. This validates the hardware connections.

2.  **Phase 2: Basic micro-ROS Integration**
    *   Task: Set up a minimal PlatformIO project with micro-ROS.
    *   Goal: Create a simple publisher that sends a static `sensor_msgs/Imu` message to the ROS host. This validates the entire communication toolchain.

3.  **Phase 3: Core Publisher Implementation**
    *   Task: Merge the logic from Phase 1 and 2.
    *   Goal: Read sensor data in the main loop, populate the `sensor_msgs/Imu` message, and publish it at the target 200Hz rate.

4.  **Phase 4: Time Synchronization & State Management**
    *   Task: Implement the micro-ROS time synchronization client. Build the state machine (`Initializing`, `Awaiting Time Sync`, `Operational`).
    *   Goal: Messages are correctly timestamped, and publishing is halted until sync is achieved.

5.  **Phase 5: Status Indicators & Debugging**
    *   Task: Implement the LED control logic.
    *   Goal: The LEDs correctly reflect the current state of the application as defined in the App/User Flow.

6.  **Phase 6: Remote Configuration**
    *   Task: Add the ROS parameter client logic. Define the internal default values.
    *   Goal: The firmware can successfully fetch parameters on startup and correctly falls back to defaults if the host is unavailable.

7.  **Phase 7: Dynamic Covariance & Calibration Logic**
    *   Task: Implement the logic to query the sensor's calibration status and apply the scaling factors to the covariance matrices before publishing.
    *   Goal: The published covariance values change in real-time based on sensor movement and calibration quality.

8.  **Phase 8: Final Integration & Testing**
    *   Task: Conduct end-to-end testing of all features.
    *   Goal: Verify robustness by simulating fault conditions (e.g., disconnecting the sensor, stopping the ROS agent mid-operation) and confirm all requirements are met.
