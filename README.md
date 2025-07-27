# BNO085 micro-ROS IMU Driver

A micro-ROS driver for the BNO085 9-DOF IMU sensor, designed for the Raspberry Pi Pico. This driver provides IMU data publishing to ROS 2 with dynamic covariance scaling based on sensor calibration status.

## Hardware Requirements

### Microcontroller
- **Raspberry Pi Pico** (RP2040-based)
- USB connection to host computer

### IMU Sensor
- **Adafruit BNO085 9-DOF IMU** breakout board
- I2C interface (default address: 0x4A)

### Wiring
| BNO085 Pin | Raspberry Pi Pico Pin | Description |
|------------|----------------------|-------------|
| VIN        | 3V3 (Pin 36)        | Power supply |
| GND        | GND (Pin 38)        | Ground |
| SDA        | GPIO 6 (Pin 9)      | I2C Data |
| SCL        | GPIO 7 (Pin 10)     | I2C Clock |

### Status LEDs (Optional)
| LED Color | Raspberry Pi Pico Pin | State Indication |
|-----------|----------------------|------------------|
| Blue      | GPIO 25 (Pin 30)    | Initializing/Awaiting Sync |
| Green     | GPIO 16 (Pin 21)    | Operational |
| Red       | GPIO 17 (Pin 22)    | Critical Fault |

## Software Requirements

### Development Environment
- **PlatformIO IDE** (VS Code extension)
- **Python 3.8+** for ROS 2 agent
- **ROS 2 Jazzy** (or compatible distribution)

### Dependencies
- Adafruit BNO08x library (v1.2.5)
- micro-ROS for PlatformIO
- ROS 2 micro-ROS agent

## Installation and Setup

### Prerequisites
- **ROS 2 Jazzy** (or compatible distribution)
- **PlatformIO IDE** for firmware development
- **Raspberry Pi Pico** with BNO085 IMU sensor

### 1. Install as ROS 2 Package

#### From Source (Recommended)
```bash
# Create a ROS 2 workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone the repository
git clone https://github.com/tyleralford/bno085-driver-v3.git bno085_driver

# Install dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build the package
colcon build --packages-select bno085_driver

# Source the workspace
source install/setup.bash
```

#### Using Quick Start Script
```bash
cd ~/ros2_ws/src/bno085_driver
chmod +x quick_start.sh
./quick_start.sh
```

### 2. Hardware Setup
Connect your BNO085 sensor to the Raspberry Pi Pico according to the wiring table above.

### 3. Build and Upload Firmware
```bash
# Navigate to the firmware directory
cd ~/ros2_ws/src/bno085_driver

# Build the project
pio run

# Upload to Raspberry Pi Pico
# Put Pico in bootloader mode (hold BOOTSEL while connecting USB)
pio run --target upload

# Optional: Monitor serial output
pio device monitor --baud 115200
```

# Install micro-ROS agent
https://micro.ros.org/docs/tutorials/core/first_application_linux/
```bash
mkdir -p ~/microros_ws/src
cd ~/microros_ws/src
git clone -b jazzy https://github.com/micro-ROS/micro_ros_setup.git
cd ~/microros_ws
colcon build
source install/setup.bash
```

## Usage

### 1. Launch the Complete System
The package provides several launch files for different use cases:

#### Full System Launch (Recommended)
```bash
# Launch everything: micro-ROS agent, transforms, and visualization
ros2 launch bno085_driver bno085_complete.launch.py

# With custom device path
ros2 launch bno085_driver bno085_complete.launch.py device:=/dev/ttyACM0

# Without RViz
ros2 launch bno085_driver bno085_complete.launch.py start_rviz:=false

# With real-time plotting
ros2 launch bno085_driver bno085_complete.launch.py start_rqt_plot:=true
```

#### Agent Only
```bash
# Start just the micro-ROS agent
ros2 launch bno085_driver bno085_agent.launch.py device:=/dev/ttyACM0
```

### 2. Monitor IMU Data
View the IMU data directly using ROS 2 tools:

```bash
# View IMU data directly
ros2 topic echo /imu/data

# Check publishing rate
ros2 topic hz /imu/data
```

### 3. Verify System Operation
```bash
# List active topics
ros2 topic list

# Check IMU data publishing rate
ros2 topic hz /imu/data

# View calibration status (reflected in covariance values)
ros2 topic echo /imu/data --field orientation_covariance

# Monitor transforms
ros2 run tf2_tools view_frames.py
```

## System States and LED Indicators

The driver implements a robust state machine with visual feedback:

| State | LED Indicator | Description |
|-------|---------------|-------------|
| **INITIALIZING** | Blue rapid blink (5Hz) | System startup, sensor initialization |
| **AWAITING_SYNC** | Blue slow pulse (1Hz) | Waiting for time synchronization with ROS 2 agent |
| **OPERATIONAL** | Solid Green | Normal operation, publishing IMU data |
| **CRITICAL_FAULT** | Solid Red | Sensor or communication failure, attempting recovery |

## Published Topics

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/imu/data` | `sensor_msgs/Imu` | 60Hz | Complete IMU data with dynamic covariance |

### Message Fields
- **Header**: ROS 2 timestamp and frame ID (`imu_link`)
- **Orientation**: Quaternion from Game Rotation Vector (magnetometer-free)
- **Angular Velocity**: Calibrated gyroscope data (rad/s)
- **Linear Acceleration**: Accelerometer data (m/s²)
- **Covariance Matrices**: Dynamic scaling based on individual sensor calibration status

## Troubleshooting

### ROS 2 Debugging
```bash
# Check node status
ros2 node list
ros2 node info /micro_ros_pico_node

# Monitor topic performance  
ros2 topic hz /imu/data
ros2 topic bw /imu/data

# View detailed message content
ros2 topic echo /imu/data --once
```

## Configuration

The driver uses compile-time configuration parameters that can be modified in `main.cpp`:

### Publishing Parameters
```cpp
config.publish_rate_hz = 60.0;          // Publishing frequency
config.sync_timeout_ms = 5000;          // Time sync timeout
config.sync_retry_attempts = 3;         // Sync retry attempts
```

### Covariance Configuration
```cpp
config.orientation_covariance_base = 0.05;        // Base orientation uncertainty
config.angular_velocity_covariance_base = 0.01;   // Base gyro uncertainty  
config.linear_acceleration_covariance_base = 0.05; // Base accel uncertainty

// Scaling factors for calibration levels 0-3
config.covariance_scale_factors[0] = 10.0;  // Uncalibrated
config.covariance_scale_factors[1] = 5.0;   // Poor
config.covariance_scale_factors[2] = 2.0;   // Good
config.covariance_scale_factors[3] = 1.0;   // Excellent
```

## License

This project is licensed under the MIT License - see the LICENSE file for details.
