# BNO085 micro-ROS IMU Driver

A high-performance micro-ROS driver for the Adafruit BNO085 9-DOF IMU sensor, designed for the Raspberry Pi Pico. This driver provides real-time IMU data publishing to ROS 2 with dynamic covariance scaling based on sensor calibration status.

## Features

- **High-frequency data publishing** at 60Hz with synchronized timestamps
- **Dynamic covariance scaling** based on individual sensor calibration status
- **Robust state machine** with fault recovery and LED status indicators
- **Per-sensor calibration tracking** for accelerometer, gyroscope, and orientation
- **Game Rotation Vector** for magnetometer-free orientation estimation
- **Time synchronization** with ROS 2 agent for accurate timestamping
- **Hardware abstraction** with configurable I2C pins and LED indicators

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

### 1. Clone the Repository
```bash
git clone <repository-url>
cd bno085-driver-v3
```

### 2. Install PlatformIO
If you haven't already, install PlatformIO:
```bash
# Install PlatformIO Core
pip install platformio

# Or install via VS Code extension
# Search for "PlatformIO IDE" in VS Code extensions
```

### 3. Build and Upload Firmware
```bash
# Build the project
pio run

# Upload to Raspberry Pi Pico
# Put Pico in bootloader mode (hold BOOTSEL while connecting USB)
pio run --target upload

# Optional: Monitor serial output
pio device monitor --baud 115200
```

### 4. Install ROS 2 and micro-ROS Agent
```bash
# Install ROS 2 Jazzy (Ubuntu 22.04)
sudo apt update
sudo apt install ros-jazzy-desktop

# Source ROS 2
source /opt/ros/jazzy/setup.bash

# Install micro-ROS agent
sudo apt install ros-jazzy-micro-ros-agent

# Or build from source:
# mkdir -p ~/microros_ws/src
# cd ~/microros_ws/src
# git clone -b jazzy https://github.com/micro-ROS/micro_ros_setup.git
# cd ~/microros_ws
# colcon build
# source install/setup.bash
```

## Usage

### 1. Start the micro-ROS Agent
The micro-ROS agent bridges communication between the Pico and ROS 2. Choose one of the following methods:

#### Option A: Direct Serial Connection
```bash
# Find the Pico's serial port
ls /dev/ttyACM*

# Start the agent (replace /dev/ttyACM0 with your device)
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 --baud 115200
```

#### Option B: Using Launch File (Recommended)
Create a launch file for easier management:

```bash
# Create launch directory
mkdir -p ~/imu_ws/src/bno085_launch/launch

# Copy the launch file (see launch files section below)
# Then build and source the workspace
cd ~/imu_ws
colcon build
source install/setup.bash

# Launch the agent
ros2 launch bno085_launch bno085_agent.launch.py
```

### 2. Verify Data Publishing
```bash
# List active topics
ros2 topic list

# View IMU data
ros2 topic echo /imu/data

# Check publishing rate
ros2 topic hz /imu/data

# View calibration status (reflected in covariance values)
ros2 topic echo /imu/data --field orientation_covariance
```

## Launch Files

### Agent Launch File
Create `~/imu_ws/src/bno085_launch/launch/bno085_agent.launch.py`:

```python
#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'device',
            default_value='/dev/ttyACM0',
            description='Serial device path for the Raspberry Pi Pico'
        ),
        
        DeclareLaunchArgument(
            'baud',
            default_value='115200',
            description='Baud rate for serial communication'
        ),
        
        DeclareLaunchArgument(
            'verbose',
            default_value='false',
            description='Enable verbose logging'
        ),
        
        # Log launch information
        LogInfo(
            msg=['Starting micro-ROS agent for BNO085 IMU driver']
        ),
        
        LogInfo(
            msg=[
                'Device: ', LaunchConfiguration('device'),
                ', Baud: ', LaunchConfiguration('baud')
            ]
        ),
        
        # micro-ROS agent node
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            arguments=[
                'serial',
                '--dev', LaunchConfiguration('device'),
                '--baud', LaunchConfiguration('baud'),
            ],
            output='screen',
            parameters=[
                {'verbose': LaunchConfiguration('verbose')}
            ],
            respawn=True,
            respawn_delay=2.0
        )
    ])
```

### Complete System Launch File
Create `~/imu_ws/src/bno085_launch/launch/bno085_system.launch.py`:

```python
#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'device',
            default_value='/dev/ttyACM0',
            description='Serial device path for the Raspberry Pi Pico'
        ),
        
        DeclareLaunchArgument(
            'start_rviz',
            default_value='true',
            description='Start RViz for IMU visualization'
        ),
        
        DeclareLaunchArgument(
            'start_rqt',
            default_value='false',
            description='Start rqt_plot for real-time data plotting'
        ),
        
        # Include the agent launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('bno085_launch'),
                    'launch',
                    'bno085_agent.launch.py'
                ])
            ]),
            launch_arguments={
                'device': LaunchConfiguration('device'),
                'verbose': 'true'
            }.items()
        ),
        
        # Static transform publisher (IMU to base_link)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_static_transform',
            arguments=[
                '0', '0', '0',  # translation x, y, z
                '0', '0', '0', '1',  # rotation x, y, z, w
                'base_link',  # parent frame
                'imu_link'    # child frame
            ],
            output='screen'
        ),
        
        # IMU filter for sensor fusion (optional)
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter',
            parameters=[{
                'use_mag': False,  # Using game rotation vector (no magnetometer)
                'publish_tf': False,
                'world_frame': 'enu',
                'fixed_frame': 'imu_link'
            }],
            remappings=[
                ('imu/data_raw', '/imu/data'),
                ('imu/data', '/imu/filtered')
            ],
            condition=UnlessCondition(LaunchConfiguration('start_rviz'))
        ),
        
        # RViz for visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', PathJoinSubstitution([
                FindPackageShare('bno085_launch'),
                'rviz',
                'bno085_imu.rviz'
            ])],
            condition=IfCondition(LaunchConfiguration('start_rviz')),
            output='screen'
        ),
        
        # rqt_plot for real-time data visualization
        Node(
            package='rqt_plot',
            executable='rqt_plot',
            name='rqt_plot_imu',
            arguments=[
                '/imu/data/orientation/w',
                '/imu/data/orientation/x',
                '/imu/data/orientation/y',
                '/imu/data/orientation/z',
                '--title', 'IMU Quaternion Data'
            ],
            condition=IfCondition(LaunchConfiguration('start_rqt')),
            output='screen'
        )
    ])
```

### Package Configuration
Create `~/imu_ws/src/bno085_launch/package.xml`:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypes="ROS package format description"?>
<package format="3">
  <name>bno085_launch</name>
  <version>1.0.0</version>
  <description>Launch files for BNO085 IMU driver</description>
  
  <maintainer email="your.email@example.com">Your Name</maintainer>
  <license>MIT</license>
  
  <buildtool_depend>ament_cmake</buildtool_depend>
  
  <exec_depend>micro_ros_agent</exec_depend>
  <exec_depend>tf2_ros</exec_depend>
  <exec_depend>rviz2</exec_depend>
  <exec_depend>rqt_plot</exec_depend>
  <exec_depend>imu_filter_madgwick</exec_depend>
  
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

Create `~/imu_ws/src/bno085_launch/CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.8)
project(bno085_launch)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install RViz config (if created)
install(DIRECTORY
  rviz
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_package()
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
