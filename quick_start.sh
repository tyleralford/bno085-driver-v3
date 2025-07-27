#!/bin/bash

# BNO085 IMU Driver Quick Start Script
# This script helps you get started with the BNO085 driver quickly

set -e

echo "BNO085 IMU Driver Quick Start"
echo "=============================="

# Function to check if command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Function to find the Pico device
find_pico_device() {
    echo "Looking for Raspberry Pi Pico..."
    
    # Check common device paths
    for device in /dev/ttyACM0 /dev/ttyACM1 /dev/ttyACM2 /dev/ttyUSB0; do
        if [ -e "$device" ]; then
            echo "Found potential device: $device"
            return 0
        fi
    done
    
    echo "No device found automatically."
    echo "Please ensure the Raspberry Pi Pico is connected and in the correct mode."
    return 1
}

# Check dependencies
echo "Checking dependencies..."

if ! command_exists ros2; then
    echo "Error: ROS 2 is not installed or not sourced."
    echo "Please install ROS 2 and source the setup script:"
    echo "  source /opt/ros/jazzy/setup.bash"
    exit 1
fi

if ! ros2 pkg list | grep -q micro_ros_agent; then
    echo "Error: micro-ROS agent not found."
    echo "Please install it:"
    echo "  https://micro.ros.org/docs/tutorials/core/first_application_linux/"
    exit 1
fi

echo "Dependencies OK!"

# Find device
if find_pico_device; then
    DEVICE=$(ls /dev/ttyACM* 2>/dev/null | head -n1)
    echo "Using device: $DEVICE"
else
    echo "Enter device path manually (e.g., /dev/ttyACM0):"
    read -r DEVICE
fi

# Check permissions
echo "Checking device permissions..."
if [ ! -r "$DEVICE" ] || [ ! -w "$DEVICE" ]; then
    echo "Setting device permissions..."
    sudo chmod 666 "$DEVICE"
fi

echo "Starting micro-ROS agent..."
echo "Device: $DEVICE"
echo "Baud rate: 115200"
echo ""
echo "Press Ctrl+C to stop the agent."
echo ""

# Start the micro-ROS agent
ros2 run micro_ros_agent micro_ros_agent serial --dev "$DEVICE" --baud 115200
