# ROS 2 Workspace Configuration for BNO085 Driver

# Source ROS 2 environment
source /opt/ros/jazzy/setup.bash

# Set workspace-specific environment variables
export BNO085_DEFAULT_DEVICE="/dev/ttyACM0"
export BNO085_DEFAULT_BAUD="115200"
export BNO085_FIRMWARE_PATH="$PWD/src/"

# Aliases for common commands
alias bno085_build="colcon build --packages-select bno085_driver"
alias bno085_test="colcon test --packages-select bno085_driver"
alias bno085_launch="ros2 launch bno085_driver bno085_complete.launch.py"
alias bno085_monitor="ros2 run bno085_driver bno085_calibration_monitor"
alias bno085_upload="cd src/bno085_driver && pio run --target upload"

echo "BNO085 ROS 2 workspace configured!"
echo "Available aliases:"
echo "  bno085_build   - Build the package"
echo "  bno085_test    - Run tests"
echo "  bno085_launch  - Launch the complete system"
echo "  bno085_monitor - Start calibration monitor"
echo "  bno085_upload  - Upload firmware to Pico"
