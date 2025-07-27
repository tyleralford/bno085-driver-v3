#!/bin/bash

# BNO085 ROS 2 Package Validation Script
# This script validates that the package is properly structured and can be built

set -e  # Exit on any error

echo "=============================================="
echo "BNO085 ROS 2 Package Validation"
echo "=============================================="

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    if [ $1 -eq 0 ]; then
        echo -e "${GREEN}✓ $2${NC}"
    else
        echo -e "${RED}✗ $2${NC}"
        exit 1
    fi
}

print_warning() {
    echo -e "${YELLOW}⚠ $1${NC}"
}

print_info() {
    echo -e "${YELLOW}ℹ $1${NC}"
}

# Check if we're in the right directory
if [ ! -f "package.xml" ]; then
    echo -e "${RED}Error: package.xml not found. Please run this script from the package root directory.${NC}"
    exit 1
fi

print_info "Validating package structure..."

# Check required files
echo ""
echo "Checking required ROS 2 package files:"
[ -f "package.xml" ] && print_status 0 "package.xml exists" || print_status 1 "package.xml missing"
[ -f "CMakeLists.txt" ] && print_status 0 "CMakeLists.txt exists" || print_status 1 "CMakeLists.txt missing"
[ -f "setup.py" ] && print_status 0 "setup.py exists" || print_status 1 "setup.py missing"
[ -f "setup.cfg" ] && print_status 0 "setup.cfg exists" || print_status 1 "setup.cfg missing"

# Check directory structure
echo ""
echo "Checking directory structure:"
[ -d "launch" ] && print_status 0 "launch/ directory exists" || print_status 1 "launch/ directory missing"
[ -d "rviz" ] && print_status 0 "rviz/ directory exists" || print_status 1 "rviz/ directory missing"
[ -d "examples" ] && print_status 0 "examples/ directory exists" || print_status 1 "examples/ directory missing"
[ -d "bno085_driver" ] && print_status 0 "bno085_driver/ Python package exists" || print_status 1 "bno085_driver/ Python package missing"
[ -d "firmware" ] && print_status 0 "firmware/ directory exists" || print_status 1 "firmware/ directory missing"
[ -d "test" ] && print_status 0 "test/ directory exists" || print_status 1 "test/ directory missing"
[ -d "resource" ] && print_status 0 "resource/ directory exists" || print_status 1 "resource/ directory missing"

# Check Python package structure
echo ""
echo "Checking Python package structure:"
[ -f "bno085_driver/__init__.py" ] && print_status 0 "bno085_driver/__init__.py exists" || print_status 1 "bno085_driver/__init__.py missing"
[ -f "bno085_driver/calibration_monitor.py" ] && print_status 0 "calibration_monitor.py exists" || print_status 1 "calibration_monitor.py missing"
[ -f "resource/bno085_driver" ] && print_status 0 "resource marker file exists" || print_status 1 "resource marker file missing"

# Check launch files
echo ""
echo "Checking launch files:"
[ -f "launch/bno085_complete.launch.py" ] && print_status 0 "Complete launch file exists" || print_status 1 "Complete launch file missing"
[ -f "launch/bno085_agent.launch.py" ] && print_status 0 "Agent launch file exists" || print_status 1 "Agent launch file missing"

# Check firmware files
echo ""
echo "Checking firmware files:"
[ -f "firmware/platformio.ini" ] && print_status 0 "PlatformIO config exists" || print_status 1 "PlatformIO config missing"
[ -f "firmware/src/main.cpp" ] && print_status 0 "Main firmware file exists" || print_status 1 "Main firmware file missing"
[ -d "firmware/include" ] && print_status 0 "Firmware include directory exists" || print_status 1 "Firmware include directory missing"

# Check if ROS 2 is sourced
echo ""
echo "Checking ROS 2 environment:"
if [ -z "$ROS_DISTRO" ]; then
    print_warning "ROS 2 not sourced. Please run: source /opt/ros/jazzy/setup.bash"
else
    print_status 0 "ROS 2 $ROS_DISTRO is sourced"
fi

# Test package.xml validation
echo ""
echo "Validating package.xml:"
if command -v xmllint &> /dev/null; then
    if xmllint --noout package.xml 2>/dev/null; then
        print_status 0 "package.xml is valid XML"
    else
        print_status 1 "package.xml has XML errors"
    fi
else
    print_warning "xmllint not available, skipping XML validation"
fi

# Check for common dependencies
echo ""
echo "Checking for common ROS 2 dependencies:"
if [ -n "$ROS_DISTRO" ]; then
    ros2 pkg list | grep -q "sensor_msgs" && print_status 0 "sensor_msgs available" || print_warning "sensor_msgs not found"
    ros2 pkg list | grep -q "geometry_msgs" && print_status 0 "geometry_msgs available" || print_warning "geometry_msgs not found"
    ros2 pkg list | grep -q "tf2_ros" && print_status 0 "tf2_ros available" || print_warning "tf2_ros not found"
    ros2 pkg list | grep -q "micro_ros_agent" && print_status 0 "micro_ros_agent available" || print_warning "micro_ros_agent not found"
fi

echo ""
echo "=============================================="
if [ -n "$ROS_DISTRO" ]; then
    echo -e "${GREEN}Package validation completed successfully!${NC}"
    echo ""
    echo "Next steps:"
    echo "1. Build the package: colcon build --packages-select bno085_driver"
    echo "2. Source the workspace: source install/setup.bash"
    echo "3. Test the package: colcon test --packages-select bno085_driver"
    echo "4. Launch the system: ros2 launch bno085_driver bno085_complete.launch.py"
else
    echo -e "${YELLOW}Package structure is valid, but ROS 2 is not sourced.${NC}"
    echo "Please source ROS 2 before building: source /opt/ros/jazzy/setup.bash"
fi
echo "=============================================="
