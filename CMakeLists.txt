cmake_minimum_required(VERSION 3.8)
project(bno085_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(launch REQUIRED)
find_package(launch_ros REQUIRED)

# Install Python package
ament_python_install_package(${PROJECT_NAME})

# Install launch files
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install RViz configuration files
install(DIRECTORY
  rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

# Install examples
install(DIRECTORY
  examples/
  DESTINATION share/${PROJECT_NAME}/examples
)

# Install documentation
install(FILES
  README.md
  LICENSE
  DESTINATION share/${PROJECT_NAME}
)

# Install PlatformIO firmware project
install(DIRECTORY
  firmware/
  DESTINATION share/${PROJECT_NAME}/firmware
  USE_SOURCE_PERMISSIONS
)

# Install quick start script
install(PROGRAMS
  quick_start.sh
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  
  # Add Python tests
  ament_add_pytest_test(test_bno085_driver test/test_bno085_driver.py)
endif()

ament_package()
