cmake_minimum_required(VERSION 3.16)
project(falcon_arm_hardware)

# Set C++ standard to 17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required ROS2 dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(serial REQUIRED)
find_package(std_msgs REQUIRED)
find_package(controller_interface REQUIRED)

# Find the serial library explicitly
find_library(SERIAL_LIBRARY NAMES serial HINTS ${serial_DIR}/../../../lib REQUIRED)

# Create the hardware interface library
add_library(
  falcon_arm_hardware_interface
  SHARED
  src/falcon_arm_hardware_interface.cpp
)

# Link the required libraries to our hardware interface
target_link_libraries(
  falcon_arm_hardware_interface
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  ${SERIAL_LIBRARY}
)

# Include directories for the library
target_include_directories(
  falcon_arm_hardware_interface
  PRIVATE
  ${serial_INCLUDE_DIRS}
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Install the library
install(
  TARGETS falcon_arm_hardware_interface
  EXPORT falcon_arm_hardwareTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install header files
install(
  DIRECTORY include/
  DESTINATION include
)

# Install the plugin XML file so pluginlib can find the hardware interface
install(
  FILES falcon_arm_hardware_plugins.xml
  DESTINATION share/falcon_arm_hardware
)

# Export CMake targets  
ament_export_targets(falcon_arm_hardwareTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  serial
  std_msgs
  controller_interface
)

# Finalize the package
ament_package()
