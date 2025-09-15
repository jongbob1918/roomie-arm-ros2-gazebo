cmake_minimum_required(VERSION 3.8)
project(roomiearm_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find necessary packages
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(serial_driver REQUIRED)

# Create the shared library
add_library(roomiearm_hardware_interface SHARED
  src/roomiearm_hardware_interface.cpp
)

# Link against the necessary libraries
ament_target_dependencies(roomiearm_hardware_interface PUBLIC
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  serial_driver)

target_include_directories(roomiearm_hardware_interface PUBLIC
  include
)

# Install targets
install(TARGETS
  roomiearm_hardware_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY
  include/
  DESTINATION include)

install(FILES
  plugin_description.xml
  DESTINATION share/${PROJECT_NAME})

ament_package()