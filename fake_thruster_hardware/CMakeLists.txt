cmake_minimum_required(VERSION 3.23)
project(fake_thruster_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include(GNUInstallDirs)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(realtime_tools REQUIRED)

add_library(fake_thruster_hardware SHARED
  src/fake_thruster_hardware.cpp
)

target_include_directories(fake_thruster_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(fake_thruster_hardware
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

pluginlib_export_plugin_description_file(hardware_interface fake_thruster_hardware.xml)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS fake_thruster_hardware
  EXPORT export_fake_thruster_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  FILES fake_thruster_hardware.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_fake_thruster_hardware HAS_LIBRARY_TARGET)
ament_export_dependencies(
    "rclcpp"
    "rclcpp_lifecycle"
    "pluginlib"
    "hardware_interface"
    "mavros_msgs"
    "rcl_interfaces"
    "realtime_tools"
)

ament_package()
