cmake_minimum_required(VERSION 3.5)
project(serial_controler)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

set(CMAKE_BUILD_TYPE Debug)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

set(CMAKE_PREFIX_PATH "/tmp/usr/local/**" ;"/usr/local/serial/tmp/usr/local/**")

find_package(ament_cmake REQUIRED)
find_package(serial REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)

# add_library(serial_controler SHARED include/serial_controler/hover_comms.cpp include/serial_controler/main.cpp )
add_executable(serial_controler include/serial_controler/main.cpp include/serial_controler/hover_comms.cpp)

target_include_directories(
  serial_controler
  PRIVATE
  include
)
ament_target_dependencies(
  serial_controler
  hardware_interface
  controller_manager
  serial
  rclcpp
  pluginlib
)

pluginlib_export_plugin_description_file(hardware_interface robot_hardware.xml)



# add_library(fake_robot SHARED src/fake_robot.cpp src/wheel.cpp)
# set_property(TARGET fake_robot PROPERTY POSITION_INDEPENDENT_CODE ON)

# target_include_directories(
#   fake_robot
#   PRIVATE
#   include
# )
# ament_target_dependencies(
#   fake_robot
#   hardware_interface
#   controller_manager
#   rclcpp
#   pluginlib
# )

# pluginlib_export_plugin_description_file(hardware_interface fake_robot_hardware.xml)




# install(
#   TARGETS diffdrive_arduino fake_robot
#   DESTINATION lib
# )




# install(
#   DIRECTORY controllers launch/
#   DESTINATION share/${PROJECT_NAME}
# )


# ament_export_libraries(
#   diffdrive_arduino
#   fake_robot
# )
# ament_export_dependencies(
#   hardware_interface
#   controller_manager
#   serial
#   rclcpp
#   pluginlib
# )

ament_package()


