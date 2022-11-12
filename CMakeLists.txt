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

# set(CMAKE_PREFIX_PATH "/usr/include/")


# set(IMPORTED_LOCATION"/usr/lib/x86_64-linux-gnu")
# set(libserial_INCLUDE_DIRS "/usr/include/libserial")
# set(libserial_LIBRARIES "/usr/lib/x86_64-linux-gnu/libserial")
# add_library(serial SHARED IMPORTED)
add_executable(serial_controler include/serial_controler/main.cpp include/serial_controler/hover_comms.cpp)

find_package(ament_cmake REQUIRED)
# find_package(serial REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)

target_link_libraries(${PROJECT_NAME} serial) # the magic line to add the Serial library (drop the lib off the front)

ament_export_libraries(serial)


target_include_directories(
  serial_controler
  PRIVATE
  include
)

ament_target_dependencies(
  serial_controler
  hardware_interface
  controller_manager
  rclcpp
  pluginlib
)

pluginlib_export_plugin_description_file(hardware_interface robot_hardware.xml)





ament_package()