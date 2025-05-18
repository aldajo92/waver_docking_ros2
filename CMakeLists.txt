cmake_minimum_required(VERSION 3.8)
project(waver_docking)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include)
include_directories(include/pid_module)

# Add the pid_module source files
add_library(pid_module STATIC 
    src/pid_module/pid_controller.cpp
)

target_include_directories(pid_module PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/pid_module>
    $<INSTALL_INTERFACE:include/pid_module>
)

# Add the PID parameters source files
add_library(pid_parameters STATIC
    src/pid_parameters.cpp
)

target_include_directories(pid_parameters PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/pid_module>
    $<INSTALL_INTERFACE:include>
    $<INSTALL_INTERFACE:include/pid_module>
)

ament_target_dependencies(pid_parameters
    rclcpp
)

# Add the docking node executable
add_executable(docking_node src/docking_node.cpp)

# Include the directories for docking_node
target_include_directories(docking_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/pid_module>
    $<INSTALL_INTERFACE:include>
)

# Link the docking_node executable with the required libraries
target_link_libraries(docking_node
    pid_parameters
    pid_module
)

ament_target_dependencies(docking_node
    rclcpp
)
install(TARGETS docking_node
    DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY params/
  DESTINATION share/${PROJECT_NAME}/params
)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()
