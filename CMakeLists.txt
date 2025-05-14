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

# Add the PID controller source files
add_library(pid STATIC 
    src/pid/pid_controller.cpp
)

target_include_directories(pid PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/pid>
    $<INSTALL_INTERFACE:include/pid>
)

add_executable(docking_node src/docking_node.cpp)

target_link_libraries(docking_node
    pid
)

ament_target_dependencies(docking_node
    rclcpp
)
install(TARGETS docking_node
    DESTINATION lib/${PROJECT_NAME}
)

# ament_target_dependencies(docking_controller_node
#   rclcpp
#   rclcpp_action
#   geometry_msgs
#   taskbot_docking_interfaces
#   tf2_ros
#   tf2_geometry_msgs
#   yaml-cpp
#   Eigen3
# )
# install(TARGETS docking_controller_node
#   DESTINATION lib/${PROJECT_NAME}
# )



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
