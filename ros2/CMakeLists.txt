cmake_minimum_required(VERSION 3.8)
project(airbot_ros2_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(fmt REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(airbot_play REQUIRED)

include_directories( "${EIGEN3_INCLUDE_DIR}" )
include_directories( "${OpenCV_INCLUDE_DIRS}" )

add_executable(airbot_arm_ros src/airbot_main.cpp)
ament_target_dependencies(airbot_arm_ros rclcpp sensor_msgs std_msgs geometry_msgs kdl_parser)

target_link_libraries(airbot_arm_ros airbot_play fmt::fmt kdl_parser)

add_executable(joy_proxy src/joy_proxy.cpp)
ament_target_dependencies(joy_proxy rclcpp sensor_msgs)

install(TARGETS
  airbot_arm_ros
  joy_proxy
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)


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
