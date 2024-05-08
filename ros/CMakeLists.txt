cmake_minimum_required(VERSION 3.0.2)
project(
  ros_interface
  VERSION 2.7.2
  LANGUAGES CXX)

find_package(
  catkin REQUIRED
  COMPONENTS message_generation
             message_runtime
             roscpp
             rospy
             sensor_msgs
             std_msgs
             tf
             geometry_msgs)
find_package(airbot_play REQUIRED)

set(CATKIN_BUILD_BINARY_PACKAGE true)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(THREADS_PREFER_PTHREAD_FLAG ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

catkin_package(
  INCLUDE_DIRS
  src
  LIBRARIES
  ros_interface
  CATKIN_DEPENDS
  message_generation
  message_runtime
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  geometry_msgs)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(airbot_arm_ros src/main.cpp)
add_dependencies(airbot_arm_ros ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})
target_link_libraries(airbot_arm_ros ${catkin_LIBRARIES} airbot_play)

add_executable(joy_proxy src/joy_proxy.cpp)
add_dependencies(airbot_arm_ros ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})
target_link_libraries(joy_proxy ${catkin_LIBRARIES})

install(
  TARGETS airbot_arm_ros joy_proxy
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
