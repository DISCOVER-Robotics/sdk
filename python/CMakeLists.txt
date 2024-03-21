cmake_minimum_required(VERSION 3.4...3.18)
project(airbot)

find_package(
  Python
  COMPONENTS Interpreter Development
  REQUIRED)
find_package(airbot_play REQUIRED)
find_package(assimp REQUIRED)
find_package(Boost REQUIRED COMPONENTS serialization system thread filesystem)
find_package(Eigen3 REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(Threads)
find_package(orocos_kdl REQUIRED)
find_package(ompl REQUIRED)
find_package(ruckig REQUIRED)
find_package(Curses REQUIRED)
find_package(spdlog REQUIRED)

# FIXME: This is a hack to get the correct include directory for arm_control
message(${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_INCLUDEDIR}/airbot)

add_subdirectory(pybind11)
pybind11_add_module(airbot src/main.cpp)
target_include_directories(airbot PRIVATE ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_INCLUDEDIR}/airbot)
target_link_libraries(airbot PRIVATE Python::Python airbot_play_drivers
                                     airbot_play_modules)

# EXAMPLE_VERSION_INFO is defined by setup.py and passed into the C++ code as a
# define (VERSION_INFO) here.
target_compile_definitions(airbot PRIVATE VERSION_INFO=${EXAMPLE_VERSION_INFO})
