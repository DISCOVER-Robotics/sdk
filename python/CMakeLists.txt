cmake_minimum_required(VERSION 3.4...3.18)

project(
  airbot_pybridge
  LANGUAGES CXX
  VERSION 2.8.0)

set(THREADS_PREFER_PTHREAD_FLAG ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_BUILD_TYPE Debug)

find_package(
  Python
  COMPONENTS Interpreter Development
  REQUIRED)
find_package(airbot_play REQUIRED)

add_subdirectory(pybind11)
pybind11_add_module(airbot src/main.cpp)
target_link_libraries(airbot PRIVATE Python::Python airbot_play)

# EXAMPLE_VERSION_INFO is defined by setup.py and passed into the C++ code as a
# define (VERSION_INFO) here.
target_compile_definitions(airbot PRIVATE VERSION_INFO=${EXAMPLE_VERSION_INFO})
