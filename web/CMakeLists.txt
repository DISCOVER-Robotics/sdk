cmake_minimum_required(VERSION 3.16.2)
set(project_name arm-control-daemon)
project(${project_name})

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)

find_package(websocketpp REQUIRED)
find_package(httplib REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ruckig REQUIRED)
find_package(assimp REQUIRED)
find_package(airbot_play REQUIRED)
find_package(Curses REQUIRED)

include_directories(/usr/local/include/airbot)
include_directories(/usr/local/include/eigen3)
include_directories(include)

file(GLOB_RECURSE ARM_SOURCES "src/arm/*.cpp")
add_library(${project_name}-arm SHARED ${ARM_SOURCES})
target_link_libraries(${project_name}-arm
    airbot_play_drivers
    airbot_play_modules
)

file(GLOB_RECURSE SERVER_SOURCES "src/server/*.cpp")
add_library(${project_name}-server SHARED ${SERVER_SOURCES})
target_link_libraries(${project_name}-server
    ${project_name}-arm
)

add_executable(${project_name}
    src/entry.cpp
)
target_link_libraries(${project_name}
    ${project_name}-server
)

add_executable(remotesync
    src/remote_sync.cpp
)
target_link_libraries(remotesync
    airbot_play_drivers
    airbot_play_modules
    ${CURSES_LIBRARIES}
)
