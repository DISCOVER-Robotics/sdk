cmake_minimum_required(VERSION 3.16.3)
project(
  airbot_tools
  VERSION 2.6.4
  LANGUAGES CXX)

option(GIT_CLEAN_CHECK "Check if git workspace is clean" ON)
option(COMPILE_TESTS "Compile tests" ON)
include(cmake/Version.cmake)

set(THREADS_PREFER_PTHREAD_FLAG ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)

find_package(airbot_play REQUIRED)
find_package(Threads REQUIRED)
find_package(Curses REQUIRED)
find_package(OpenSSL REQUIRED)

add_subdirectory(thirdparty/argparse)

set(DEPENCENCIES airbot_play::airbot_play ${OpenCV_LIBS} Threads::Threads
                 ${NCURSES_LIBRARIES} argparse)
set(INCLUDE_DIRECTORIES ${OpenCV_INCLUDE_DIRS})

file(GLOB SOURCES "src/*.cpp")

foreach(source ${SOURCES})
  get_filename_component(NAME ${source} NAME_WE)
  message(STATUS "Adding tool: ${NAME}")
  add_executable(${NAME} ${source})
  target_link_libraries(${NAME} ${DEPENCENCIES})
  target_include_directories(${NAME} PRIVATE ${INCLUDE_DIRECTORIES})
  install(
    TARGETS ${NAME}
    EXPORT ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endforeach()

include(cmake/Packing.cmake)
