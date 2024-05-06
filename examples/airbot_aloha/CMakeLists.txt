cmake_minimum_required(VERSION 3.16.3)
project(
  airbot_aloha
  VERSION 2.8.0
  LANGUAGES CXX)

option(GIT_CLEAN_CHECK "Check if git workspace is clean" ON)
option(COMPILE_TESTS "Compile tests" ON)
include(cmake/Version.cmake)
include(GNUInstallDirs)

set(THREADS_PREFER_PTHREAD_FLAG ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)

find_package(airbot_play REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)
find_package(Curses REQUIRED)

add_subdirectory(thirdparty/argparse)

find_package(Python COMPONENTS Interpreter Development)
install(
  DIRECTORY python/${PROJECT_NAME}/
  DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/${PROJECT_NAME}
  FILES_MATCHING
  PATTERN "*.py")
install(FILES python/setup.py
        DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME})
message(STATUS "Installing Python setup.py file to \
    ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/setup.py")

add_custom_target(
  install_python_packages
  COMMAND ${Python_EXECUTABLE} -m pip install ${CMAKE_SOURCE_DIR}/python -i
          https://mirrors.tuna.tsinghua.edu.cn/pypi/web/simple
  COMMENT "Installing Python airbot_aloha package")
install(CODE "execute_process(COMMAND ${CMAKE_COMMAND} \
--build ${CMAKE_BINARY_DIR} --target install_python_packages)")

add_library(webcam STATIC src/camera/webcam.cpp)
target_include_directories(webcam PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(webcam ${OpenCV_LIBS})

set(PUBLIC_DEPENDENCIES airbot_play::airbot_play ${OpenCV_LIBS}
                        Threads::Threads ${NCURSES_LIBRARIES} argparse)
set(PRIVATE_DEPENDENCIES webcam)
set(INCLUDE_DIRECTORIES ${OpenCV_INCLUDE_DIRS})

file(GLOB SOURCES "src/*.cpp")

foreach(source ${SOURCES})
  get_filename_component(NAME ${source} NAME_WE)
  message(STATUS "Adding tool: ${NAME}")
  add_executable(${NAME} ${source})
  target_link_libraries(${NAME} PUBLIC ${PUBLIC_DEPENDENCIES})
  target_link_libraries(${NAME} PRIVATE ${PRIVATE_DEPENDENCIES})
  target_include_directories(${NAME} PRIVATE ${INCLUDE_DIRECTORIES})
  install(
    TARGETS ${NAME}
    EXPORT ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endforeach()

include(cmake/Packing.cmake)
