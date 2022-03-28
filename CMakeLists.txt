cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 11)
project(mc-sawyer LANGUAGES CXX VERSION 1.0)

find_package(sawyer_description REQUIRED)
find_package(mc_rtc REQUIRED)

if("${sawyer_description_INSTALL_PREFIX}" STREQUAL "")
  if("${sawyer_description_SOURCE_PREFIX}" STREQUAL "")
    message(FATAL_ERROR "Your sawyer_description package does not define where to find the data")
  endif()
  set(SAWYER_DESCRIPTION_PATH "${sawyer_description_SOURCE_PREFIX}")
else()
  set(SAWYER_DESCRIPTION_PATH "${sawyer_description_INSTALL_PREFIX}")
endif()

message(STATUS "SAWYER_DESCRIPTION_PATH: ${SAWYER_DESCRIPTION_PATH}")
configure_file(src/config.h.in "${CMAKE_CURRENT_BINARY_DIR}/include/config.h")

macro(add_module NAME)
  add_robot(${NAME} src/${NAME}.cpp src/${NAME}.h)
  target_include_directories(${NAME} PRIVATE "${CMAKE_CURRENT_BINARY_DIR}/include")
endmacro()

add_module(sawyer)

option(DISABLE_TESTS "Disable unit tests" OFF)
if(NOT ${DISABLE_TESTS})
  enable_testing()
  add_subdirectory(tests)
endif()
