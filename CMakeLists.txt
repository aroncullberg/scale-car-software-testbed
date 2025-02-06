# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Consider adding explicit error checking
if(NOT DEFINED ENV{IDF_PATH})
  message(FATAL_ERROR "IDF_PATH environment variable is not set")
endif()

# Check if SDKCONFIG is set in environment
if(NOT DEFINED ENV{SDKCONFIG})
  # Default to vehicle if not specified
  set(ENV{SDKCONFIG} ${CMAKE_SOURCE_DIR}/sdkconfig.vehicle)
endif()


include($ENV{IDF_PATH}/tools/cmake/project.cmake)
set(MINIMUM_ESP_IDF_VERSION "v5.4")

project(fulltech)