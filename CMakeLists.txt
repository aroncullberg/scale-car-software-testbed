cmake_minimum_required(VERSION 3.22)

if(NOT DEFINED IDF_TARGET AND NOT DEFINED ENV{IDF_TARGET})
    set(IDF_TARGET "esp32s3" CACHE STRING "ESP-IDF target")
endif()

if(NOT DEFINED SDKCONFIG)
    set(SDKCONFIG "${CMAKE_CURRENT_LIST_DIR}/sdkconfig.2026" CACHE FILEPATH "Car configuration")
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
idf_build_set_property(MINIMAL_BUILD ON)
set(PROJECT_VER "2026.1")
project(car)

# Compile patched copies, leaving the managed dependency and its hash intact.
idf_component_get_property(bno08x_directory myles-parfeniuk__esp32_BNO08x COMPONENT_DIR)
idf_component_get_property(bno08x_library myles-parfeniuk__esp32_BNO08x COMPONENT_LIB)
set(bno08x_patch "${CMAKE_CURRENT_LIST_DIR}/components/imu/patches/shared_spi.patch")
set(bno08x_patched "${CMAKE_BINARY_DIR}/bno08x_shared_spi")
file(MAKE_DIRECTORY "${bno08x_patched}/source")
foreach(source BNO08x.cpp BNO08xSH2HAL.cpp)
    configure_file("${bno08x_directory}/source/${source}"
                   "${bno08x_patched}/source/${source}" COPYONLY)
endforeach()
find_program(PATCH_EXECUTABLE patch REQUIRED)
execute_process(
    COMMAND "${PATCH_EXECUTABLE}" --batch -p1 -F0 -i "${bno08x_patch}"
    WORKING_DIRECTORY "${bno08x_patched}"
    RESULT_VARIABLE patch_result OUTPUT_VARIABLE patch_output ERROR_VARIABLE patch_error
)
if(NOT patch_result EQUAL 0)
    message(FATAL_ERROR "BNO08x shared-SPI patch failed: ${patch_output}${patch_error}")
endif()
get_target_property(bno08x_sources ${bno08x_library} SOURCES)
list(FILTER bno08x_sources EXCLUDE REGEX "(^|/)source/(BNO08x|BNO08xSH2HAL)\\.cpp$")
set_property(TARGET ${bno08x_library} PROPERTY SOURCES ${bno08x_sources})
target_sources(${bno08x_library} PRIVATE
    "${bno08x_patched}/source/BNO08x.cpp"
    "${bno08x_patched}/source/BNO08xSH2HAL.cpp"
)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS "${bno08x_patch}")

# The existing TinyGPSPlus component omits its esp_timer requirement and uses
# intentional switch fallthrough without an annotation for the newer compiler.
idf_component_get_property(tinygps_library andyfinity__TinyGPSPlus-ESPIDF COMPONENT_LIB)
target_link_libraries(${tinygps_library} PUBLIC idf::esp_timer)
target_compile_options(${tinygps_library} PRIVATE -Wno-error=implicit-fallthrough)
