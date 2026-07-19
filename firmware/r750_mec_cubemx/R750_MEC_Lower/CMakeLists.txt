cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME R750_MEC_Lower)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    App/Src/app.c
    App/Src/app_comm_protocol.c
    App/Src/app_control_timebase.c
    App/Src/app_control_tick_buffer.c
    App/Src/app_control_timing.c
    App/Src/app_encoder_accumulator.c
    App/Src/app_encoder_period_accumulator.c
    App/Src/app_motor_capture.c
    App/Src/app_motor_open_loop.c
    App/Src/app_motion_gate.c
    App/Src/app_safety_policy.c
    App/Src/app_telemetry.c
    App/Src/app_wheel_speed_estimator.c
    App/Src/app_imu.c
    App/Src/app_tasks.c
    App/Src/imu_eskf.c
    App/Bsp/Src/bsp.c
    App/Bsp/Src/bsp_adc.c
    App/Bsp/Src/bsp_beep.c
    App/Bsp/Src/bsp_encoder.c
    App/Bsp/Src/bsp_imu.c
    App/Bsp/Src/bsp_motor.c
    App/Bsp/Src/bsp_uart.c
    App/Bsp/Src/bsp_uart_tx_queue.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    App/Inc
    App/Bsp/Inc
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
