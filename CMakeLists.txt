cmake_minimum_required(VERSION 3.22)

set(FIRMWARE_PROJECT_DIR "${CMAKE_CURRENT_LIST_DIR}/firmware/r750_mec_cubemx/R750_MEC_Lower")
set(CMAKE_TOOLCHAIN_FILE
    "${FIRMWARE_PROJECT_DIR}/cmake/gcc-arm-none-eabi.cmake"
    CACHE FILEPATH "ARM GCC toolchain file")
set(CMAKE_MAKE_PROGRAM
    "D:/STM32CubeCLT_1.20.0/Ninja/bin/ninja.exe"
    CACHE FILEPATH "Ninja executable")

project(MobileRobotLowerWorkspace C ASM)

add_subdirectory("${FIRMWARE_PROJECT_DIR}" firmware_build)
