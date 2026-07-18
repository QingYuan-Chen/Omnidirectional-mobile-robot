cmake_minimum_required(VERSION 3.22)

project(lower_controller_host_tests C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

enable_testing()

set(APP_DIR "${CMAKE_CURRENT_LIST_DIR}/../firmware/r750_mec_cubemx/R750_MEC_Lower/App")

if(WIN32)
  find_program(POWERSHELL_EXECUTABLE NAMES pwsh.exe powershell.exe REQUIRED)
  add_test(
    NAME serial_capture_tools
    COMMAND
      "${POWERSHELL_EXECUTABLE}"
      -NoProfile
      -ExecutionPolicy Bypass
      -File "${CMAKE_CURRENT_LIST_DIR}/test_serial_capture.ps1"
  )
  add_test(
    NAME g2_experiment_tools
    COMMAND
      "${POWERSHELL_EXECUTABLE}"
      -NoProfile
      -ExecutionPolicy Bypass
      -File "${CMAKE_CURRENT_LIST_DIR}/test_g2_experiment_tools.ps1"
  )
endif()

add_executable(test_app_debug_uart_config
  test_app_debug_uart_config.c
)
target_include_directories(test_app_debug_uart_config PRIVATE "${APP_DIR}/Inc" "${APP_DIR}/Bsp/Inc")
target_compile_options(test_app_debug_uart_config PRIVATE -Wall -Wextra -Wshadow -Wconversion -Werror)
add_test(NAME app_debug_uart_config COMMAND test_app_debug_uart_config)

add_executable(test_app_control_timing
  test_app_control_timing.c
  "${APP_DIR}/Src/app_control_timing.c"
)
target_include_directories(test_app_control_timing PRIVATE "${APP_DIR}/Inc")
target_compile_options(test_app_control_timing PRIVATE -Wall -Wextra -Wshadow -Wconversion -Werror)
add_test(NAME app_control_timing COMMAND test_app_control_timing)

add_executable(test_app_encoder_accumulator
  test_app_encoder_accumulator.c
  "${APP_DIR}/Src/app_encoder_accumulator.c"
)
target_include_directories(test_app_encoder_accumulator PRIVATE "${APP_DIR}/Inc" "${APP_DIR}/Bsp/Inc")
target_compile_options(test_app_encoder_accumulator PRIVATE -Wall -Wextra -Wshadow -Wconversion -Werror)
add_test(NAME app_encoder_accumulator COMMAND test_app_encoder_accumulator)

add_executable(test_app_control_tick_buffer
  test_app_control_tick_buffer.c
  "${APP_DIR}/Src/app_control_tick_buffer.c"
)
target_include_directories(test_app_control_tick_buffer PRIVATE "${APP_DIR}/Inc" "${APP_DIR}/Bsp/Inc")
target_compile_options(test_app_control_tick_buffer PRIVATE -Wall -Wextra -Wshadow -Wconversion -Werror)
add_test(NAME app_control_tick_buffer COMMAND test_app_control_tick_buffer)

add_executable(test_app_motor_open_loop
  test_app_motor_open_loop.c
  "${APP_DIR}/Src/app_motor_open_loop.c"
)
target_include_directories(test_app_motor_open_loop PRIVATE "${APP_DIR}/Inc")
target_compile_options(test_app_motor_open_loop PRIVATE -Wall -Wextra -Wshadow -Wconversion -Werror)
add_test(NAME app_motor_open_loop COMMAND test_app_motor_open_loop)

add_executable(test_app_motion_gate
  test_app_motion_gate.c
  "${APP_DIR}/Src/app_comm_protocol.c"
  "${APP_DIR}/Src/app_motion_gate.c"
)
target_include_directories(test_app_motion_gate PRIVATE "${APP_DIR}/Inc")
target_compile_options(test_app_motion_gate PRIVATE -Wall -Wextra -Wshadow -Wconversion -Werror)
add_test(NAME app_motion_gate COMMAND test_app_motion_gate)

add_executable(test_app_safety_policy
  test_app_safety_policy.c
  "${APP_DIR}/Src/app_safety_policy.c"
)
target_include_directories(test_app_safety_policy PRIVATE "${APP_DIR}/Inc")
target_compile_options(test_app_safety_policy PRIVATE -Wall -Wextra -Wshadow -Wconversion -Werror)
add_test(NAME app_safety_policy COMMAND test_app_safety_policy)

add_executable(test_app_comm_protocol
  test_app_comm_protocol.c
  "${APP_DIR}/Src/app_comm_protocol.c"
  "${APP_DIR}/Src/app_motor_open_loop.c"
)
target_include_directories(test_app_comm_protocol PRIVATE "${APP_DIR}/Inc")
target_compile_options(test_app_comm_protocol PRIVATE -Wall -Wextra -Wshadow -Wconversion -Werror)
add_test(NAME app_comm_protocol COMMAND test_app_comm_protocol)

add_executable(test_app_telemetry
  test_app_telemetry.c
  "${APP_DIR}/Src/app_telemetry.c"
)
target_include_directories(test_app_telemetry PRIVATE "${APP_DIR}/Inc" "${APP_DIR}/Bsp/Inc")
target_compile_options(test_app_telemetry PRIVATE -Wall -Wextra -Wshadow -Wconversion -Werror)
add_test(NAME app_telemetry COMMAND test_app_telemetry)

add_executable(test_app_imu
  test_app_imu.c
  "${APP_DIR}/Src/app_imu.c"
  "${APP_DIR}/Src/imu_eskf.c"
)
target_include_directories(test_app_imu PRIVATE
  "${CMAKE_CURRENT_LIST_DIR}/stubs"
  "${APP_DIR}/Inc"
  "${APP_DIR}/Bsp/Inc"
)
target_compile_options(test_app_imu PRIVATE -Wall -Wextra -Wshadow -Wconversion -Werror)
add_test(NAME app_imu COMMAND test_app_imu)

add_executable(test_imu_eskf
  test_imu_eskf.c
  "${APP_DIR}/Src/imu_eskf.c"
)
target_include_directories(test_imu_eskf PRIVATE "${APP_DIR}/Inc")
target_compile_options(test_imu_eskf PRIVATE -Wall -Wextra -Wshadow -Wconversion -Werror)
add_test(NAME imu_eskf COMMAND test_imu_eskf)

add_executable(test_bsp_uart_tx_queue
  test_bsp_uart_tx_queue.c
  "${APP_DIR}/Bsp/Src/bsp_uart_tx_queue.c"
)
target_include_directories(test_bsp_uart_tx_queue PRIVATE "${APP_DIR}/Inc" "${APP_DIR}/Bsp/Inc")
target_compile_options(test_bsp_uart_tx_queue PRIVATE -Wall -Wextra -Wshadow -Wconversion -Werror)
add_test(NAME bsp_uart_tx_queue COMMAND test_bsp_uart_tx_queue)

add_executable(test_bsp_uart_async
  test_bsp_uart_async.c
  "${APP_DIR}/Bsp/Src/bsp_uart.c"
  "${APP_DIR}/Bsp/Src/bsp_uart_tx_queue.c"
)
target_include_directories(test_bsp_uart_async PRIVATE
  "${CMAKE_CURRENT_LIST_DIR}/stubs"
  "${APP_DIR}/Inc"
  "${APP_DIR}/Bsp/Inc"
)
target_compile_options(test_bsp_uart_async PRIVATE -Wall -Wextra -Wshadow -Wconversion -Werror)
add_test(NAME bsp_uart_async COMMAND test_bsp_uart_async)
