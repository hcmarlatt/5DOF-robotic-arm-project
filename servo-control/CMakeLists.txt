cmake_minimum_required(VERSION 3.13)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(servo_control C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(servo_control
    servo.c
)

pico_enable_stdio_usb(servo_control 1)
pico_enable_stdio_uart(servo_control 0)

pico_add_extra_outputs(servo_control)

target_link_libraries(servo_control pico_stdlib hardware_pwm hardware_adc)
