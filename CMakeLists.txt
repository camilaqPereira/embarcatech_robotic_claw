# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ====================================================================================
set(PICO_BOARD pico_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(embarcatech_robotic_claw C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(embarcatech_robotic_claw embarcatech_robotic_claw.c )

pico_set_program_name(embarcatech_robotic_claw "embarcatech_robotic_claw")
pico_set_program_version(embarcatech_robotic_claw "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(embarcatech_robotic_claw 0)
pico_enable_stdio_usb(embarcatech_robotic_claw 1)

# Add the standard library to the build
target_link_libraries(embarcatech_robotic_claw
        pico_stdlib
        hardware_pwm
        hardware_adc
        )

# Add the standard include files to the build
target_include_directories(embarcatech_robotic_claw PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

# Add any user requested libraries
target_link_libraries(embarcatech_robotic_claw 
        
        )

pico_add_extra_outputs(embarcatech_robotic_claw)

