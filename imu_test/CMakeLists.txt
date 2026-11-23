# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.3.1)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(imu_test C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

set(FREERTOS_KERNEL_PATH "${CMAKE_CURRENT_LIST_DIR}/FreeRTOS_Kernel")
set(FREERTOS_CONFIG_FILE_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}")
add_subdirectory(FreeRTOS_Kernel/portable/ThirdParty/GCC/RP2040)

# Add executable. Default name is the project name, version 0.1

add_executable(imu_test imu_test.c )

pico_set_program_name(imu_test "imu_test")
pico_set_program_version(imu_test "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(imu_test 0)
pico_enable_stdio_usb(imu_test 1)

# Limit the time spent waiting for USB transmission during sampling.
target_compile_definitions(imu_test PRIVATE PICO_STDIO_USB_STDOUT_TIMEOUT_US=2000)
target_link_libraries(imu_test PRIVATE pico_stdlib hardware_i2c hardware_pio FreeRTOS-Kernel-Heap4 m)
pico_generate_pio_header(imu_test ${CMAKE_CURRENT_LIST_DIR}/quadrature_encoder.pio)
pico_generate_pio_header(imu_test ${CMAKE_CURRENT_LIST_DIR}/encoder_channel.pio)

pico_add_extra_outputs(imu_test)

