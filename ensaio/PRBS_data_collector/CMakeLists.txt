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

project(PRBS_data_collector C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Usa a porta RP2040 do FreeRTOS incluída neste projeto.
set(FREERTOS_CONFIG_FILE_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}")
add_subdirectory(
        "${CMAKE_CURRENT_LIST_DIR}/FreeRTOS_Kernel/portable/ThirdParty/GCC/RP2040"
        "${CMAKE_CURRENT_BINARY_DIR}/FreeRTOS_Kernel"
)

# Add executable. Default name is the project name, version 0.1

add_executable(PRBS_data_collector PRBS_data_collector.c )

pico_set_program_name(PRBS_data_collector "PRBS_data_collector")
pico_set_program_version(PRBS_data_collector "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(PRBS_data_collector 0)
pico_enable_stdio_usb(PRBS_data_collector 1)

# Add the standard library to the build
target_link_libraries(PRBS_data_collector
        pico_stdlib
        FreeRTOS-Kernel-Heap4
        hardware_adc
        hardware_gpio
        hardware_pwm
)

# Add the standard include files to the build
target_include_directories(PRBS_data_collector PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

pico_add_extra_outputs(PRBS_data_collector)

