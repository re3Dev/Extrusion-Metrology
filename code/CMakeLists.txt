cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(beadsense_test C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()


add_executable(beadsense_test
    main.c
)

# ------------------------------------------------------------
# Libraries
# ------------------------------------------------------------

target_link_libraries(
    beadsense_test

    pico_stdlib

    hardware_adc
)


# ------------------------------------------------------------
# USB serial
# ------------------------------------------------------------

pico_enable_stdio_usb(
    beadsense_test
    1
)

pico_enable_stdio_uart(
    beadsense_test
    0
)


# ------------------------------------------------------------
# Generate UF2 / ELF / HEX
# ------------------------------------------------------------

pico_add_extra_outputs(
    beadsense_test
)