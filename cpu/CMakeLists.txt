cmake_minimum_required(VERSION 3.13)

set(PICO_BOARD pimoroni_pga2350 CACHE STRING "Target board")
set(PICO_PLATFORM rp2350 CACHE STRING "Target platform")

include(${CMAKE_CURRENT_LIST_DIR}/pico_sdk_import.cmake)
project(cpu C CXX ASM)
pico_sdk_init()

set(GCV2_ROOT ${CMAKE_CURRENT_LIST_DIR}/../external/GameConsoleV2 CACHE PATH "GameConsoleV2 tree: engine, synth, input, PSRAM, image format")
set(GCV2_SRC ${GCV2_ROOT}/src)
if(NOT EXISTS ${GCV2_SRC}/console.ld)
    message(FATAL_ERROR "GameConsoleV2 not found at ${GCV2_ROOT}: git submodule update --init")
endif()

# The BIOS: boots a GCV2 game image off the data tape and serves its API through the DE1.
add_executable(bios
    src/bios.c src/tape_ui.c src/tape_state.c src/tape_load.c src/tape_draw.c
    src/link.c src/link_phy.c src/video_link.c src/audio_link.c
    ${GCV2_SRC}/engine/engine.c
    ${GCV2_SRC}/audio/audio_synth.c
    ${GCV2_SRC}/video/render3d.c
    ${GCV2_SRC}/video/psram.c
    ${GCV2_SRC}/cart/cart_format.c
    ${GCV2_SRC}/input/gamepad.c
    src/n64.c
    src/onewire.c)
target_include_directories(bios PRIVATE src ${GCV2_SRC} ${GCV2_SRC}/cart)
pico_generate_pio_header(bios ${CMAKE_CURRENT_LIST_DIR}/src/link.pio)
pico_generate_pio_header(bios ${GCV2_SRC}/input/onewire.pio)
target_link_libraries(bios pico_stdlib pico_multicore hardware_pio hardware_dma hardware_clocks hardware_vreg
    hardware_watchdog)
pico_set_binary_type(bios copy_to_ram)
pico_set_linker_script(bios ${GCV2_SRC}/console.ld)   # firmware RAM ends at 0x20050000; GAMERAM above
pico_enable_stdio_usb(bios 0)
pico_enable_stdio_uart(bios 1)
pico_add_extra_outputs(bios)

add_executable(link_test src/link_test.c)
pico_generate_pio_header(link_test ${CMAKE_CURRENT_LIST_DIR}/src/link_tx.pio)
target_link_libraries(link_test pico_stdlib hardware_pio hardware_dma)
pico_set_binary_type(link_test copy_to_ram)
pico_enable_stdio_usb(link_test 0)
pico_enable_stdio_uart(link_test 0)
pico_add_extra_outputs(link_test)
