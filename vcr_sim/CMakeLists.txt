cmake_minimum_required(VERSION 3.13)

set(PICO_BOARD pimoroni_pga2350 CACHE STRING "Target board")
set(PICO_PLATFORM rp2350 CACHE STRING "Target platform")

include(pico_sdk_import.cmake)
project(vcr_sim C CXX ASM)
pico_sdk_init()

set(VHS_ROOT ${CMAKE_CURRENT_LIST_DIR}/..)
set(VHS_PYTHON ${VHS_ROOT}/.venv/bin/python CACHE FILEPATH "Python with numpy for tools/pattern.py")
set(FIELD_BIN ${CMAKE_CURRENT_BINARY_DIR}/field.bin)

add_custom_command(
    OUTPUT ${FIELD_BIN}
    COMMAND ${VHS_PYTHON} ${VHS_ROOT}/tools/pattern.py --emit ${FIELD_BIN}
    DEPENDS ${VHS_ROOT}/tools/pattern.py
    COMMENT "Rendering NTSC field from tools/pattern.py")
add_custom_target(field_bin DEPENDS ${FIELD_BIN})

# Flashed separately at 0x10040000 (scripts/flash_rp2350.sh clips), so firmware builds never wait on it.
set(CLIPS_BIN ${CMAKE_CURRENT_BINARY_DIR}/clips.bin)
file(GLOB CLIP_VIDEOS CONFIGURE_DEPENDS ${VHS_ROOT}/videos/*)
add_custom_command(
    OUTPUT ${CLIPS_BIN}
    COMMAND ${VHS_PYTHON} ${VHS_ROOT}/tools/clips.py ${VHS_ROOT}/videos -o ${CLIPS_BIN}
    DEPENDS ${VHS_ROOT}/tools/clips.py ${CLIP_VIDEOS}
    COMMENT "Baking videos/ into clips.bin")
add_custom_target(clips_bin DEPENDS ${CLIPS_BIN})

# Flashed at 0x10F00000 (scripts/flash_rp2350.sh data): the mock game image as a data tape.
set(DATATAPE_BIN ${CMAKE_CURRENT_BINARY_DIR}/datatape.bin)
add_custom_command(
    OUTPUT ${DATATAPE_BIN}
    COMMAND ${VHS_PYTHON} ${VHS_ROOT}/tools/datatape.py bake -o ${DATATAPE_BIN}
    DEPENDS ${VHS_ROOT}/tools/datatape.py
    COMMENT "Baking the mock game into datatape.bin")
add_custom_target(datatape_bin DEPENDS ${DATATAPE_BIN})

add_executable(vhs_pattern src/main.c src/field.S)
add_dependencies(vhs_pattern field_bin)
set_source_files_properties(src/field.S PROPERTIES
    COMPILE_DEFINITIONS "FIELD_BIN=\"${FIELD_BIN}\""
    OBJECT_DEPENDS ${FIELD_BIN})
pico_generate_pio_header(vhs_pattern ${CMAKE_CURRENT_LIST_DIR}/src/tvout.pio)
target_link_libraries(vhs_pattern pico_stdlib hardware_pio hardware_dma hardware_clocks)
pico_set_binary_type(vhs_pattern copy_to_ram)
pico_enable_stdio_usb(vhs_pattern 0)
pico_enable_stdio_uart(vhs_pattern 0)
pico_add_extra_outputs(vhs_pattern)

add_executable(vhs_streams src/streams.c src/pad.c src/vcr.c src/sd.c src/tape_store.c)
pico_generate_pio_header(vhs_streams ${CMAKE_CURRENT_LIST_DIR}/src/tvout.pio)
pico_generate_pio_header(vhs_streams ${CMAKE_CURRENT_LIST_DIR}/src/sdio.pio)
pico_generate_pio_header(vhs_streams ${CMAKE_CURRENT_LIST_DIR}/src/onewire.pio)  # from GameConsoleV2
pico_generate_pio_header(vhs_streams ${CMAKE_CURRENT_LIST_DIR}/src/i2s.pio)      # from GameConsoleV2
target_link_libraries(vhs_streams pico_stdlib pico_multicore hardware_pio hardware_dma hardware_clocks hardware_flash hardware_pwm)
pico_set_binary_type(vhs_streams copy_to_ram)
pico_enable_stdio_usb(vhs_streams 0)
pico_enable_stdio_uart(vhs_streams 0)
pico_add_extra_outputs(vhs_streams)
