cmake_minimum_required(VERSION 3.13)

# A GCV2 game image for the VHS console, built with GameConsoleV2's cart SDK:
#   cmake -S cpu/roms/cinema -B cpu/roms/cinema/build -DCMAKE_TOOLCHAIN_FILE=$PWD/external/GameConsoleV2/cart/sdk/toolchain.cmake
set(GCV2_ROOT ${CMAKE_CURRENT_LIST_DIR}/../../../external/GameConsoleV2 CACHE PATH "GameConsoleV2 tree")
set(CART_SDK_DIR ${GCV2_ROOT}/cart/sdk)

project(cinema C ASM)

set(CMAKE_C_FLAGS "-mcpu=cortex-m33 -mthumb -mfloat-abi=softfp -mfpu=fpv5-sp-d16 -Os -ffunction-sections -fdata-sections -ffreestanding")
set(CMAKE_EXE_LINKER_FLAGS "-T${CART_SDK_DIR}/cart.ld -nostartfiles --specs=nano.specs --specs=nosys.specs -Wl,--gc-sections -Wl,--no-warn-rwx-segments")

add_executable(cinema.elf cinema.c)
target_include_directories(cinema.elf PRIVATE ${CART_SDK_DIR}/include ${CMAKE_CURRENT_LIST_DIR}/../../src)

add_custom_command(TARGET cinema.elf POST_BUILD
    COMMAND arm-none-eabi-objcopy -O binary $<TARGET_FILE:cinema.elf> ${CMAKE_CURRENT_BINARY_DIR}/cinema.bin
    COMMAND python3 ${GCV2_ROOT}/tools/pack_cart.py ${CMAKE_CURRENT_BINARY_DIR}/cinema.bin -t "CINEMA"
            -o ${CMAKE_CURRENT_BINARY_DIR}/cinema.cart
    COMMENT "Packing cinema.cart")
