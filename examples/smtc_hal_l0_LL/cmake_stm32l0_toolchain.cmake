# SPDX-License-Identifier: BSD-3-Clause-Clear

# This CMake toolchain file describes how to build for STM32L476xx

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)
SET(CMAKE_CROSSCOMPILING 1)

# Toolchain prefix for ARM GNU Toolchain
set(TOOLCHAIN_CROSS_PREFIX "arm-none-eabi-" CACHE STRING "Cross compiler toolchain prefix")

set(CMAKE_C_COMPILER   ${TOOLCHAIN_CROSS_PREFIX}gcc)
set(CMAKE_CXX_COMPILER ${TOOLCHAIN_CROSS_PREFIX}g++)
set(CMAKE_ASM_COMPILER ${TOOLCHAIN_CROSS_PREFIX}gcc)
set(CMAKE_SIZE         ${TOOLCHAIN_CROSS_PREFIX}size)


set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)


set(C_FLAGS_COMMON "\
-mcpu=cortex-m0plus -mthumb -mabi=aapcs -fno-builtin \
\
-fno-unroll-loops -ffast-math -ftree-vectorize -fomit-frame-pointer \
-fdata-sections -ffunction-sections -falign-functions=4 \
-DSTM32L073xx -DUSE_FULL_LL_DRIVER \
"
)

set(CMAKE_C_FLAGS_INIT   "${C_FLAGS_COMMON}")
set(CMAKE_ASM_FLAGS_INIT "${C_FLAGS_COMMON} -x assembler-with-cpp")

set(BOARD_LDSCRIPT ${CMAKE_CURRENT_LIST_DIR}/../mcu_drivers/core/STM32L0xx/stm32l073xx_flash.ld)
set(CMAKE_EXE_LINKER_FLAGS_INIT "--specs=nano.specs --specs=nosys.specs -T${BOARD_LDSCRIPT} -lnosys -lstdc++ -lsupc++ -lc -Wl,--gc-sections -Wl,--print-memory-usage")
