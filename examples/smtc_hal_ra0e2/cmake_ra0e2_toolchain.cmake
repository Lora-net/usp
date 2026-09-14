# SPDX-License-Identifier: BSD-3-Clause-Clear
# Toolchain file for Renesas FPB-RA0E2 (R7FA0E209 - ARM Cortex-M23)

# System configuration
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_PROCESSOR arm)

# Toolchain configuration
set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)
set(CMAKE_OBJCOPY arm-none-eabi-objcopy)
set(CMAKE_OBJDUMP arm-none-eabi-objdump)
set(CMAKE_SIZE arm-none-eabi-size)

# MCU-specific compiler flags
# Cortex-M23: ARMv8-M baseline architecture, NO FPU
set(MCU_FLAGS "-mcpu=cortex-m23 -mthumb")

# Common compiler flags
set(CMAKE_C_FLAGS_INIT "${MCU_FLAGS} -ffunction-sections -fdata-sections -Wall -Wextra")
set(CMAKE_CXX_FLAGS_INIT "${MCU_FLAGS} -ffunction-sections -fdata-sections -Wall -Wextra")
set(CMAKE_ASM_FLAGS_INIT "${MCU_FLAGS}")

# Linker flags
set(CMAKE_EXE_LINKER_FLAGS_INIT "${MCU_FLAGS} -specs=nano.specs -specs=nosys.specs -Wl,--gc-sections -Wl,--print-memory-usage")

# Prevent CMake from adding system directories
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# Disable CMake's standard library detection
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)
