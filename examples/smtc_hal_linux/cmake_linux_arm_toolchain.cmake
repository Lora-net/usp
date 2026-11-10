# SPDX-License-Identifier: BSD-3-Clause-Clear

# This CMake toolchain file describes how to cross-compile for Linux ARM targets
# (e.g., Raspberry Pi, BeagleBone, ARM development boards)

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_CROSSCOMPILING 1)

# Toolchain prefix for ARM Linux GNU Toolchain 32-bit
set(TOOLCHAIN_CROSS_PREFIX "arm-linux-gnueabihf-" CACHE STRING "Cross compiler toolchain prefix")

# Specify the cross-compiler
set(CMAKE_C_COMPILER   ${TOOLCHAIN_CROSS_PREFIX}gcc)
set(CMAKE_CXX_COMPILER ${TOOLCHAIN_CROSS_PREFIX}g++)
set(CMAKE_ASM_COMPILER ${TOOLCHAIN_CROSS_PREFIX}gcc)
set(CMAKE_AR           ${TOOLCHAIN_CROSS_PREFIX}ar)
set(CMAKE_LINKER       ${TOOLCHAIN_CROSS_PREFIX}ld)
set(CMAKE_NM           ${TOOLCHAIN_CROSS_PREFIX}nm)
set(CMAKE_OBJCOPY      ${TOOLCHAIN_CROSS_PREFIX}objcopy)
set(CMAKE_OBJDUMP      ${TOOLCHAIN_CROSS_PREFIX}objdump)
set(CMAKE_STRIP        ${TOOLCHAIN_CROSS_PREFIX}strip)
set(CMAKE_RANLIB       ${TOOLCHAIN_CROSS_PREFIX}ranlib)

# Where to look for the target environment
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# Compiler flags for ARM Cortex-A series
# These are optimized for ARMv7/ARMv8 targets (32-bit mode)
# Adjust -march and -mfpu flags based on your specific ARM target
set(C_FLAGS_COMMON "\
-march=armv7-a -mfpu=neon-vfpv4 -mfloat-abi=hard \
-fdata-sections -ffunction-sections \
")

set(CMAKE_C_FLAGS_INIT   "${C_FLAGS_COMMON}")
set(CMAKE_CXX_FLAGS_INIT "${C_FLAGS_COMMON}")

# Linker flags to remove unused sections
set(CMAKE_EXE_LINKER_FLAGS_INIT "-Wl,--gc-sections")



