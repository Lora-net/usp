# SPDX-License-Identifier: BSD-3-Clause-Clear

# This CMake toolchain file describes how to cross-compile for Linux ARM targets
# (e.g., Raspberry Pi, BeagleBone, ARM development boards)

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR arm)
set(CMAKE_CROSSCOMPILING 1)

# Specify the cross-compiler
set(CMAKE_C_COMPILER   arm-linux-gnueabihf-gcc)
set(CMAKE_CXX_COMPILER arm-linux-gnueabihf-g++)
set(CMAKE_ASM_COMPILER arm-linux-gnueabihf-gcc)
set(CMAKE_AR           arm-linux-gnueabihf-ar)
set(CMAKE_LINKER       arm-linux-gnueabihf-ld)
set(CMAKE_NM           arm-linux-gnueabihf-nm)
set(CMAKE_OBJCOPY      arm-linux-gnueabihf-objcopy)
set(CMAKE_OBJDUMP      arm-linux-gnueabihf-objdump)
set(CMAKE_STRIP        arm-linux-gnueabihf-strip)
set(CMAKE_RANLIB       arm-linux-gnueabihf-ranlib)

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



