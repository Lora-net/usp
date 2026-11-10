# SPDX-License-Identifier: BSD-3-Clause-Clear

# This CMake toolchain file describes how to cross-compile for 64-bit Linux ARM targets
# (e.g., Raspberry Pi 4/5 running a 64-bit OS such as Debian Trixie)
#
# Requires the ARM GNU Toolchain (aarch64-linux-gnu) in PATH.

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)
set(CMAKE_CROSSCOMPILING 1)

# Toolchain prefix for ARM GNU Toolchain 64-bit
set(TOOLCHAIN_CROSS_PREFIX "aarch64-linux-gnu-" CACHE STRING "Cross compiler toolchain prefix")

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

# Search paths: never look in the host system for programs,
# always look in the sysroot for libraries and headers.
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

# Compiler flags for AArch64 (ARMv8-A)
# -march=armv8-a targets Raspberry Pi 4 (Cortex-A72) and Pi 5 (Cortex-A76)
# NEON/FP are mandatory in AArch64, no -mfpu/-mfloat-abi flags needed.
set(C_FLAGS_COMMON "\
-march=armv8-a \
-fdata-sections -ffunction-sections \
")

set(CMAKE_C_FLAGS_INIT   "${C_FLAGS_COMMON}")
set(CMAKE_CXX_FLAGS_INIT "${C_FLAGS_COMMON}")

# Linker flags to remove unused sections
set(CMAKE_EXE_LINKER_FLAGS_INIT "-Wl,--gc-sections")
