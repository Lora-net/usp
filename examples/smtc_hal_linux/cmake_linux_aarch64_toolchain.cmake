# SPDX-License-Identifier: BSD-3-Clause-Clear

# This CMake toolchain file describes how to cross-compile for 64-bit Linux ARM targets
# (e.g., Raspberry Pi 4/5 running a 64-bit OS such as Debian Trixie)
#
# Requires the ARM GNU Toolchain (aarch64-none-linux-gnu) in PATH.
# Source ~/raspberry/setup_rpi.sh before invoking cmake.

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)
set(CMAKE_CROSSCOMPILING 1)

# Toolchain prefix for ARM GNU Toolchain 64-bit
set(CROSS_PREFIX aarch64-none-linux-gnu-)

# Specify the cross-compiler
set(CMAKE_C_COMPILER   ${CROSS_PREFIX}gcc)
set(CMAKE_CXX_COMPILER ${CROSS_PREFIX}g++)
set(CMAKE_ASM_COMPILER ${CROSS_PREFIX}gcc)
set(CMAKE_AR           ${CROSS_PREFIX}ar)
set(CMAKE_LINKER       ${CROSS_PREFIX}ld)
set(CMAKE_NM           ${CROSS_PREFIX}nm)
set(CMAKE_OBJCOPY      ${CROSS_PREFIX}objcopy)
set(CMAKE_OBJDUMP      ${CROSS_PREFIX}objdump)
set(CMAKE_STRIP        ${CROSS_PREFIX}strip)
set(CMAKE_RANLIB       ${CROSS_PREFIX}ranlib)

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
