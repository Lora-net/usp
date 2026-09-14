# SPDX-License-Identifier: BSD-3-Clause-Clear
#
# CMakeLists.txt for building all examples
# Use: cmake -S examples/main_examples -B build -DBOARD=... -DRAC_RADIO=...
#
# To build a single example instead, point cmake to its folder:
#   cmake -S examples/main_examples/periodical_uplink_example -B build -DBOARD=... -DRAC_RADIO=...

cmake_minimum_required(VERSION 3.25)

################################################################################
# Toolchain and Project

include(${CMAKE_CURRENT_SOURCE_DIR}/../toolchain.cmake)
project(all_examples LANGUAGES C ASM)

################################################################################
# Requirements for building all examples
# For minimal builds, build individual examples instead

# Enable full LBM features to support all examples (hw_modem, geolocation, etc.)
set(ENABLE_FULL_LBM_FEATURES ON CACHE BOOL "")

# Enable FLRC protocol
set(USE_FLRC_PROTOCOL ON CACHE BOOL "")

include(${CMAKE_CURRENT_SOURCE_DIR}/../common.cmake)

################################################################################
# Include all example subdirectories


file(GLOB children RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/*)
foreach(child ${children})
    if(IS_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${child})
        if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${child}/CMakeLists.txt)
            add_subdirectory(${child})
        else()
            file(GLOB childrenbis RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}/${child} ${CMAKE_CURRENT_SOURCE_DIR}/${child}/*)
            foreach(childbis ${childrenbis})
                if(IS_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${child}/${childbis})
                    if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${child}/${childbis}/CMakeLists.txt)
                        add_subdirectory(${child}/${childbis})
                    endif()
                endif()
            endforeach()
        endif()
    endif()
endforeach()


################################################################################
# Create "all_examples" target

add_custom_target(all_examples
    DEPENDS ${REGISTERED_EXAMPLES}
    COMMENT "Building all registered examples..."
)

################################################################################
# Display information

message(STATUS "")
message(STATUS "Available examples:")

# Find the longest example name for alignment
set(max_name_length 0)
foreach(example IN LISTS REGISTERED_EXAMPLES)
    string(LENGTH "${example}" name_length)
    if(name_length GREATER max_name_length)
        set(max_name_length ${name_length})
    endif()
endforeach()

# Display each registered example with its description
foreach(example IN LISTS REGISTERED_EXAMPLES)
    string(LENGTH "${example}" name_length)
    math(EXPR padding_length "${max_name_length} - ${name_length}")
    string(REPEAT " " ${padding_length} padding)
    get_property(desc CACHE EXAMPLE_${example}_DESCRIPTION PROPERTY VALUE)
    message(STATUS "  - ${example}${padding} : ${desc}")
endforeach()

message(STATUS "")
message(STATUS "Build targets:")
message(STATUS "  - ninja all_examples    : Build all examples")
message(STATUS "  - ninja <example_name>  : Build specific example")
message(STATUS "  - ninja flash_<example> : Flash specific example to device")
message(STATUS "")