# SPDX-License-Identifier: BSD-3-Clause-Clear





if(RADIO_FAMILY STREQUAL "sx126x")
  set(SX126X_ENABLE_LR_FHSS true)
  if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/dev_env.cmake")
    include("${CMAKE_CURRENT_LIST_DIR}/dev_env.cmake")
  endif()
endif()

# UDP virtual radio has no driver to compile
if(NOT RADIO_FAMILY STREQUAL "udp_pf")
    add_subdirectory(${RADIO_FAMILY}_driver)
    set(radio_driver_library "${RADIO_FAMILY}_driver::${RADIO_FAMILY}_driver" CACHE STRING "")
endif()
