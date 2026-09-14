# SPDX-License-Identifier: BSD-3-Clause-Clear
# RF Certification Examples (one for each region)

cmake_minimum_required(VERSION 3.25)

set(DESCRIPTION "RF certification")

################################################################################
# Toolchain and Project

include(${CMAKE_CURRENT_SOURCE_DIR}/../../toolchain.cmake)
project(rf_certification LANGUAGES C ASM DESCRIPTION "${DESCRIPTION}")
include(${CMAKE_CURRENT_SOURCE_DIR}/../../common.cmake)

################################################################################
# Options

set(NOTIFICATION_MODE "NOTIFICATIONS_OFF" CACHE STRING "Notification mode (ON or OFF)")
set(APP_MODE "APP_MODE_CERTIFICATION" CACHE STRING "App mode (CERTIFICATION or NOTIFICATION)")

################################################################################
# Create targets

set(RF_CERT_SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/main_rf_certification.c
    ${CMAKE_CURRENT_SOURCE_DIR}/app_rf_certification.c
    ${CMAKE_CURRENT_SOURCE_DIR}/app_notification.c
)

# ETSI Region
add_executable(rf_certification_etsi ${RF_CERT_SOURCES})
setup_example_target(rf_certification_etsi)
register_example(rf_certification_etsi "${DESCRIPTION} (ETSI region)")
target_include_directories(rf_certification_etsi PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_definitions(rf_certification_etsi PRIVATE
    RF_CERT_REGION=RF_CERT_REGION_ETSI
    NOTIFICATION_MODE=${NOTIFICATION_MODE}
    APP_MODE=${APP_MODE}
)

# ARIB Region
add_executable(rf_certification_arib ${RF_CERT_SOURCES})
setup_example_target(rf_certification_arib)
register_example(rf_certification_arib "${DESCRIPTION} (ARIB region)")
target_include_directories(rf_certification_arib PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_definitions(rf_certification_arib PRIVATE
    RF_CERT_REGION=RF_CERT_REGION_ARIB
    NOTIFICATION_MODE=${NOTIFICATION_MODE}
    APP_MODE=${APP_MODE}
)

# FCC Region
add_executable(rf_certification_fcc ${RF_CERT_SOURCES})
setup_example_target(rf_certification_fcc)
register_example(rf_certification_fcc "${DESCRIPTION} (FCC region)")
target_include_directories(rf_certification_fcc PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_definitions(rf_certification_fcc PRIVATE
    RF_CERT_REGION=RF_CERT_REGION_FCC
    NOTIFICATION_MODE=${NOTIFICATION_MODE}
    APP_MODE=${APP_MODE}
)

# Build all regions
add_custom_target(rf_certification DEPENDS
    rf_certification_etsi
    rf_certification_arib
    rf_certification_fcc
)