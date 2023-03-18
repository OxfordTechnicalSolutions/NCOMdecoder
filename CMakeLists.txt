#
# Copyright © 2023 Glydways, Inc.
# https://glydways.com/
#

#
# The NComRx an SDK header and source from the Oxford Technical Solutions
# to interface with their INS devices, and intended to be directly included
# by drivers using it.
#
# Source for the library:
# https://github.com/OxfordTechnicalSolutions/NCOMdecoder/tree/master/nav
#
# Original OxTS ROS driver:
# https://github.com/OxfordTechnicalSolutions/oxts_ros2_driver
#

# The library files as written contain many readability errors, so
# we're going to turn off warnings for this library.
set(NCOM_COMPILE_OPTIONS "-w")

file(GLOB NCOM_HEADERS
    "nav/*.h"
    )


file(GLOB NCOM_SOURCES
    "nav/*.c"
)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

glyd_add_library(
    oxts_ncom_sdk
    ALIAS
        third_party::oxts_ncom_sdk
    HEADERS
        ${NCOM_HEADERS}
    SOURCES
        ${NCOM_SOURCES}
    INCLUDES
        ${CMAKE_CURRENT_SOURCE_DIR}
    COMPILE_OPTIONS
        ${NCOM_COMPILE_OPTIONS}
    )
