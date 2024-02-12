cmake_minimum_required(VERSION 3.19.0)

project(NCOM2CSV)

if(${CMAKE_SYSTEM_NAME} MATCHES Darwin)
    message("Apple OS (Darwin) detected")
elseif(${CMAKE_SYSTEM_NAME} MATCHES Linux)
    message("Linux OS detected")
else()
    message(FATAL_ERROR "${CMAKE_SYSTEM_NAME} OS detected - currently not supported - EXIT")
endif()

include_directories(nav)
set(SOURCES nav/NComRxC.c example/NcomToCsv.c)

add_executable(ncom2csv ${SOURCES})
