set(SIM_LIB_HEADERS
    simulator.h
)

set(SIM_LIB_SOURCES
    simulator.cpp
)

add_library(simulator_lib STATIC ${SIM_LIB_HEADERS} ${SIM_LIB_SOURCES})

target_include_directories(simulator_lib PRIVATE ${CMAKE_SOURCE_DIR} ${CMAKE_SOURCE_DIR}/tools)
