set(SIM_LIB_HEADERS
    simulator.h
)

set(SIM_LIB_SOURCES
    simulator.cpp
)

add_library(simulator_lib STATIC ${SIM_LIB_HEADERS} ${SIM_LIB_SOURCES})
target_sources(simulator_lib PUBLIC ${SIM_LIB_HEADERS} ${SIM_LIB_SOURCES})
