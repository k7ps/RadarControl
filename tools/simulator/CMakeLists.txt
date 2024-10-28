set(SIM_LIB_HEADERS
    simulator.h
)

set(SIM_LIB_SOURCES
    simulator.cpp
)

include_directories(${CMAKE_SOURCE_DIR})
add_library(simulator_lib STATIC ${SIM_LIB_HEADERS} ${SIM_LIB_SOURCES})

target_include_directories(simulator_lib PRIVATE ${CMAKE_SOURCE_DIR})
target_link_libraries(simulator_lib PRIVATE radar_control_lib util_lib)
