set(RC_LIB_HEADERS
    data.h
    radar_controller.h
)

set(RC_LIB_SOURCES
    radar_controller.cpp
)

add_library(radar_control_lib STATIC ${RC_LIB_HEADERS} ${RC_LIB_SOURCES})

target_include_directories(radar_control_lib PRIVATE ${CMAKE_SOURCE_DIR})
target_link_libraries(radar_control_lib PRIVATE util_lib proto_lib)
