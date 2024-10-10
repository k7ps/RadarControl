set(RC_LIB_HEADERS
    data.h
    radar_controller.h
)

set(RC_LIB_SOURCES
    radar_controller.cpp
)

add_library(radar_control_lib STATIC ${RC_LIB_HEADERS} ${RC_LIB_SOURCES})
target_sources(radar_control_lib PUBLIC ${RC_LIB_HEADERS} ${RC_LIB_SOURCES})

target_link_libraries(radar_control_lib PRIVATE util_lib)
