set(RC_HEADERS
    radar_controller.h
)

set(RC_SOURCES
    radar_controller.cpp
)

add_library(radar_controller STATIC ${RC_HEADERS} ${RC_SOURCES})

target_include_directories(radar_controller PUBLIC ${CMAKE_SOURCE_DIR} ${CMAKE_SOURCE_DIR}/radar_control)
target_link_libraries(radar_controller PRIVATE radar_control_lib)
