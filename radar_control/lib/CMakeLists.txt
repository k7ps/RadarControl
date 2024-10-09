set(RC_LIB_NAME radar_control_lib)

set(RC_LIB_HEADERS
    test.h
)

set(RC_LIB_SOURCES
    test.cpp
)

add_library(${RC_LIB_NAME} STATIC ${RC_LIB_HEADERS} ${RC_LIB_SOURCES})
target_sources(${RC_LIB_NAME} PUBLIC ${RC_LIB_HEADERS} ${RC_LIB_SOURCES})
