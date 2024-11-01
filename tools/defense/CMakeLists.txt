set(DEF_LIB_HEADERS
    defense.h
)

set(DEF_LIB_SOURCES
    defense.cpp
)

add_library(defense_lib STATIC ${DEF_LIB_HEADERS} ${DEF_LIB_SOURCES})

target_include_directories(defense_lib PRIVATE ${CMAKE_SOURCE_DIR} ${CMAKE_SOURCE_DIR}/tools)
