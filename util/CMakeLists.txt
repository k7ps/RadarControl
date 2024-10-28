set(UTIL_HEADERS
    flat.h
    util.h
)

set(UTIL_SOURCES
    util.cpp
)

add_library(util_lib STATIC ${UTIL_HEADERS} ${UTIL_SOURCES})
target_link_libraries(util_lib PRIVATE flatbuf_lib)
