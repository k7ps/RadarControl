set(UTIL_HEADERS
    proto.h
    util.h
)

set(UTIL_SOURCES
    util.cpp
)

add_library(util_lib STATIC ${UTIL_HEADERS} ${UTIL_SOURCES})
