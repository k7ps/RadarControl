set(UTIL_HEADERS
    points.h
    proto.h
    timer.h
    util.h
)

set(UTIL_SOURCES
    points.cpp
    proto.cpp
    util.cpp
)

add_library(util_lib STATIC ${UTIL_HEADERS} ${UTIL_SOURCES})

target_include_directories(util_lib PRIVATE ${CMAKE_SOURCE_DIR})
target_link_libraries(util_lib PRIVATE proto_lib)
