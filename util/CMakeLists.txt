set(UTIL_HEADERS
    json.h
    util.h
)

set(UTIL_SOURCES
    json.cpp
    util.cpp
)

find_package(jsoncpp REQUIRED)

add_library(util_lib STATIC ${UTIL_HEADERS} ${UTIL_SOURCES})
target_sources(util_lib PUBLIC ${UTIL_HEADERS} ${UTIL_SOURCES})

target_link_libraries(util_lib PRIVATE jsoncpp_lib)
