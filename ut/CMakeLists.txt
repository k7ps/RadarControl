set(UT_SOURCES
    calculate_angle.cpp
)

include(FetchContent)
FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG v1.16.0
)
FetchContent_MakeAvailable(googletest)

add_executable(ut ${UT_SOURCES})
target_include_directories(ut PRIVATE ${CMAKE_SOURCE_DIR})
target_link_libraries(ut PRIVATE radar_control gtest gtest_main)

include(GoogleTest)
gtest_discover_tests(ut)
