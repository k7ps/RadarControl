set(UT_SOURCES
    calc_angle_1_target.cpp
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
target_link_libraries(ut PRIVATE radar_control_lib gtest gtest_main)

include(GoogleTest)
gtest_discover_tests(ut)
