set(SIM_EXEC_NAME "RadarControl")

set(SIM_HEADERS
    defense.h
    simulator.h
    visualizer.h
)

set(SIM_SOURCES
    defense.cpp
    main.cpp
    simulator.cpp
    visualizer.cpp
)

include(FetchContent)
FetchContent_Declare(
    argparse
    GIT_REPOSITORY https://github.com/p-ranav/argparse.git
    GIT_TAG v3.2
)
FetchContent_MakeAvailable(argparse)

find_package(raylib REQUIRED PATHS "/home/k1ps/raylib-5.0")
include_directories(/home/k1ps/raylib-cpp-5.0.2/include)

add_executable(${SIM_EXEC_NAME} ${SIM_HEADERS} ${SIM_SOURCES})

target_include_directories(${SIM_EXEC_NAME} PRIVATE ${CMAKE_SOURCE_DIR} ${argparse_SOURCE_DIR}/include)
target_link_libraries(${SIM_EXEC_NAME}PRIVATE radar_control util_lib argparse raylib)
