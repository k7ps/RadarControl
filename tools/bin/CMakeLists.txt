set(TOOLS_EXEC_NAME "Tools")

set(TOOLS_BIN_SOURCES
    main.cpp
)

include(FetchContent)
FetchContent_Declare(
    argparse
    GIT_REPOSITORY https://github.com/p-ranav/argparse.git
    GIT_TAG v3.2
)
FetchContent_MakeAvailable(argparse)

add_executable(${TOOLS_EXEC_NAME} ${TOOLS_BIN_SOURCES})

target_include_directories(
    ${TOOLS_EXEC_NAME}
    PRIVATE ${CMAKE_SOURCE_DIR} ${CMAKE_SOURCE_DIR}/tools ${argparse_SOURCE_DIR}/include
)
target_link_libraries(
    ${TOOLS_EXEC_NAME}
    PRIVATE radar_controller defense_lib simulator_lib visualizer_lib util_lib argparse
)
