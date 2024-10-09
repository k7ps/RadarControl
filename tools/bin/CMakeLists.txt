set(TOOLS_PROJECT_NAME "Tools")
project(${TOOLS_PROJECT_NAME})

set(TOOLS_BIN_SOURCES
    main.cpp
)

add_executable(${TOOLS_PROJECT_NAME} ${TOOLS_BIN_SOURCES})

target_include_directories(${TOOLS_PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR} ${CMAKE_SOURCE_DIR}/tools)

target_link_libraries(${TOOLS_PROJECT_NAME} PRIVATE radar_control_lib simulator_lib visualizer_lib)
