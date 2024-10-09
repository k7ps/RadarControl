set(RC_BIN_PROJECT_NAME "RadarControl")
project(${RC_BIN_PROJECT_NAME})

set(RC_BIN_SOURCES
    main.cpp
)

add_executable(${RC_BIN_PROJECT_NAME} ${RC_BIN_SOURCES})

target_include_directories(${RC_BIN_PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/radar_control)

target_link_libraries(${RC_BIN_PROJECT_NAME} PRIVATE radar_control_lib)
