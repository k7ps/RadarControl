set(RC_EXEC_NAME "RadarControl")

set(RC_BIN_SOURCES
    main.cpp
)

add_executable(${RC_EXEC_NAME} ${RC_BIN_SOURCES})

target_include_directories(${RC_EXEC_NAME} PRIVATE ${CMAKE_SOURCE_DIR} ${CMAKE_SOURCE_DIR}/radar_control)
