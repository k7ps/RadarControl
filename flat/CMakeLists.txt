find_package(FlatBuffers REQUIRED)

set(GENERATED_DIR ${CMAKE_CURRENT_SOURCE_DIR}/generated)
set(SCHEMA_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/params.fbs
)

file(MAKE_DIRECTORY ${GENERATED_DIR})

foreach(flatbuffer ${SCHEMA_FILES})
    execute_process(COMMAND "flatc" --cpp -o ${GENERATED_DIR} --filename-suffix "" ${flatbuffer})
endforeach(flatbuffer)

file(GLOB FLATBUF_HDRS "${GENERATED_DIR}/*")

add_library(flatbuf_lib INTERFACE)
target_sources(flatbuf_lib INTERFACE ${FLATBUF_HDRS})
target_link_libraries(flatbuf_lib INTERFACE flatbuffers)
target_include_directories(flatbuf_lib INTERFACE ${GENERATED_DIR})
