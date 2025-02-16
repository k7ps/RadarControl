find_package(Protobuf REQUIRED)
find_package(absl REQUIRED)

set(GENERATED_DIR ${CMAKE_CURRENT_SOURCE_DIR}/generated)
set(SCHEMA_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/params.proto
    ${CMAKE_CURRENT_SOURCE_DIR}/scenario.proto
)
file(MAKE_DIRECTORY ${GENERATED_DIR})

foreach(proto ${SCHEMA_FILES})
    execute_process(COMMAND "protoc" --proto_path=${CMAKE_CURRENT_SOURCE_DIR} --cpp_out=${GENERATED_DIR} ${proto})
endforeach(proto)

file(GLOB PROTO_HDRS "${GENERATED_DIR}/*.pb.h")
file(GLOB PROTO_SRCS "${GENERATED_DIR}/*.pb.cc")

add_library(proto_lib ${PROTO_SRCS} ${PROTO_HDRS})

target_link_libraries(proto_lib PRIVATE
    ${PROTOBUF_LIBRARIES}
    absl::log_internal_check_op
    absl::flags
    utf8_validity
)
include_directories(${PROTOBUF_INCLUDE_DIRS})
