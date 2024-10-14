set(PROTO_PATH ${CMAKE_SOURCE_DIR}/proto)
set(GENER_PATH ${CMAKE_SOURCE_DIR}/proto/generated)

set(PROTOS
    ${CMAKE_SOURCE_DIR}/proto/params.proto
)

execute_process(COMMAND "mkdir" -p ${GENER_PATH})

foreach(proto ${PROTOS})
    execute_process(COMMAND "protoc" --proto_path=${PROTO_PATH} --cpp_out=${GENER_PATH} ${proto})
endforeach(proto)

file(GLOB PROTO_HDRS "${GENER_PATH}/*.pb.h")
file(GLOB PROTO_SRCS "${GENER_PATH}/*.pb.cc")

find_package(Protobuf REQUIRED)

add_library(proto_lib ${PROTO_HDRS} ${PROTO_SRCS})
target_link_libraries(proto_lib ${Protobuf_LIBRARIES})
