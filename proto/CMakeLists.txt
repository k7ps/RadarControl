set(GENERATED_PATH ${CMAKE_CURRENT_SOURCE_DIR}/generated)

set(PROTOS
    params.proto
)

# create dir for proto sources
execute_process(COMMAND "mkdir" -p ${GENERATED_PATH})

# generate proto sources
foreach(proto ${PROTOS})
    execute_process(COMMAND "protoc" --proto_path=${CMAKE_CURRENT_SOURCE_DIR} --cpp_out=${GENERATED_PATH} ${proto})
endforeach(proto)

# find proto sources and add to target
file(GLOB PROTO_HDRS "${GENERATED_PATH}/*.pb.h")
file(GLOB PROTO_SRCS "${GENERATED_PATH}/*.pb.cc")

find_package(Protobuf REQUIRED)

add_library(proto_lib ${PROTO_HDRS} ${PROTO_SRCS})
target_link_libraries(proto_lib ${Protobuf_LIBRARIES})
