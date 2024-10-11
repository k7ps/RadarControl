set(PROTO_PATH ${CMAKE_SOURCE_DIR}/proto)
set(GENER_PATH ${CMAKE_SOURCE_DIR}/generated)
file(GLOB PROTOS "*.proto")

foreach(proto ${PROTOS})
    EXECUTE_PROCESS(COMMAND "protoc" --proto_path=${PROTO_PATH} --cpp_out=${GENER_PATH} ${proto})
endforeach(proto)
