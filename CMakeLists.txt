cmake_minimum_required(VERSION 3.15)
set(CMAKE_CXX_STANDARD 17)

set(ProjectName "RadarControl")
project(${ProjectName})

find_package(Protobuf REQUIRED)
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS data.proto)

add_executable(${ProjectName} 
    main.cpp
    ${PROTO_SRCS}
    ${PROTO_HDRS}
)

target_link_libraries(${ProjectName} PRIVATE protobuf::libprotobuf)
