find_package(Protobuf REQUIRED)

FILE(GLOB HDRS "*.pb.h")
FILE(GLOB SRCS "*.pb.cc")

ADD_LIBRARY(proto_lib ${HDRS} ${SRCS})
TARGET_LINK_LIBRARIES(proto_lib ${Protobuf_LIBRARIES})
