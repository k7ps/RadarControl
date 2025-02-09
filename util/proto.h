#ifndef PROTO_H
#define PROTO_H

#include <fcntl.h>
#include <string>

#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/text_format.h>

#include "proto/generated/params.pb.h"


template<class T>
T ParseProtoFromFile(const std::string& filename) {
    T res;
    int fd = open(filename.data(), O_RDONLY);
    google::protobuf::io::FileInputStream fstream(fd);
    google::protobuf::TextFormat::Parse(&fstream, &res);
    return res;
}

void PrepareParams(Proto::Parameters& params);


#endif // PROTO_H
