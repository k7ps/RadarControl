#ifndef PROTO_H
#define PROTO_H

#include <fcntl.h>
#include <string>

#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/text_format.h>


template<class T>
T ParseProtoFromFile(std::string filename) {
    T res;
    int fd = open(filename.data(), O_RDONLY);
    google::protobuf::io::FileInputStream fstream(fd);
    google::protobuf::TextFormat::Parse(&fstream, &res);
    return res;
}


#endif // PROTO_H
