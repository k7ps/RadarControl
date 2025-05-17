#ifndef PROTO_H
#define PROTO_H


#include "proto/generated/params.pb.h"

#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/text_format.h>

#include <fcntl.h>
#include <string>


template<class T>
T ParseProtoFromFile(const std::string& filename) {
    T res;
    int fd = open(filename.data(), O_RDONLY);
    google::protobuf::io::FileInputStream fstream(fd);
    google::protobuf::TextFormat::Parse(&fstream, &res);
    return res;
}

void PrepareParams(Proto::Parameters& params);

template<class T>
std::vector<std::pair<double, double>> SegmentsFromProto(const T& protoMsg) {
    std::vector<std::pair<double, double>> res;
    for (const auto& seg : protoMsg) {
        res.emplace_back(seg.start(), seg.end());
    }
    return res;
}


#endif // PROTO_H
