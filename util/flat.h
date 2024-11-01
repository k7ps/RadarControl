#ifndef FLAT_H
#define FLAT_H

#include "flat/generated/params.h"
#include "flatbuffers/idl.h"


const Flat::Parameters* ParseParameters(
    flatbuffers::Parser& parser,
    const std::string& jsonFile,
    const std::string& schemaFile
);


#endif // FLAT_H
