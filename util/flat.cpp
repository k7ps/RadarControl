#include "flat.h"

#include <iostream>
#include <fstream>

#include "flatbuffers/flatbuffers.h"
#include "flatbuffers/util.h"


const Flat::Parameters* ParseParameters(
    flatbuffers::Parser& parser,
    const std::string& jsonFile,
    const std::string& schemaFile
) {
    std::string json_content;
    if (!flatbuffers::LoadFile(jsonFile.c_str(), false, &json_content)) {
        std::cerr << "Failed to load JSON file" << std::endl;
        return nullptr;
    }

    std::string schema_content;
    if (!flatbuffers::LoadFile(schemaFile.c_str(), false, &schema_content)) {
        std::cerr << "Failed to load schema file" << std::endl;
        return nullptr;
    }

    if (!parser.Parse(schema_content.c_str())) {
        std::cerr << "Failed to parse schema" << std::endl;
        return nullptr;
    }

    if (!parser.Parse(json_content.c_str())) {
        std::cerr << "Failed to parse JSON" << std::endl;
        return nullptr;
    }

    const uint8_t* buf = parser.builder_.GetBufferPointer();
    return Flat::GetParameters(buf);
}
