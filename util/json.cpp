#include "json.h"

#include <fstream>
#include <iostream>


Json::Value ParseJsonFile(std::string filename) {
    std::ifstream file(filename);
    Json::Value res;
    if (file.is_open()) {
        file >> res;
    } else {
        std::cout << "Error while opening \'" << filename << "\'\n";
    }
    return res;
}
