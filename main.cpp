#include <stdio.h>
#include <fcntl.h>
#include <string>
#include <sys/types.h>
#include <sys/stat.h> 
#include <unistd.h>
#include <iostream>
#include <fstream>

#include "data.pb.h"

int main() {
    GOOGLE_PROTOBUF_VERIFY_VERSION;

    std::string input_pipename = "input.fifo";
    mkfifo(input_pipename.data(), 0666);
    std::cout << "C++: started" << std::endl;


    if (fork() == 0) {
        std::ofstream file("input.fifo");
        for (int i=0; i<3; i++) {
            BigRadarTargetInfo info;
            info.set_id(i);
            info.set_fi(0.5);
            info.set_rad(1);
            info.set_priority(0.3);
            info.set_speedx(2);
            info.set_speedy(-2);

            std::cout << i << std::endl;

            std::string msg;
            info.SerializeToString(&msg);
            file << msg << std::flush;
            std::cout << "Wrote to file\n\n";
            // file.close();

            sleep(3);
        }
    } else {
        BigRadarTargetInfo info;
        std::ifstream file("input.fifo");
        for (int i=0; i<3; i++) {
            info.ParseFromIstream(&file);
            std::cout << "Read:\n" << info.DebugString() << std::endl;
        }
        std::cout << "ended\n";
    }

    std::cout << "C++: ended" << std::endl;

    return 0;
}
