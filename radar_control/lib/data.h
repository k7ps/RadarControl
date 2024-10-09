#ifndef DATA_H
#define DATA_H

#include <stdint.h>
#include <vector>


struct SmallRadarData {
    uint32_t Id;
    double Priority;
    double Rad;
    double Angle;
};

struct BigRadarData : public SmallRadarData {
    double SpeedX;
    double SpeedY;
};

struct Result {
    double AngleDelta;
    std::vector<uint32_t> AttackedTargets;
};

#endif // DATA_H
