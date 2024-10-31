#ifndef DATA_H
#define DATA_H

#include <vector>


struct SmallRadarData {
    unsigned int Id;
    double Priority;
    double Rad;
    double Ang;
    double H;
};

struct BigRadarData : public SmallRadarData {
    double SpeedX;
    double SpeedY;
    double SpeedZ;
};

struct Result {
    double AngleDelta;
    std::vector<unsigned int> AttackedTargets;
};


#endif // DATA_H
