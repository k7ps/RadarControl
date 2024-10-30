#ifndef DATA_H
#define DATA_H

#include <vector>


struct SmallRadarData {
    unsigned int Id;
    float Priority;
    double X;
    double Y;
};

struct BigRadarData : public SmallRadarData {
    double SpeedX;
    double SpeedY;
};

struct Result {
    float AngleDelta;
    std::vector<unsigned int> AttackedTargets;
};


#endif // DATA_H
