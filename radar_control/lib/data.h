#ifndef DATA_H
#define DATA_H

#include <vector>


struct SmallRadarData {
    unsigned int Id;
    float Priority;
    float X;
    float Y;
};

struct BigRadarData : public SmallRadarData {
    float SpeedX;
    float SpeedY;
};

struct Result {
    float AngleDelta;
    std::vector<unsigned int> AttackedTargets;
};


#endif // DATA_H
