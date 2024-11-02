#ifndef DATA_H
#define DATA_H

#include "util/points.h"


struct SmallRadarData {
    unsigned int Id;
    double Priority;
    double Rad;
    double Ang;
    double H;
};

struct BigRadarData : public SmallRadarData {
    Vector3d Speed;
};


#endif // DATA_H
