#ifndef DATA_H
#define DATA_H

#include "util/points.h"


struct SmallRadarData {
    int Id;
    Vector3d Pos;
};

struct BigRadarData : public SmallRadarData {
    Vector3d Speed;
};


#endif // DATA_H
