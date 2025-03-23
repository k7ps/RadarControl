#ifndef DATA_H
#define DATA_H

#include "util/points.h"


struct SmallRadarData {
    int Id;
    Vector3d Pos;
};

struct BigRadarData : public SmallRadarData {
    Vector3d Speed;
    double PresetPriority = -1;
};


#endif // DATA_H
