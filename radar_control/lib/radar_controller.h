#ifndef TEST_H
#define TEST_H

#include "data.h"


class RadarController {
public:
    RadarController();

    void Process(BigRadarData);
    void Process(SmallRadarData);

    Result GetDeltaAngleAndTargets();

private:

};

#endif // TEST_H
