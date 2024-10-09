#ifndef TEST_H
#define TEST_H

#include "data.h"


class RadarController {
public:
    RadarController();

    void Process(const std::vector<BigRadarData>&);
    void Process(const std::vector<SmallRadarData>&);

    Result GetDeltaAngleAndTargets();

private:

};

#endif // TEST_H
