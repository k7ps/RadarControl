#ifndef RADAR_CONTROLLER_H
#define RADAR_CONTROLLER_H

#include "data.h"
#include "flat/generated/params.h"


class RadarController {
public:
    RadarController(const Flat::Parameters& params);

    void Process(const std::vector<BigRadarData>&);
    void Process(const std::vector<SmallRadarData>&);

    Result GetDeltaAngleAndTargets();

private:
    const Flat::Parameters& Params;

};


#endif // RADAR_CONTROLLER_H
