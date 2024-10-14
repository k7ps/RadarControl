#ifndef RADAR_CONTROLLER_H
#define RADAR_CONTROLLER_H

#include "data.h"
#include "proto/generated/params.pb.h"


class RadarController {
public:
    RadarController(const Proto::Parameters& params);

    void Process(const std::vector<BigRadarData>&);
    void Process(const std::vector<SmallRadarData>&);

    Result GetDeltaAngleAndTargets();

private:
    const Proto::Parameters& Params;

};


#endif // RADAR_CONTROLLER_H
