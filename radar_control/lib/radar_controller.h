#ifndef RADAR_CONTROLLER_H
#define RADAR_CONTROLLER_H

#include "data.h"
#include "flat/generated/params.h"
#include "util/points.h"


class RadarController {
public:
    struct Result {
        double Angle;
        std::vector<std::pair<Vector3d, unsigned>> MeetingPointsAndTargetIds;
    };

    RadarController(const Flat::Parameters& params);

    void Process(const std::vector<BigRadarData>&);
    void Process(const std::vector<SmallRadarData>&);

    Result GetAngleAndMeetingPoints();

private:
    const Flat::Parameters& Params;

    double RadarAnglePos;

    Vector3d test = Vector3d(-1, 0, 0);
    bool isTested = false;
};


#endif // RADAR_CONTROLLER_H
