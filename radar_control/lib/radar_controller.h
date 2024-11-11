#ifndef RADAR_CONTROLLER_H
#define RADAR_CONTROLLER_H

#include "data.h"
#include "flat/generated/params.h"
#include "util/points.h"
#include "util/timer.h"


namespace RC {

    class Target {
    public:
        Target();

        void Update(Vector3d pos, Vector3d speed);

        bool IsDead() const;

    private:
        int Id;
        double Priority;

        Vector3d Pos;
        Vector3d Speed;

        SimpleTimer DeathTimer;
    };

}


class RadarController {
public:
    struct Result {
        double Angle;
        std::vector<std::pair<Vector3d, unsigned>> MeetingPointsAndTargetIds;
    };

    RadarController(const Flat::Parameters& params);

    void Process(const std::vector<BigRadarData>&, const std::vector<SmallRadarData>&);

    Result GetAngleAndMeetingPoints();

private:
    const Flat::Parameters& Params;

    double RadarAnglePos;
    double RadarAngleTarget = -1;

    std::vector<unsigned> FollowedTargetIds;
    std::vector<std::pair<Vector3d, unsigned>> MeetingPointsAndTargetIds;

    SimpleTimer Timer;
};


#endif // RADAR_CONTROLLER_H
