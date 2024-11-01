#ifndef DEFENCE_H
#define DEFENCE_H

#include "flat/generated/params.h"
#include "radar_control/lib/data.h"
#include "util/timer.h"
#include "util/util.h"


class DefRocket {
public:
    DefRocket(Vector3d meetingPoint, double speed, unsigned timeToLaunchMs);

    void UpdatePosition();

    Vector3d GetPosition() const;
    bool IsLaunched() const;
    bool IsExploded() const;

private:
    Vector3d Pos;
    Vector3d MeetingPoint;
    Vector3d Speed;

    SimpleTimer Timer;

    bool IsLaunchedFlag = false;
    bool IsExplodedFlag = false;
    unsigned TimeToLaunchMs;
};


class Defense {
public:
    Defense(const Flat::Parameters& params);

    void LaunchRockets(const std::vector<std::pair<Vector3d, unsigned>>& meetingPointsAndTargetIds);

    std::vector<unsigned> GetDestroyedTargetsId();
    std::vector<Vector3d> GetRocketsPositions();

private:
    const Flat::Parameters& Params;

    std::vector<std::pair<DefRocket, unsigned>> Rockets;
};


#endif // DEFENCE_H
