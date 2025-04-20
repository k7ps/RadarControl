#ifndef DEFENCE_H
#define DEFENCE_H

#include "proto/generated/params.pb.h"
#include "util/points.h"
#include "util/timer.h"


class DefRocket {
public:
    DefRocket(Vector3d meetPoint, double speed, unsigned timeToLaunchMs);

    void UpdatePosition();

    Vector3d GetPosition() const;
    Vector3d GetMeetPoint() const;
    bool IsLaunched() const;
    bool IsExploded() const;

private:
    Vector3d Pos;
    Vector3d MeetPoint;
    Vector3d Speed;

    SimpleTimer Timer;
    SimpleTimer TestTimer;

    bool IsLaunchedFlag = false;
    bool IsExplodedFlag = false;
    unsigned TimeToLaunchMs;
};


class Defense {
public:
    Defense(const Proto::Parameters& params);

    void LaunchRockets(const std::vector<std::pair<Vector3d, int>>& meetPointsAndTargetIds);

    std::vector<int> GetDestroyedTargetsId();
    std::vector<Vector3d> GetRocketsPositions();
    std::vector<Vector3d> GetMeetPoints();

private:
    const Proto::Parameters& Params;

    std::vector<std::pair<DefRocket, int>> Rockets;
};


#endif // DEFENCE_H
