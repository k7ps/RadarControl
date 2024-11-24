#ifndef RADAR_CONTROLLER_H
#define RADAR_CONTROLLER_H

#include "lib/data.h"
#include "proto/generated/params.pb.h"
#include "util/points.h"
#include "util/timer.h"


namespace RC {

    class Target {
    public:
        Target(int id, double priority, double deathTime);
        Target(const BigRadarData& data, double deatTime);

        void Update(Vector3d pos);
        void Update(Vector3d pos, Vector3d speed);

        int GetId() const;
        double GetPriority() const;
        Vector3d GetPosition() const;

        bool HavePreciseSpeed() const;
        Vector3d GetSpeed() const;

        bool IsDead() const;
        void SetFollowed(bool f);
        bool IsFollowed() const;
        void SetIsRocketLaunched(bool f);
        bool IsRocketLaunched() const;

    private:
        int Id;
        double Priority;

        std::vector<Vector3d> Positions;
        std::vector<int> DeltaTimes;

        Vector3d Pos;
        Vector3d Speed;

        SimpleTimer Timer;
        double DeathTime;

        bool IsFollowedFlag = false;
        bool IsRocketLaunchedFlag = false;
        bool HavePreciseSpeedFlag = false;
    };

}


class RadarController {
public:
    struct Result {
        double Angle;
        std::vector<unsigned> FollowedTargetIds;
        std::vector<std::pair<Vector3d, unsigned>> MeetingPointsAndTargetIds;
    };

    RadarController(const Proto::Parameters& params);

    void Process(const std::vector<BigRadarData>&, const std::vector<SmallRadarData>&);

    Result GetAngleAndMeetingPoints();

private:
    RC::Target& GetTargetById(int id);
    void SelectTargetToFollow();
    void RemoveDeadTargets();

private:
    const Proto::Parameters& Params;

    double RadarAnglePos;
    double RadarAngleTarget = -1;

    std::vector<RC::Target> Targets;

    std::vector<unsigned> FollowedTargetIds;
    std::vector<std::pair<Vector3d, unsigned>> MeetingPointsAndTargetIds;

    SimpleTimer Timer;
};


#endif // RADAR_CONTROLLER_H
