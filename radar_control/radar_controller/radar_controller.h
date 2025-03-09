#ifndef RADAR_CONTROLLER_H
#define RADAR_CONTROLLER_H

#include "lib/data.h"
#include "proto/generated/params.pb.h"
#include "util/points.h"
#include "util/timer.h"
#include <vector>


namespace RC {

    class Target {
    public:
        Target(int id, double priority, double deathTime, const Proto::Parameters& params);
        Target(const BigRadarData& data, double deathTime, const Proto::Parameters& params);

        void SmallRadarUpdate(Vector3d pos);
        void BigRadarUpdate(Vector3d pos, Vector3d speed);

        int GetId() const;
        double GetPriority() const;
        Vector3d GetPosition() const;
        Vector3d GetFilteredPosition() const;

        bool HavePreciseSpeed() const;
        Vector3d GetFilteredSpeed() const;

        bool IsDead() const;
        void SetFollowed(bool f);
        bool IsFollowed() const;
        void SetIsRocketLaunched(bool f);
        bool IsRocketLaunched() const;

        void SetEntryPoint(Vector3d p);
        Vector3d GetEntryPoint() const;

        void SetApproximateMeetingPoint(Vector3d p);
        Vector3d GetApproximateMeetingPoint() const;

        bool NeedToUpdateEntryPoint() const;
        void SetNeedToUpdateEntryPoint(bool f);

        bool NeedToUpdateMeetingPoint() const;
        void SetNeedToUpdateMeetingPoint(bool f);

    private:
        void ABFilterIterate(double dt);

    private:
        int Id;
        double Priority;

        Vector3d Pos;
        Vector3d FilteredPos;
        Vector3d FilteredSpeed;

        Vector3d EntryPoint;
        Vector3d ApproximateMeetingPoint;

        SimpleTimer Timer;
        double DeathTime;

        bool IsFollowedFlag = false;
        bool IsRocketLaunchedFlag = false;
        bool NeedToUpdateEntryPointFlag = false;
        bool NeedToUpdateMeetingPointFlag= false;

        int CurrBigRadarMeasureCount = 0;
        int BigRadarMeasureCount;
        int CurrSmallRadarMeasureCount = 0;
        int SmallRadarMeasureCount;
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
    std::vector<Vector3d> GetEntryPoints() const;
    std::vector<Vector3d> GetApproximateMeetingPoints() const;

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
