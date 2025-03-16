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

        int GetId() const { return Id; }
        double GetPriority() const { return Priority; }
        Vector3d GetPosition() const { return Pos; }
        Vector3d GetFilteredPosition() const { return FilteredPos; }

        bool HavePreciseSpeed() const { return CurrSmallRadarMeasureCount >= SmallRadarMeasureCount; }
        Vector3d GetFilteredSpeed() const { return FilteredSpeed; }

        bool IsDead() const { return (double) Timer.GetElapsedTimeAsMs() >= DeathTime; }
        void SetFollowed(bool f) { IsFollowedFlag = f; }
        bool IsFollowed() const { return IsFollowedFlag; }
        void SetIsRocketLaunched(bool f) { IsRocketLaunchedFlag = f; }
        bool IsRocketLaunched() const { return IsRocketLaunchedFlag; }

        void SetEntryPoint(Vector3d p) { EntryPoint = p; }
        Vector3d GetEntryPoint() const { return EntryPoint; }

        void SetApproximateMeetingPoint(Vector3d p) { ApproximateMeetingPoint = p; }
        Vector3d GetApproximateMeetingPoint() const { return ApproximateMeetingPoint; }

        bool NeedToUpdateEntryPoint() const { return NeedToUpdateEntryPointFlag; }
        void SetNeedToUpdateEntryPoint(bool f) { NeedToUpdateEntryPointFlag = f; }

        bool NeedToUpdateMeetingPoint() const { return NeedToUpdateMeetingPointFlag; }
        void SetNeedToUpdateMeetingPoint(bool f) { NeedToUpdateMeetingPointFlag = f; }

        bool CanBeFollowed() const {
            return EntryPoint != Vector3d::Zero() && ApproximateMeetingPoint != Vector3d::Zero();
        }

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
    void TrySelectTargetToFollow();
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
