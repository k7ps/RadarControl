#ifndef RADAR_CONTROLLER_H
#define RADAR_CONTROLLER_H

#include "data.h"
#include "proto/generated/params.pb.h"
#include "util/points.h"
#include "util/timer.h"
#include "util/util.h"
#include <vector>


namespace RC {

    class Target {
    public:
        Target(int id, double presetPriority, double deathTime, const Proto::Parameters& params);
        Target(const BigRadarData& data, double deathTime, const Proto::Parameters& params);

        void SmallRadarUpdate(Vector3d pos);
        void BigRadarUpdate(Vector3d pos, Vector3d speed);

        int GetId() const { return Id; }
        double GetPriority() const { return Priority; }
        double GetPresetPriority() const { return PresetPriority; }
        void SetPriority(double p) { Priority = p; }

        Vector3d GetPosition() const { return Pos; }
        Vector3d GetFilteredPosition() const { return FilteredPos; }

        bool HavePreciseSpeed() const { return CurrSmallRadarMeasureCount >= SmallRadarMeasureCount; }
        Vector3d GetFilteredSpeed() const {
            return (CurrSmallRadarMeasureCount < ApproxSmallRadarMeasureCount ? SpeedFromBigRadar : FilteredSpeed);
        }
        int GetMeasureCountToPreciseSpeed() const {
            return std::max(0, SmallRadarMeasureCount - CurrSmallRadarMeasureCount);
        }

        bool IsDead() const { return (double) Timer.GetElapsedTimeAsMs() >= DeathTime; }
        void SetFollowed(bool f) { IsFollowedFlag = f; }
        bool IsFollowed() const { return IsFollowedFlag; }
        void SetIsRocketLaunched(bool f) { IsRocketLaunchedFlag = f; }
        bool IsRocketLaunched() const { return IsRocketLaunchedFlag; }
        bool CanBeFollowed() const {
            return EntryPoint != Vector3d::Zero() && ApproximateMeetingPoint != Vector3d::Zero();
        }
        bool CanLaunchRocket() const { return CurrSmallRadarMeasureCount >= SmallRadarMeasureCount; }

        void SetEntryPoint(Vector3d p) { EntryPoint = p; }
        Vector3d GetEntryPoint() const { return EntryPoint; }
        double GetEntryAngle() const { return (EntryPoint == Vector3d::Zero() ? -1 : GetPhi(EntryPoint)); }

        void SetApproximateMeetingPoint(Vector3d p) { ApproximateMeetingPoint = p; }
        Vector3d GetApproximateMeetingPoint() const { return ApproximateMeetingPoint; }
        double GetMeetAngle() const {
            return (ApproximateMeetingPoint == Vector3d::Zero() ? -1 : GetPhi(ApproximateMeetingPoint));
        }

        bool NeedToUpdateEntryPoint() const { return NeedToUpdateEntryPointFlag; }
        void SetNeedToUpdateEntryPoint(bool f) { NeedToUpdateEntryPointFlag = f; }

        bool NeedToUpdateMeetingPoint() const { return NeedToUpdateMeetingPointFlag; }
        void SetNeedToUpdateMeetingPoint(bool f) { NeedToUpdateMeetingPointFlag = f; }

        bool IsInSector(double rad, double angView, double angPos) const;
        double GetTimeToEntryPoint() const {
            return Distance(FilteredPos, EntryPoint) / SqrtOfSumSquares(GetFilteredSpeed());
        }
        double GetTimeToMeetingPoint() const {
            return Distance(FilteredPos, ApproximateMeetingPoint) / SqrtOfSumSquares(GetFilteredSpeed());
        }

        std::string DebugString() const;

    private:
        int Id;
        double Priority = -1;
        double PresetPriority = -1;

        Vector3d Pos;
        Vector3d FilteredPos;
        Vector3d SpeedFromBigRadar;
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
        int ApproxSmallRadarMeasureCount;
    };

}


class RadarController {
public:
    struct Result {
        double Angle;
        std::vector<int> FollowedTargetIds;
        std::vector<std::pair<Vector3d, int>> MeetingPointsAndTargetIds;
    };

    RadarController(const Proto::Parameters& params, double startAngle);

    void Process(const std::vector<BigRadarData>&, const std::vector<SmallRadarData>&);

    Result GetAngleAndMeetingPoints();
    std::vector<Vector3d> GetEntryPoints() const;
    std::vector<Vector3d> GetApproximateMeetingPoints() const;
    std::map<int, double> GetPriorities() const;

    ~RadarController();

private:
    void TrySelectTargetToFollow();
    void RemoveDeadTargets();
    bool IsTargetInSector(const RC::Target* target) const;

private:
    const Proto::Parameters& Params;

    double RadarAnglePos;
    double RadarAngleTarget = -1;

    std::vector<RC::Target*> Targets;

    std::vector<int> FollowedTargetIds;
    std::vector<std::pair<Vector3d, int>> MeetingPointsAndTargetIds;

    SimpleTimer Timer;
};


#endif // RADAR_CONTROLLER_H
