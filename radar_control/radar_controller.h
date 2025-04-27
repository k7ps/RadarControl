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

        Vector3d GetUnfilteredPosition() const { return UnfilteredPos; }
        Vector3d GetPosition() const { return Pos; }
        double GetPosAngle() const { return CalculateAngle(Pos); }

        bool HavePreciseSpeed() const { return CurrSmallRadarMeasureCount >= SmallRadarMeasureCount; }
        Vector3d GetFilteredSpeed() const { return (CurrSmallRadarMeasureCount < ApproxSmallRadarMeasureCount ? SpeedFromBigRadar : FilteredSpeed); }
        int GetMeasureCountToPreciseSpeed() const { return std::max(0, SmallRadarMeasureCount - CurrSmallRadarMeasureCount); }

        bool IsDead() const { return (double) Timer.GetElapsedTimeAsMs() >= DeathTime; }
        void SetFollowed(bool f) { IsFollowedFlag = f; }
        bool IsFollowed() const { return IsFollowedFlag; }
        void SetIsRocketLaunched(bool f) { IsRocketLaunchedFlag = f; }
        bool IsRocketLaunched() const { return IsRocketLaunchedFlag; }
        bool CanBeFollowed() const { return EntryPoint != Vector3d::Zero() && ApproximateMeetPoint != Vector3d::Zero(); }
        bool CanLaunchRocket() const { return CurrSmallRadarMeasureCount >= SmallRadarMeasureCount; }

        void SetEntryPoint(Vector3d p) { EntryPoint = p; }
        Vector3d GetEntryPoint() const { return EntryPoint; }
        double GetEntryAngle() const { return (EntryPoint == Vector3d::Zero() ? -1 : CalculateAngle(EntryPoint)); }
        double GetTimeToEntryPoint() const { return Distance(Pos, EntryPoint) / SqrtOfSumSquares(GetFilteredSpeed()); }
        bool NeedToUpdateEntryPoint() const { return NeedToUpdateEntryPointFlag; }
        void SetNeedToUpdateEntryPoint(bool f) { NeedToUpdateEntryPointFlag = f; }

        void SetNearPoint(Vector3d p) { NearPoint = p; }
        Vector3d GetNearPoint() const { return NearPoint; }
        double GetNearAngle() const { return (NearPoint == Vector3d::Zero() ? -1 : CalculateAngle(NearPoint)); }
        double GetTimeToNearPoint() const { return Distance(Pos, NearPoint) / SqrtOfSumSquares(GetFilteredSpeed()); }
        bool NeedToUpdateNearPoint() const { return NeedToUpdateNearPointFlag; }
        void SetNeedToUpdateNearPoint(bool f) { NeedToUpdateNearPointFlag = f; }

        void SetApproximateMeetPoint(Vector3d p) { ApproximateMeetPoint = p; }
        Vector3d GetApproximateMeetPoint() const { return ApproximateMeetPoint; }
        double GetMeetAngle() const { return (ApproximateMeetPoint == Vector3d::Zero() ? -1 : CalculateAngle(ApproximateMeetPoint)); }
        double GetTimeToMeetPoint() const { return Distance(Pos, ApproximateMeetPoint) / SqrtOfSumSquares(GetFilteredSpeed()); }
        bool NeedToUpdateMeetPoint() const { return NeedToUpdateMeetPointFlag; }
        void SetNeedToUpdateMeetPoint(bool f) { NeedToUpdateMeetPointFlag = f; }

        bool IsInSector(double rad, double angView, double angPos) const;

        bool CanBeInRadarSector() const { return Distance(Pos, Vector3d::Zero()) <= SmallRadarRadius; }

        std::string DebugString() const;

    private:
        int Id;
        double Priority = -1;
        double PresetPriority = -1;
        double SmallRadarRadius;

        Vector3d UnfilteredPos;
        Vector3d Pos;
        Vector3d SpeedFromBigRadar;
        Vector3d FilteredSpeed;

        Vector3d EntryPoint;
        Vector3d NearPoint;
        Vector3d ApproximateMeetPoint;

        SimpleTimer Timer;
        double DeathTime;

        bool IsFollowedFlag = false;
        bool IsRocketLaunchedFlag = false;
        bool NeedToUpdateEntryPointFlag = false;
        bool NeedToUpdateNearPointFlag = false;
        bool NeedToUpdateMeetPointFlag= false;

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
        std::vector<std::pair<Vector3d, int>> MeetPointsAndTargetIds;
    };

    RadarController(const Proto::Parameters& params, double startAngle);

    void Process(const std::vector<BigRadarData>&, const std::vector<SmallRadarData>&);

    Result GetAngleAndMeetPoints();
    std::vector<Vector3d> GetEntryPoints() const;
    std::vector<Vector3d> GetApproximateMeetPoints() const;
    std::map<int, double> GetPriorities() const;

    ~RadarController();

private:
    void RemoveDeadTargets();
    bool IsTargetInRadarSector(const RC::Target* target) const;
    bool IsTargetInResponsibleSector(const RC::Target* target) const;

private:
    const Proto::Parameters& Params;

    RadarPos Pos;
    RadarTargetPos TargetPos;

    std::vector<RC::Target*> Targets;

    std::vector<int> FollowedTargetIds;
    std::vector<std::pair<Vector3d, int>> MeetPointsAndTargetIds;

    SimpleTimer Timer;
};


#endif // RADAR_CONTROLLER_H
