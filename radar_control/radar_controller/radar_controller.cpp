#include "radar_controller.h"
#include "proto/generated/params.pb.h"
#include "util/points.h"
#include "util/util.h"
#include "lib/calculations.h"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

using namespace RC;


Target::Target(int id, double deathTime, const Proto::Parameters& params)
    : Id(id)
    , DeathTime(deathTime)
    , BigRadarMeasureCount(params.general().big_radar_measure_cnt())
    , SmallRadarMeasureCount(params.general().small_radar_measure_cnt())
    , ApproxSmallRadarMeasureCount(params.general().aprox_small_radar_measure_cnt())
{}

Target::Target(const BigRadarData& data, double deathTime, const Proto::Parameters& params)
    : Target(data.Id, deathTime, params)
{
    BigRadarUpdate(data.Pos, data.Speed);
}

void Target::BigRadarUpdate(Vector3d pos, Vector3d speed) {
    Timer.Restart();
    if (FilteredPos == pos) {
        return;
    }

    FilteredPos = pos;
    FilteredSpeed = speed;
    ++CurrBigRadarMeasureCount;

    if (CurrBigRadarMeasureCount >= BigRadarMeasureCount) {
        SetNeedToUpdateEntryPoint(true);
        SetNeedToUpdateMeetingPoint(true);
    }
}

void Target::SmallRadarUpdate(Vector3d pos) {
    if (Pos == pos) {
        return;
    }

    int dt = Timer.GetElapsedTimeAsMs();
    Timer.Restart();

    Pos = pos;
    ABFilterIterate(dt);

    if (CurrSmallRadarMeasureCount >= ApproxSmallRadarMeasureCount) {
        SetNeedToUpdateMeetingPoint(true);
    } else {
        ApproximateMeetingPoint = Vector3d::Zero();
    }
    EntryPoint = Vector3d::Zero();
}

void Target::ABFilterIterate(double dt) {
    auto prevPos = FilteredPos;
    auto prevSpeed = FilteredSpeed;

    auto filtered = ABFilter(Pos, prevPos, prevSpeed, dt, CurrSmallRadarMeasureCount);
    FilteredPos = filtered.first;
    FilteredSpeed = filtered.second;

    ++CurrSmallRadarMeasureCount;
}

bool Target::IsInSector(double rad, double angView, double angPos) const {
    double startAng = angPos - angView / 2;
    double endAng = angPos + angView / 2;
    auto polarPos = CartesianToCylindrical(FilteredPos);
    return polarPos.X <= rad && startAng <= polarPos.Y && polarPos.Y <= endAng;
}


std::string Target::DebugString() const {
    return std::string("TargetInfo:")
        + "\n  ID: " + std::to_string(Id)
        + "\n  Priority: " + std::to_string(Priority)
        + "\n  Pos: " + Pos.DebugString()
        + "\n  IsFollowed: " + std::to_string(IsFollowedFlag)
        + "\n  IsRocketLaunchedFlag: " + std::to_string(IsFollowedFlag)
        + "\n  CanBeFollowed: " + std::to_string(CanBeFollowed())
        + "\n  CurrBigRadarMeasureCount: " + std::to_string(CurrBigRadarMeasureCount)
        + "\n  CurrSmallRadarMeasureCount: " + std::to_string(CurrSmallRadarMeasureCount)
    ;
}


RadarController::RadarController(const Proto::Parameters& params)
    : Params(params)
    , RadarAnglePos(M_PI_2)
{}

void RadarController::Process(
    const std::vector<BigRadarData>& bigDatas,
    const std::vector<SmallRadarData>& smallDatas
) {
    // update from radars
    std::set<int> updatedTargets;
    for (const auto& data : smallDatas) {
        for (auto& target : Targets) {
            if (target.GetId() != data.Id) continue;

            target.SmallRadarUpdate(data.Pos);
            updatedTargets.insert(data.Id);
            break;
        }
    }
    for (const auto& data : bigDatas) {
        for (auto& target : Targets) {
            if (target.GetId() != data.Id || updatedTargets.count(data.Id)) continue;

            target.BigRadarUpdate(data.Pos, data.Speed);
            updatedTargets.insert(data.Id);
            break;
        }
        if (!updatedTargets.count(data.Id)) {
            Targets.emplace_back(data, Params.general().death_time(), Params);
        }
    }

    // calculate entry and meeting points
    static const double timeToCalculatePrecizeSpeed =
        Params.general().small_radar_measure_cnt() / Params.small_radar().frequency() * 1000;

    for (auto& target : Targets) {
        if (target.GetPriority() == -1 && (target.NeedToUpdateEntryPoint() || target.NeedToUpdateMeetingPoint())) {
            target.SetPriority(
                CalculatePriority(
                    target.GetFilteredPosition(),
                    target.GetFilteredSpeed(),
                    Params.simulator().max_target_speed()
                )
            );
        }
        if (target.NeedToUpdateEntryPoint()) {
            target.SetEntryPoint(
                CalculateEntryPoint(
                    target.GetFilteredPosition(),
                    target.GetFilteredSpeed(),
                    Vector3d::Zero(),
                    Params.small_radar().radius()
                )
            );
            target.SetNeedToUpdateEntryPoint(false);
        }
        if (target.NeedToUpdateMeetingPoint() && !target.IsRocketLaunched()) {
            Vector3d meetingPoint;
            if (IsTargetInSector(target)) {
                meetingPoint = CalculateMeetingPoint(
                    target.GetFilteredPosition() + target.GetFilteredSpeed() * Params.defense().time_to_launch_rocket(),
                    target.GetFilteredSpeed(),
                    Vector3d::Zero(),
                    Params.defense().rocket_speed()
                );
            } else {
                meetingPoint = CalculateMeetingPoint(
                    target.GetEntryPoint() + target.GetFilteredSpeed()
                        * (timeToCalculatePrecizeSpeed + Params.defense().time_to_launch_rocket()),
                    target.GetFilteredSpeed(),
                    Vector3d::Zero(),
                    Params.defense().rocket_speed()
                );
            }
            target.SetApproximateMeetingPoint(meetingPoint);
            target.SetNeedToUpdateMeetingPoint(false);
        }
    }
    RemoveDeadTargets();

    // follow target
    if (FollowedTargetIds.empty()) {
        TrySelectTargetToFollow();
    }
    if (!FollowedTargetIds.empty()) {
        int id = FollowedTargetIds.front();
        auto& target = GetTargetById(id);
        auto targetPos = target.GetFilteredPosition();

        RadarAngleTarget = CalculateRadarAngleOneTarget(
            RadarAnglePos,
            RadarAngleTarget,
            (target.GetEntryPoint() == Vector3d::Zero() ? -1 : GetPhi(target.GetEntryPoint())),
            (target.GetApproximateMeetingPoint() == Vector3d::Zero() ? -1 : GetPhi(target.GetApproximateMeetingPoint())),
            Params.small_radar().view_angle(),
            Params.general().margin_angle()
        );

        if (!target.IsRocketLaunched() && target.HavePreciseSpeed()) {
            auto meetingPoint = CalculateMeetingPoint(
                targetPos + target.GetFilteredSpeed() * Params.defense().time_to_launch_rocket(),
                target.GetFilteredSpeed(),
                Vector3d::Zero(),
                Params.defense().rocket_speed()
            );
            MeetingPointsAndTargetIds.emplace_back(meetingPoint, id);
            target.SetIsRocketLaunched(true);
        }
    }
}

void RadarController::TrySelectTargetToFollow() {
    if (Targets.empty()) {
        return;
    }
    int targetId = -1;
    double maxPriority = -10;
    for (const auto& target : Targets) {
        if (target.CanBeFollowed() && target.GetPriority() > maxPriority) {
            maxPriority = target.GetPriority();
            targetId = target.GetId();
        }
    }
    if (targetId != -1) {
        FollowedTargetIds.push_back(targetId);
    }
}

void RadarController::RemoveDeadTargets() {
    for (int i=0; i<Targets.size(); ++i) {
        if (Targets[i].IsDead()) {
            if (!FollowedTargetIds.empty() && FollowedTargetIds.front() == Targets[i].GetId()) {
                FollowedTargetIds.clear();
            }
            Targets.erase(Targets.begin() + i);
            --i;
        }
    }
}


Target& RadarController::GetTargetById(int id) {
    if (Targets.size() > id && Targets[id].GetId() == id) {
        return Targets[id];
    }
    for (auto& target : Targets) {
        if (target.GetId() == id) {
            return target;
        }
    }
    throw std::out_of_range("Target with id " + std::to_string(id) + " not found\n");
}


RadarController::Result RadarController::GetAngleAndMeetingPoints() {
    double ms = Timer.GetElapsedTimeAsMs();
    Timer.Restart();

    if (RadarAngleTarget != -1) {
        double maxDelta = Params.small_radar().angle_speed() * ms / 1000;

        if (std::abs(RadarAnglePos - RadarAngleTarget) <= maxDelta) {
            RadarAnglePos = RadarAngleTarget;
        } else {
            if (RadarAnglePos < RadarAngleTarget) {
                RadarAnglePos += maxDelta;
            } else {
                RadarAnglePos -= maxDelta;
            }
        }
    }
    const auto minAngle = Params.small_radar().view_angle() / 2;
    const auto maxAngle = M_PI - Params.small_radar().view_angle() / 2;
    RadarAnglePos = std::max(minAngle, std::min(maxAngle, RadarAnglePos));

    auto res = RadarController::Result{
        .Angle = RadarAnglePos,
        .FollowedTargetIds = FollowedTargetIds,
        .MeetingPointsAndTargetIds = MeetingPointsAndTargetIds
    };
    MeetingPointsAndTargetIds.clear();
    return res;
}

std::vector<Vector3d> RadarController::GetEntryPoints() const {
    std::vector<Vector3d> res;
    for (const auto& target : Targets) {
        auto p = target.GetEntryPoint();
        if (p != Vector3d::Zero()) {
            res.push_back(p);
        }
    }
    return res;
}

std::vector<Vector3d> RadarController::GetApproximateMeetingPoints() const {
    std::vector<Vector3d> res;
    for (const auto& target : Targets) {
        auto p = target.GetApproximateMeetingPoint();
        if (p != Vector3d::Zero()) {
            res.push_back(p);
        }
    }
    return res;
}

std::map<int, double> RadarController::GetPriorities() const {
    std::map<int, double>  res;
    for (const auto& target : Targets) {
        res[target.GetId()] = target.GetPriority();
    }
    return res;
}

bool RadarController::IsTargetInSector(const RC::Target& target) const {
    double startAng = RadarAnglePos - Params.small_radar().view_angle() / 2;
    double endAng = RadarAnglePos + Params.small_radar().view_angle() / 2;
    auto polarPos = CartesianToCylindrical(target.GetFilteredPosition());
    return polarPos.X <= Params.small_radar().radius() && startAng <= polarPos.Y && polarPos.Y <= endAng;
}
