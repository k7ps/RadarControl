#include "radar_controller.h"
#include "proto/generated/params.pb.h"
#include "util/points.h"
#include "util/util.h"
#include "lib/calculations.h"

#include <cmath>
#include <iostream>
#include <stdexcept>

using namespace RC;


Target::Target(int id, double priority, double deathTime, const Proto::Parameters& params)
    : Id(id)
    , Priority(priority)
    , DeathTime(deathTime)
    , BigRadarMeasureCount(params.general().big_radar_measure_cnt())
    , SmallRadarMeasureCount(params.general().small_radar_measure_cnt())
{}

Target::Target(const BigRadarData& data, double deatTime, const Proto::Parameters& params)
    : Target(data.Id, data.Priority, deatTime, params)
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
        NeedToUpdateEntryPointFlag = true;
        NeedToUpdateMeetingPointFlag = true;
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

    NeedToUpdateMeetingPointFlag = true;
}

void Target::ABFilterIterate(double dt) {
    auto prevPos = FilteredPos;
    auto prevSpeed = FilteredSpeed;

    auto filtered = ABFilter(Pos, prevPos, prevSpeed, dt, CurrSmallRadarMeasureCount);
    FilteredPos = filtered.first;
    FilteredSpeed = filtered.second;

    ++CurrSmallRadarMeasureCount;
}

int Target::GetId() const {
    return Id;
}

double Target::GetPriority() const {
    return Priority;
}

Vector3d Target::GetPosition() const {
    return Pos;
}

Vector3d Target::GetFilteredPosition() const {
    return FilteredPos;
}

Vector3d Target::GetFilteredSpeed() const {
    return FilteredSpeed;
}

bool Target::HavePreciseSpeed() const {
    return CurrSmallRadarMeasureCount >= SmallRadarMeasureCount;
}

bool Target::IsDead() const {
    return (double) Timer.GetElapsedTimeAsMs() >= DeathTime;
}

void Target::SetFollowed(bool f) {
    IsFollowedFlag = f;
}

bool Target::IsFollowed() const {
    return IsFollowedFlag;
}

void Target::SetIsRocketLaunched(bool f) {
    IsRocketLaunchedFlag = f;
}

bool Target::IsRocketLaunched() const {
    return IsRocketLaunchedFlag;
}

void Target::SetEntryPoint(Vector3d p) {
    EntryPoint = p;
}

Vector3d Target::GetEntryPoint() const {
    return EntryPoint;
}

void Target::SetApproximateMeetingPoint(Vector3d p) {
    ApproximateMeetingPoint = p;
}

Vector3d Target::GetApproximateMeetingPoint() const {
    return ApproximateMeetingPoint;
}

bool Target::NeedToUpdateEntryPoint() const {
    return NeedToUpdateEntryPointFlag;
}

void Target::SetNeedToUpdateEntryPoint(bool f) {
    NeedToUpdateEntryPointFlag = f;
}

bool Target::NeedToUpdateMeetingPoint() const {
    return NeedToUpdateMeetingPointFlag;
}

void Target::SetNeedToUpdateMeetingPoint(bool f) {
    NeedToUpdateMeetingPointFlag = f;
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
        if (target.NeedToUpdateMeetingPoint()) {
            target.SetApproximateMeetingPoint(
                CalculateMeetingPoint(
                    target.GetEntryPoint() + target.GetFilteredSpeed()
                        * (timeToCalculatePrecizeSpeed + Params.defense().time_to_launch_rocket()),
                    target.GetFilteredSpeed(),
                    Vector3d::Zero(),
                    Params.defense().rocket_speed()
                )
            );
            target.SetNeedToUpdateMeetingPoint(false);
        }
    }
    RemoveDeadTargets();

    // follow target
    if (FollowedTargetIds.empty()) {
        SelectTargetToFollow();
    }
    if (!FollowedTargetIds.empty()) {
        int id = FollowedTargetIds.front();
        auto& target = GetTargetById(id);
        auto targetPos = target.GetFilteredPosition();
        auto cylindricalPos = CartesianToCylindrical(targetPos);

        RadarAngleTarget = cylindricalPos.Y;

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

void RadarController::SelectTargetToFollow() {
    if (Targets.empty()) {
        return;
    }
    int targetId = -1;
    double maxPriority = -1;
    for (const auto& target : Targets) {
        if (target.GetPriority() > maxPriority) {
            maxPriority = target.GetPriority();
            targetId = target.GetId();
        }
    }
    FollowedTargetIds.push_back(targetId);
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
