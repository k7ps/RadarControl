#include "radar_controller.h"
#include "proto/generated/params.pb.h"
#include "util/points.h"
#include "util/util.h"
#include "calculations.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

using namespace RC;


namespace {

    bool CanAddToAngleList(
        double maxDiff,
        const std::vector<double>& angles,
        const std::vector<double>& newAngles
    ) {
        if (angles.empty()) return true;
        auto min = *std::min_element(angles.begin(), angles.end());
        auto max = *std::max_element(angles.begin(), angles.end());
        for (auto newAngle : newAngles) {
            if (newAngle != -1 && (std::abs(newAngle - min) > maxDiff || std::abs(newAngle - max) > maxDiff)) {
                return false;
            }
        }
        return true;
    }

    const Target* GetTargetById(const std::vector<const Target*>& targets, int id){
        for (const auto* target : targets) {
            if (target->GetId() == id) {
                return target;
            }
        }
        throw std::out_of_range("Target with id " + std::to_string(id) + " not found\n");
    }

    double TimeToRotate(
        double currAngle,
        const std::vector<double>& angles,
        Vector3d pos,
        double angleSpeed
    ) {
        std::vector<double> diffs;
        for (auto angle : angles)
            if (angle != -1)
                diffs.push_back(std::abs(angle - currAngle));
        if (diffs.empty())
            diffs.push_back(std::abs(GetPhi(pos) - currAngle));
        return *std::max_element(diffs.begin(), diffs.end()) / angleSpeed;
    }

    std::pair<double, std::vector<int>> CalculateRadarAngle(
        double currRadarAngle,
        double currRadarTargetAngle,
        const std::vector<int>& followedTargetIds,
        const std::vector<const Target*>& targets, // sorted by priorities
        double radarAngleSpeed,
        double viewAngle,
        double margin
    ) {
        if (targets.empty()) {
            return {currRadarAngle, {}};
        }
        double halfview = viewAngle / 2;
        double willAngL = currRadarTargetAngle - halfview + margin;
        double willAngR = currRadarTargetAngle + halfview - margin;

        std::vector<int> newFollowedTargetIds;
        std::vector<double> followedAngles;

        for (const auto* target : targets) {
            auto entryAngle = target->GetEntryAngle();
            auto meetAngle = target->GetMeetAngle();
            if (CanAddToAngleList(viewAngle - 2 * margin, followedAngles, {entryAngle, meetAngle})) {
                if (
                    !IsInSegment({entryAngle, meetAngle}, willAngL, willAngR)
                    && !IsInVector(followedTargetIds, target->GetId())
                ) {
                    auto time2rotate = TimeToRotate(
                        currRadarTargetAngle,
                        {entryAngle, meetAngle},
                        target->GetFilteredPosition(),
                        radarAngleSpeed
                    );
                    auto time2entry = target->GetTimeToEntryPoint();

                    std::vector<int> targets2hit;
                    for (auto id : followedTargetIds) {
                        const auto* target = GetTargetById(targets, id);
                        auto time2meet = target->GetTimeToMeetingPoint();
                        if (
                            target->IsRocketLaunched()
                            || target->CanLaunchRocket()
                            || time2meet + time2rotate < time2entry
                        ) {
                            targets2hit.push_back(id);
                        }
                    }
                    if (!targets2hit.empty()) {
                        return {currRadarTargetAngle, targets2hit};
                    }
                }

                newFollowedTargetIds.push_back(target->GetId());
                if (entryAngle != -1) followedAngles.push_back(entryAngle);
                if (meetAngle != -1) followedAngles.push_back(meetAngle);
            }
        }
        if (followedAngles.empty()) {
            followedAngles.push_back(GetPhi(targets.front()->GetFilteredPosition()));
        }
        auto newTargetRadarAngle = CalculateRadarAngleMultiTarget(
            currRadarAngle,
            currRadarTargetAngle,
            followedAngles,
            viewAngle,
            margin
        );
        return {newTargetRadarAngle, newFollowedTargetIds};
    }

}


Target::Target(int id, double presetPriority, double deathTime, const Proto::Parameters& params)
    : Id(id)
    , PresetPriority(presetPriority)
    , DeathTime(deathTime)
    , BigRadarMeasureCount(params.general().big_radar_measure_cnt())
    , SmallRadarMeasureCount(params.general().small_radar_measure_cnt())
    , ApproxSmallRadarMeasureCount(params.general().aprox_small_radar_measure_cnt())
{}

Target::Target(const BigRadarData& data, double deathTime, const Proto::Parameters& params)
    : Target(data.Id, data.PresetPriority, deathTime, params)
{
    BigRadarUpdate(data.Pos, data.Speed);
}

void Target::BigRadarUpdate(Vector3d pos, Vector3d speed) {
    Timer.Restart();
    if (FilteredPos == pos) {
        return;
    }

    FilteredPos = pos;
    SpeedFromBigRadar = speed;
    ++CurrBigRadarMeasureCount;

    if (CurrBigRadarMeasureCount >= BigRadarMeasureCount) {
        SetNeedToUpdateEntryPoint(true);
        SetNeedToUpdateMeetingPoint(true);
    }
    CurrSmallRadarMeasureCount = 0;
}

void Target::SmallRadarUpdate(Vector3d pos) {
    if (Pos == pos) {
        return;
    }

    int dt = Timer.GetElapsedTimeAsMs();
    Timer.Restart();

    Pos = pos;

    auto filtered = ABFilter(Pos, FilteredPos, FilteredSpeed, dt, CurrSmallRadarMeasureCount);
    FilteredPos = filtered.first;
    FilteredSpeed = filtered.second;

    if (CurrSmallRadarMeasureCount >= ApproxSmallRadarMeasureCount) {
        SetNeedToUpdateMeetingPoint(true);
    }
    EntryPoint = Vector3d::Zero();
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
        for (auto* target : Targets) {
            if (target->GetId() != data.Id) continue;

            target->SmallRadarUpdate(data.Pos);
            updatedTargets.insert(data.Id);
            break;
        }
    }
    for (const auto& data : bigDatas) {
        for (auto* target : Targets) {
            if (target->GetId() != data.Id || updatedTargets.count(data.Id)) continue;

            target->BigRadarUpdate(data.Pos, data.Speed);
            updatedTargets.insert(data.Id);
            break;
        }
        if (!updatedTargets.count(data.Id)) {
            Targets.push_back(new Target(data, Params.general().death_time(), Params));
        }
    }

    // calculate entry and meeting points
    static const double time2CalculatePrecizeSpeed =
        Params.general().small_radar_measure_cnt() / Params.small_radar().frequency() * 1000;

    for (auto* target : Targets) {
        if (target->GetPriority() == -1 && (target->NeedToUpdateEntryPoint() || target->NeedToUpdateMeetingPoint())) {
            if (target->GetPresetPriority() != -1) {
                target->SetPriority(target->GetPresetPriority());
            } else {
                target->SetPriority(
                    CalculatePriority(
                        target->GetFilteredPosition(),
                        target->GetFilteredSpeed(),
                        Params.simulator().max_target_speed()
                    )
                );
            }
        }
        if (target->NeedToUpdateEntryPoint()) {
            target->SetEntryPoint(
                CalculateEntryPoint(
                    target->GetFilteredPosition(),
                    target->GetFilteredSpeed(),
                    Params.small_radar().radius()
                )
            );
            target->SetNeedToUpdateEntryPoint(false);
        }
        if (target->NeedToUpdateMeetingPoint() && !target->IsRocketLaunched()) {
            Vector3d fromPoint;
            double time2hit = Params.defense().time_to_launch_rocket();
            if (IsTargetInSector(target)) {
                fromPoint = target->GetFilteredPosition();
                time2hit += target->GetMeasureCountToPreciseSpeed() / Params.small_radar().frequency() * 1000;
            } else if (Distance(target->GetFilteredPosition(), Vector3d::Zero()) <= Params.small_radar().radius()) {
                fromPoint = target->GetFilteredPosition();
                time2hit += time2CalculatePrecizeSpeed + TimeToRotate(
                    RadarAnglePos,
                    {},
                    target->GetFilteredPosition(),
                    Params.small_radar().angle_speed()
                );
            } else {
                fromPoint = target->GetEntryPoint();
                time2hit += time2CalculatePrecizeSpeed;
            }
            target->SetApproximateMeetingPoint(CalculateMeetingPoint(
                fromPoint + target->GetFilteredSpeed() * time2hit,
                target->GetFilteredSpeed(),
                Params.defense().rocket_speed()
            ));
            target->SetNeedToUpdateMeetingPoint(false);
        }
    }
    RemoveDeadTargets();

    // follow target
    std::vector<const Target*> targetsWithPriorities;
    for (const auto* target : Targets) {
        if (target->GetPriority() != -1) {
            targetsWithPriorities.push_back(target);
        }
    }
    std::sort(
        targetsWithPriorities.begin(),
        targetsWithPriorities.end(),
        [](const Target* l, const Target* r) {
            return l->GetPriority() > r->GetPriority();
        }
    );

    auto res = CalculateRadarAngle(
        RadarAnglePos,
        RadarAngleTarget,
        FollowedTargetIds,
        targetsWithPriorities,
        Params.small_radar().angle_speed(),
        Params.small_radar().view_angle(),
        Params.general().margin_angle()
    );

    RadarAngleTarget = res.first;
    FollowedTargetIds = res.second;

    for (auto* target : Targets) {
        auto id = target->GetId();
        if (IsInVector(FollowedTargetIds, id) && target->CanLaunchRocket() && !target->IsRocketLaunched()) {
            auto meetingPoint = CalculateMeetingPoint(
                target->GetFilteredPosition() + target->GetFilteredSpeed() * Params.defense().time_to_launch_rocket(),
                target->GetFilteredSpeed(),
                Params.defense().rocket_speed()
            );
            MeetingPointsAndTargetIds.emplace_back(meetingPoint, id);
            target->SetApproximateMeetingPoint(meetingPoint);
            target->SetIsRocketLaunched(true);
        }
    }
}

void RadarController::TrySelectTargetToFollow() {
    if (Targets.empty()) {
        return;
    }
    int targetId = -1;
    double maxPriority = -10;
    for (const auto* target : Targets) {
        if (target->CanBeFollowed() && target->GetPriority() > maxPriority) {
            maxPriority = target->GetPriority();
            targetId = target->GetId();
        }
    }
    if (targetId != -1) {
        FollowedTargetIds.push_back(targetId);
    }
}

void RadarController::RemoveDeadTargets() {
    for (int i = 0; i < Targets.size(); ++i) {
        if (Targets[i]->IsDead()) {
            if (!FollowedTargetIds.empty() && IsInVector(FollowedTargetIds, Targets[i]->GetId())) {
                for (int j = 0; j < FollowedTargetIds.size(); ++j) {
                    if (FollowedTargetIds[j] == Targets[i]->GetId()) {
                        FollowedTargetIds.erase(FollowedTargetIds.begin() + j);
                        break;
                    }
                }
            }
            delete Targets[i];
            Targets.erase(Targets.begin() + i);
            --i;
        }
    }
}


RadarController::Result RadarController::GetAngleAndMeetingPoints() {
    double ms = Timer.GetElapsedTimeAsMs();
    Timer.Restart();

    if (RadarAngleTarget != -1) {
        double maxDelta = Params.small_radar().angle_speed() * ms;

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
    for (const auto* target : Targets) {
        auto p = target->GetEntryPoint();
        if (p != Vector3d::Zero()) {
            res.push_back(p);
        }
    }
    return res;
}

std::vector<Vector3d> RadarController::GetApproximateMeetingPoints() const {
    std::vector<Vector3d> res;
    for (const auto* target : Targets) {
        auto p = target->GetApproximateMeetingPoint();
        if (p != Vector3d::Zero()) {
            res.push_back(p);
        }
    }
    return res;
}

std::map<int, double> RadarController::GetPriorities() const {
    std::map<int, double>  res;
    for (const auto* target : Targets) {
        res[target->GetId()] = target->GetPriority();
    }
    return res;
}

bool RadarController::IsTargetInSector(const RC::Target* target) const {
    double startAng = RadarAnglePos - Params.small_radar().view_angle() / 2;
    double endAng = RadarAnglePos + Params.small_radar().view_angle() / 2;
    auto polarPos = CartesianToCylindrical(target->GetFilteredPosition());
    return polarPos.X <= Params.small_radar().radius() && startAng <= polarPos.Y && polarPos.Y <= endAng;
}

RadarController::~RadarController() {
    for (auto* target : Targets) {
        delete target;
    }
}
