#include "radar_controller.h"
#include "calculations.h"
#include "proto/generated/params.pb.h"
#include "radar_control/data.h"
#include "util/points.h"
#include "util/util.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>

using namespace RC;


namespace {

    bool CanAddToAngleArray(
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

    void SortTargetsByPriorities(std::vector<const Target*>& vec) {
        std::sort(
            vec.begin(),
            vec.end(),
            [](const Target* l, const Target* r) {
                return l->GetPriority() > r->GetPriority();
            }
        );
    }

    double TimeToRotate(RadarPos curr, RadarPos target, double maxEps) {
        auto time = 0.;
        auto distToTarget = target.Angle - curr.Angle;
        auto direction = (distToTarget > 0 ? 1. : -1.);
        if (std::abs(distToTarget) < 1e-3 && std::abs(curr.Speed) < maxEps * 400) return time;

        auto stoppingDist = 0.5 * curr.Speed * curr.Speed / maxEps;
        if (curr.Speed * direction < 0 || std::abs(distToTarget) < stoppingDist) {
            time += std::abs(curr.Speed) / maxEps;
            curr.Angle += (curr.Speed > 0 ? 1. : -1.) * curr.Speed * curr.Speed / (2 * maxEps);
            curr.Speed = 0;
            distToTarget = target.Angle - curr.Angle;
        }
        distToTarget = std::abs(distToTarget);

        auto accDist = 0.;
        auto decDist = 0.;
        auto currAbsSpeed = std::abs(curr.Speed);

        if (currAbsSpeed < target.Speed) {
            auto timeToMaxSpeed = (target.Speed - currAbsSpeed) / maxEps;
            accDist = currAbsSpeed * timeToMaxSpeed + 0.5 * maxEps * timeToMaxSpeed * timeToMaxSpeed;
        }
        decDist = 0.5 * target.Speed * target.Speed / maxEps;

        if (accDist + decDist < distToTarget) {
            time += (2 * target.Speed - currAbsSpeed) / maxEps + (distToTarget - accDist - decDist) / target.Speed;
        } else {
            auto times = SolveQuadraticEquation(
                maxEps,
                2 * currAbsSpeed,
                0.5 * currAbsSpeed * currAbsSpeed / maxEps - distToTarget
            );
            if (!times.empty()) {
                time += *std::max_element(times.begin(), times.end()) * 2;
            }
            time += currAbsSpeed / maxEps;
        }
        return time;
    }

    double TimeToRotateToTarget(
        RadarPos pos,
        const std::vector<double>& targetAngles,
        double maxAngleSpeed,
        double maxEps
    ) {
        auto timeTo = [&pos, &maxAngleSpeed, &maxEps](double to) {
            return TimeToRotate(pos, RadarPos{.Angle=to, .Speed=maxAngleSpeed}, maxEps);
        };
        if (targetAngles.size() == 1)
            return timeTo(targetAngles[0]);
        return std::max(
            timeTo(*std::min_element(targetAngles.begin(), targetAngles.end())),
            timeTo(*std::max_element(targetAngles.begin(), targetAngles.end()))
        );
    }

    std::vector<double> GetTargetAngles(const Target* target) {
        std::vector<double> res;
        if (target->GetMeetAngle() != -1) res.push_back(target->GetMeetAngle());
        if (target->GetEntryAngle() != -1) res.push_back(target->GetEntryAngle());
        else res.push_back(CalculateAngle(target->GetFilteredPosition()));
        return res;
    }

    RadarPos UpdateRadarPos(RadarPos curr, RadarPos target, double maxEps, double deltaTime) {
        auto distToTarget = target.Angle - curr.Angle;
        auto direction = (distToTarget > 0 ? 1. : -1.);
        auto stoppingDist = 0.5 * curr.Speed * curr.Speed / maxEps;

        double eps = 0.;
        if (std::abs(distToTarget) <= stoppingDist || direction * curr.Speed < 0) {
            eps = (curr.Speed > 0 ? -1. : 1.) * maxEps;
        } else {
            if (std::abs(curr.Speed) < target.Speed) {
                eps = (curr.Speed > 0 ? 1. : -1.) * maxEps;
            } else {
                eps = 0;
            }
        }
        double newSpeed = curr.Speed + eps * deltaTime;
        if (std::abs(newSpeed) > target.Speed) {
            newSpeed = (newSpeed > 0 ? 1. : -1.) * target.Speed;
        }

        double newAngle = curr.Angle + newSpeed * deltaTime;
        if (std::abs(newAngle - target.Angle) < 1e-3 && std::abs(newSpeed) < maxEps * 400) {
            newAngle = target.Angle;
            newSpeed = 0;
        }
        return RadarPos{
            .Angle = newAngle,
            .Speed = newSpeed
        };
    }

    std::pair<RadarPos, std::vector<int>> CalculateRadarPos(
        RadarPos currPos,
        RadarPos currTargetPos,
        const std::vector<int>& currFollowedTargetIds,
        const std::vector<const Target*>& targetsInsideResponsible, // sorted by priorities
        const std::vector<const Target*>& targetsOutsideResponsible, // sorted by priorities
        const Proto::Parameters& params
    ) {
        if (targetsInsideResponsible.empty() && targetsOutsideResponsible.empty()) {
            return {currPos, {}};
        }

        const auto viewAngle = params.small_radar().view_angle();
        const auto maxSpeed  = params.small_radar().max_angle_speed();
        const auto maxEps    = params.small_radar().max_eps();
        const auto margin    = params.general().margin_angle();

        const auto halfview = viewAngle / 2;
        const auto willAngL = currPos.Angle - halfview + margin;
        const auto willAngR = currPos.Angle + halfview - margin;

        std::vector<int> followedTargetIds;
        std::vector<double> followedTargetAngles;

        for (const auto* target : targetsInsideResponsible) {
            auto targetAngles = GetTargetAngles(target);

            if (CanAddToAngleArray(viewAngle - 2 * margin, followedTargetAngles, targetAngles)) {
                if (
                    !IsInSegment(targetAngles, willAngL, willAngR)
                    && !IsInVector(currFollowedTargetIds, target->GetId())
                ) {
                    auto timeToRotate = TimeToRotateToTarget(currPos, targetAngles, maxSpeed, maxEps);
                    auto timeToNear = target->GetTimeToNearPoint();

                    std::vector<int> targetsToHitIds;
                    std::vector<double> targetsToHitAngles;
                    for (auto id : currFollowedTargetIds) {
                        const auto* followedTarget = GetTargetById(targetsInsideResponsible, id);
                        auto timeToMeet = followedTarget->GetTimeToMeetPoint();
                        if (
                            followedTarget->IsRocketLaunched()
                            || followedTarget->CanLaunchRocket()
                            // radar should kill target and rotate to priority one before priority gets in near zone
                            || timeToMeet + timeToRotate < timeToNear
                        ) {
                            targetsToHitIds.push_back(id);
                            JoinToVector(targetsToHitAngles, GetTargetAngles(followedTarget));
                        }
                    }
                    if (
                        !targetsToHitIds.empty()
                        && !CanAddToAngleArray(viewAngle - 2 * margin, targetsToHitAngles, targetAngles)
                    ) {
                        return {currTargetPos, targetsToHitIds};
                    }
                }

                followedTargetIds.push_back(target->GetId());
                JoinToVector(followedTargetAngles, targetAngles);
            }
        }
        for (const auto* target : targetsOutsideResponsible) {
            auto targetAngles = GetTargetAngles(target);

            if (CanAddToAngleArray(viewAngle - 2 * margin, followedTargetAngles, targetAngles)) {
                followedTargetIds.push_back(target->GetId());
                JoinToVector(followedTargetAngles, targetAngles);
            }
        }
        auto newTargetRadarAngle = CalculateRadarAngleMultiTarget(
            currPos.Angle,
            currTargetPos.Angle,
            followedTargetAngles,
            viewAngle,
            margin
        );
        return {RadarPos{.Angle=newTargetRadarAngle, .Speed=maxSpeed}, followedTargetIds};
    }

}


Target::Target(int id, double presetPriority, double deathTime, const Proto::Parameters& params)
    : Id(id)
    , PresetPriority(presetPriority)
    , SmallRadarRadius(params.small_radar().radius())
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
        if (CanBeInRadarSector()) {
            EntryPoint = Vector3d::Zero();
        } else {
            SetNeedToUpdateEntryPoint(true);
        }
        SetNeedToUpdateNearPoint(true);
        SetNeedToUpdateMeetPoint(true);
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
        SetNeedToUpdateNearPoint(true);
        SetNeedToUpdateMeetPoint(true);
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


RadarController::RadarController(const Proto::Parameters& params, double startAngle)
    : Params(params)
    , Pos{.Angle = startAngle, .Speed = 0}
    , TargetPos{.Angle = -1, .Speed = 0}
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

    // calculate entry and meet points
    static const double time2CalculatePrecizeSpeed =
        Params.general().small_radar_measure_cnt() / Params.small_radar().frequency() * 1000;

    for (auto* target : Targets) {
        if (target->GetPriority() == -1 && (target->NeedToUpdateEntryPoint() || target->NeedToUpdateMeetPoint())) {
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
        if (target->NeedToUpdateNearPoint()) {
            target->SetNearPoint(
                CalculateEntryPoint(
                    target->GetFilteredPosition(),
                    target->GetFilteredSpeed(),
                    Params.small_radar().radius() * 0.5
                )
            );
            target->SetNeedToUpdateNearPoint(false);
        }
        if (target->NeedToUpdateMeetPoint() && !target->IsRocketLaunched()) {
            Vector3d fromPoint;
            double time2hit = Params.defense().time_to_launch_rocket();
            if (IsTargetInRadarSector(target)) {
                fromPoint = target->GetFilteredPosition();
                time2hit += target->GetMeasureCountToPreciseSpeed() / Params.small_radar().frequency() * 1000;
            } else if (target->CanBeInRadarSector()) {
                fromPoint = target->GetFilteredPosition();
                time2hit += time2CalculatePrecizeSpeed + TimeToRotateToTarget(
                    Pos,
                    {CalculateAngle(target->GetFilteredPosition())},
                    Params.small_radar().max_angle_speed(),
                    Params.small_radar().max_eps()
                );
            } else {
                fromPoint = target->GetEntryPoint();
                time2hit += time2CalculatePrecizeSpeed;
            }
            target->SetApproximateMeetPoint(CalculateMeetPoint(
                fromPoint + target->GetFilteredSpeed() * time2hit,
                target->GetFilteredSpeed(),
                Params.defense().rocket_speed()
            ));
            target->SetNeedToUpdateMeetPoint(false);
        }
    }
    RemoveDeadTargets();

    // follow target
    std::vector<const Target*> targetsInsideResponsible;
    std::vector<const Target*> targetsOutsideResponsible;
    for (const auto* target : Targets) {
        if (target->GetPriority() != -1) {
            if (IsTargetInResponsibleSector(target)) {
                targetsInsideResponsible.push_back(target);
            } else {
                targetsOutsideResponsible.push_back(target);
            }
        }
    }
    SortTargetsByPriorities(targetsInsideResponsible);
    SortTargetsByPriorities(targetsOutsideResponsible);

    auto res = CalculateRadarPos(
        Pos,
        TargetPos,
        FollowedTargetIds,
        targetsInsideResponsible,
        targetsOutsideResponsible,
        Params
    );

    TargetPos = res.first;
    FollowedTargetIds = res.second;

    for (auto* target : Targets) {
        auto id = target->GetId();
        if (IsInVector(FollowedTargetIds, id) && target->CanLaunchRocket() && !target->IsRocketLaunched()) {
            auto meetPoint = CalculateMeetPoint(
                target->GetFilteredPosition() + target->GetFilteredSpeed() * Params.defense().time_to_launch_rocket(),
                target->GetFilteredSpeed(),
                Params.defense().rocket_speed()
            );
            MeetPointsAndTargetIds.emplace_back(meetPoint, id);
            target->SetApproximateMeetPoint(meetPoint);
            target->SetIsRocketLaunched(true);
        }
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

RadarController::Result RadarController::GetAngleAndMeetPoints() {
    double ms = Timer.GetElapsedTimeAsMs();
    Timer.Restart();

    if (TargetPos.Angle != -1) {
        Pos = UpdateRadarPos(Pos, TargetPos, Params.small_radar().max_eps(), ms);
    }

    auto res = RadarController::Result{
        .Angle = Pos.Angle,
        .FollowedTargetIds = FollowedTargetIds,
        .MeetPointsAndTargetIds = MeetPointsAndTargetIds
    };
    MeetPointsAndTargetIds.clear();
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

std::vector<Vector3d> RadarController::GetApproximateMeetPoints() const {
    std::vector<Vector3d> res;
    for (const auto* target : Targets) {
        auto p = target->GetApproximateMeetPoint();
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

bool RadarController::IsTargetInRadarSector(const RC::Target* target) const {
    double startAng = Pos.Angle - Params.small_radar().view_angle() / 2;
    double endAng = Pos.Angle + Params.small_radar().view_angle() / 2;
    auto polarPos = CartesianToCylindrical(target->GetFilteredPosition());
    return polarPos.X <= Params.small_radar().radius() && startAng <= polarPos.Y && polarPos.Y <= endAng;
}

bool RadarController::IsTargetInResponsibleSector(const RC::Target* target) const {
    auto meetPoint = target->GetApproximateMeetPoint();
    if (meetPoint == Vector3d::Zero())
        return true;
    auto meetPointPolar = CartesianToCylindrical(meetPoint);
    return Params.small_radar().responsible_sector_start() <= meetPointPolar.Y
        && meetPointPolar.Y <= Params.small_radar().responsible_sector_end();
}

RadarController::~RadarController() {
    for (auto* target : Targets) {
        delete target;
    }
}
