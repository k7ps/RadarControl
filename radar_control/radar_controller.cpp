#include "radar_controller.h"
#include "calculations.h"
#include "proto/generated/params.pb.h"
#include "radar_control/data.h"
#include "util/points.h"
#include "util/proto.h"
#include "util/util.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>

using namespace RC;


namespace {

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

    std::vector<double> GetTargetAngles(const Target* target) {
        std::vector<double> res;
        if (target->GetMeetAngle() != -1) res.push_back(target->GetMeetAngle());
        if (target->GetEntryAngle() != -1) res.push_back(target->GetEntryAngle());
        else res.push_back(target->GetPosAngle());
        return res;
    }

    std::pair<RadarTargetPos, std::vector<int>> CalculateRadarPosBase(
        RadarPos currPos,
        RadarTargetPos currTargetPos,
        const std::vector<int>& currFollowedTargetIds,
        const std::vector<const Target*>& targetsInsideResponsible, // sorted by priorities
        const std::vector<const Target*>& targetsOutsideResponsible, // sorted by priorities
        const Proto::Parameters& params
    ) {
        if (targetsInsideResponsible.empty() && targetsOutsideResponsible.empty()) {
            return {currTargetPos, {}};
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
                    auto timeToRotate = TimeToRotateToTarget(
                        currPos, currTargetPos, targetAngles, maxSpeed, maxEps, viewAngle, margin
                    );
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
                        followedTargetIds = targetsToHitIds;
                        followedTargetAngles = targetsToHitAngles;
                        break;
                    }
                }

                followedTargetIds.push_back(target->GetId());
                JoinToVector(followedTargetAngles, targetAngles);
            }
        }
        if (followedTargetIds.empty()) {
            for (const auto* target : targetsOutsideResponsible) {
                auto targetAngles = GetTargetAngles(target);

                if (CanAddToAngleArray(viewAngle - 2 * margin, followedTargetAngles, targetAngles)) {
                    if (
                        !IsInSegment(targetAngles, willAngL, willAngR)
                        && !IsInVector(currFollowedTargetIds, target->GetId())
                    ) {
                        auto timeToRotate = TimeToRotateToTarget(
                            currPos, currTargetPos, targetAngles, maxSpeed, maxEps, viewAngle, margin
                        );
                        auto timeToNear = target->GetTimeToNearPoint();

                        std::vector<int> targetsToHitIds;
                        std::vector<double> targetsToHitAngles;
                        for (auto id : currFollowedTargetIds) {
                            const auto* followedTarget = GetTargetById(targetsOutsideResponsible, id);
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
                            followedTargetIds = targetsToHitIds;
                            followedTargetAngles = targetsToHitAngles;
                            break;
                        }
                    }

                    followedTargetIds.push_back(target->GetId());
                    JoinToVector(followedTargetAngles, targetAngles);
                }
            }
        } else {
            for (const auto* target : targetsOutsideResponsible) {
                auto targetAngles = GetTargetAngles(target);

                if (CanAddToAngleArray(viewAngle - 2 * margin, followedTargetAngles, targetAngles)) {
                    followedTargetIds.push_back(target->GetId());
                    JoinToVector(followedTargetAngles, targetAngles);
                }
            }
        }
        double nextTargetAngle = -1;
        for (const Target* target : targetsInsideResponsible) {
            if (!IsInVector(followedTargetIds, target->GetId())) {
                nextTargetAngle = target->GetPosAngle();
                break;
            }
        }
        if (nextTargetAngle == -1) {
            for (const Target* target : targetsOutsideResponsible) {
                if (!IsInVector(followedTargetIds, target->GetId())) {
                    nextTargetAngle = target->GetPosAngle();
                    break;
                }
            }
        }
        if (nextTargetAngle != -1) {
            auto minAngle = *std::min_element(followedTargetAngles.begin(), followedTargetAngles.end());
            auto maxAngle = *std::max_element(followedTargetAngles.begin(), followedTargetAngles.end());
            if (nextTargetAngle > maxAngle) {
                followedTargetAngles.push_back(minAngle + viewAngle - 2 * margin);
            } else if (nextTargetAngle < minAngle) {
                followedTargetAngles.push_back(maxAngle - viewAngle + 2 * margin);
            }
        }
        auto newTargetRadarAngle = CalculateRadarAngleMultiTarget(
            currPos.Angle,
            currTargetPos.Angle,
            followedTargetAngles,
            viewAngle,
            margin
        );
        return {RadarTargetPos{.Angle=newTargetRadarAngle, .Speed=maxSpeed}, followedTargetIds};
    }

    std::pair<std::pair<RadarTargetPos, RadarTargetPos>, std::vector<int>> CalculateRadarPosImproved(
        RadarPos currPos,
        RadarTargetPos currTargetPos,
        RadarPos shipCurrPos,
        RadarTargetPos shipCurrTargetPos,
        const std::vector<int>& currFollowedTargetIds,
        const std::vector<const Target*>& targetsInsideResponsible, // sorted by priorities
        const std::vector<const Target*>& targetsOutsideResponsible, // sorted by priorities
        const Proto::Parameters& params
    ) {
        if (targetsInsideResponsible.empty() && targetsOutsideResponsible.empty()) {
            return {{currTargetPos, shipCurrTargetPos}, {}};
        }

        const auto viewAngle = params.small_radar().view_angle();
        const auto maxSpeed  = params.small_radar().max_angle_speed();
        const auto maxEps    = params.small_radar().max_eps();
        const auto margin    = params.general().margin_angle();

        const auto halfview = viewAngle / 2;
        const auto angL     = currPos.Angle - halfview + margin;
        const auto angR     = currPos.Angle + halfview - margin;
        const auto willAngL = currTargetPos.Angle - halfview + margin;
        const auto willAngR = currTargetPos.Angle + halfview - margin;

        const auto deadZones = SegmentsFromProto(params.ship().dead_zones());

        std::vector<int> followedTargetIds;
        std::vector<double> followedTargetAngles;

        for (const auto* target : targetsInsideResponsible) {
            auto targetAngles = GetTargetAngles(target);

            if (CanAddToAngleArray(viewAngle - 2 * margin, followedTargetAngles, targetAngles)) {
                if (!IsInSegment(targetAngles, willAngL, willAngR)) {
                    auto timeToRotate = TimeToRotateToTarget(
                        currPos, currTargetPos, targetAngles, maxSpeed, maxEps, viewAngle, margin
                    );
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
                        followedTargetIds = targetsToHitIds;
                        followedTargetAngles = targetsToHitAngles;
                        break;
                    }
                }

                followedTargetIds.push_back(target->GetId());
                JoinToVector(followedTargetAngles, targetAngles);
            }
        }

        bool isOutsideTargetsChecked = false;

        if (followedTargetIds.empty()) {
            isOutsideTargetsChecked = true;
            for (const auto* target : targetsOutsideResponsible) {
                auto targetAngles = GetTargetAngles(target);

                if (CanAddToAngleArray(viewAngle - 2 * margin, followedTargetAngles, targetAngles)) {
                    if (!IsInSegment(targetAngles, willAngL, willAngR)) {
                        auto timeToRotate = TimeToRotateToTarget(
                            currPos, currTargetPos, targetAngles, maxSpeed, maxEps, viewAngle, margin
                        );
                        auto timeToNear = target->GetTimeToNearPoint();

                        std::vector<int> targetsToHitIds;
                        std::vector<double> targetsToHitAngles;
                        for (auto id : currFollowedTargetIds) {
                            const auto* followedTarget = GetTargetById(targetsOutsideResponsible, id);
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
                            followedTargetIds = targetsToHitIds;
                            followedTargetAngles = targetsToHitAngles;
                            break;
                        }
                    }

                    followedTargetIds.push_back(target->GetId());
                    JoinToVector(followedTargetAngles, targetAngles);
                }
            }
        }

        auto invertedDeadZones = InvertSegments(deadZones, M_PI, margin);

        auto newShipTargetAngle = CalculateShipAngleMultiTarget(
            shipCurrPos.Angle,
            shipCurrTargetPos.Angle,
            followedTargetAngles,
            invertedDeadZones
        );
        const auto willDeadZones = ShiftedSegmentsFromProto(params.ship().dead_zones(), newShipTargetAngle);

        if (!isOutsideTargetsChecked) {
            for (const auto* target : targetsOutsideResponsible) {
                auto targetAngles = GetTargetAngles(target);

                if (
                    CanAddToAngleArray(viewAngle - 2 * margin, followedTargetAngles, targetAngles)
                    && !IsInAnySegment(willDeadZones, targetAngles[0])
                ) {
                    followedTargetIds.push_back(target->GetId());
                    JoinToVector(followedTargetAngles, targetAngles);
                }
            }
        }

        double nextTargetAngle = -1;
        double timeToReach = -1;
        for (const Target* target : targetsInsideResponsible) {
            if (!IsInVector(followedTargetIds, target->GetId())) {
                nextTargetAngle = target->GetPosAngle();
                break;
            }
        }
        if (nextTargetAngle == -1) {
            for (const Target* target : targetsOutsideResponsible) {
                if (!IsInVector(followedTargetIds, target->GetId())) {
                    nextTargetAngle = target->GetPosAngle();
                    break;
                }
            }
        }
        auto newRadarTargetAngle = CalculateRadarAngleMultiTarget(
            currPos.Angle,
            currTargetPos.Angle,
            followedTargetAngles,
            viewAngle,
            margin
        );
        if (newRadarTargetAngle == -1) {
            if (currTargetPos.Angle == -1) {
                newRadarTargetAngle = currPos.Angle;
            } else {
                newRadarTargetAngle = currTargetPos.Angle;
            }
        }
        if (nextTargetAngle != -1 && IsInSegment(followedTargetAngles, angL, angR)) {
            const auto newWillAngL = newRadarTargetAngle - halfview + margin;
            const auto newWillAngR = newRadarTargetAngle + halfview - margin;
            const auto invertedWillDeadZones = InvertSegments(willDeadZones, M_PI, margin);

            double maxDownRadar = 1e9, maxUpRadar = 1e9;
            double maxDownShip = 1e9, maxUpShip = 1e9;
            for (const auto* target : targetsInsideResponsible) {
                if (IsInVector(followedTargetIds, target->GetId())) {
                    timeToReach = std::max(timeToReach, target->GetTimeToMeetPoint());
                    auto targetAngle = GetTargetAngles(target)[0];
                    maxDownRadar = std::min(maxDownRadar, newWillAngR - targetAngle);
                    maxUpRadar = std::min(maxUpRadar, targetAngle - newWillAngL);

                    auto segIdx = InWhichSegment(invertedWillDeadZones, targetAngle);
                    if (segIdx != -1) {
                        maxDownShip = std::min(maxDownShip, invertedWillDeadZones[segIdx].second - targetAngle);
                        maxUpShip = std::min(maxUpShip, targetAngle - invertedWillDeadZones[segIdx].first);
                    }
                }
            }
            for (const auto* target : targetsOutsideResponsible) {
                if (IsInVector(followedTargetIds, target->GetId())) {
                    timeToReach = std::max(timeToReach, target->GetTimeToMeetPoint());
                    auto targetAngle = GetTargetAngles(target)[0];
                    maxDownRadar = std::min(maxDownRadar, newWillAngR - targetAngle);
                    maxUpRadar = std::min(maxUpRadar, targetAngle - newWillAngL);

                    auto segIdx = InWhichSegment(invertedWillDeadZones, targetAngle);
                    if (segIdx != -1) {
                        maxDownShip = std::min(maxDownShip, invertedWillDeadZones[segIdx].second - targetAngle);
                        maxUpShip = std::min(maxUpShip, targetAngle - invertedWillDeadZones[segIdx].first);
                    }
                }
            }

            if (nextTargetAngle > newRadarTargetAngle) {
                newRadarTargetAngle += maxUpRadar;
            } else {
                newRadarTargetAngle -= maxDownRadar;
            }

            for (auto seg : willDeadZones) {
                if (IsInSegment(nextTargetAngle, seg.first - margin, seg.second + margin)) {
                    if (nextTargetAngle - seg.first < seg.second - nextTargetAngle) {
                        newShipTargetAngle += std::min(maxUpShip, nextTargetAngle - seg.first + margin);
                    } else {
                        newShipTargetAngle -= std::min(maxDownShip, seg.second - nextTargetAngle + margin);
                    }
                    break;
                }
            }

            timeToReach += params.general().margin_time();
        }
        return {
            {
                RadarTargetPos{.Angle=newRadarTargetAngle, .Speed=maxSpeed, .TimeToReach=timeToReach},
                RadarTargetPos{.Angle=newShipTargetAngle, .Speed=params.ship().max_angle_speed(), .TimeToReach=timeToReach}
            },
            followedTargetIds
        };
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
    if (Pos == pos) {
        return;
    }

    Pos = pos;
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
    if (UnfilteredPos == pos) {
        return;
    }

    int dt = Timer.GetElapsedTimeAsMs();
    Timer.Restart();

    UnfilteredPos = pos;

    auto filtered = ABFilter(UnfilteredPos, Pos, FilteredSpeed, dt, CurrSmallRadarMeasureCount);
    Pos = filtered.first;
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
    auto polarPos = CartesianToCylindrical(Pos);
    return polarPos.X <= rad && startAng <= polarPos.Y && polarPos.Y <= endAng;
}

std::string Target::DebugString() const {
    return std::string("TargetInfo:")
        + "\n  ID: " + std::to_string(Id)
        + "\n  Priority: " + std::to_string(Priority)
        + "\n  UnfilteredPos: " + UnfilteredPos.DebugString()
        + "\n  IsFollowed: " + std::to_string(IsFollowedFlag)
        + "\n  IsRocketLaunchedFlag: " + std::to_string(IsFollowedFlag)
        + "\n  CanBeFollowed: " + std::to_string(CanBeFollowed())
        + "\n  CurrBigRadarMeasureCount: " + std::to_string(CurrBigRadarMeasureCount)
        + "\n  CurrSmallRadarMeasureCount: " + std::to_string(CurrSmallRadarMeasureCount)
    ;
}


RadarController::RadarController(const Proto::Parameters& params, double startAngle, double shipStartAngle)
    : Params(params)
    , Pos{.Angle = startAngle, .Speed = 0}
    , TargetPos{.Angle = -1, .Speed = 0}
    , ShipPos{.Angle = shipStartAngle, .Speed = 0}
    , ShipTargetPos{.Angle = -1, .Speed = 0}
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
    for (auto* target : Targets) {
        if (target->GetPriority() == -1 && (target->NeedToUpdateEntryPoint() || target->NeedToUpdateMeetPoint())) {
            if (target->GetPresetPriority() != -1) {
                target->SetPriority(target->GetPresetPriority());
            } else {
                target->SetPriority(
                    CalculatePriority(
                        target->GetPosition(),
                        target->GetFilteredSpeed(),
                        Params.simulator().max_target_speed()
                    )
                );
            }
        }
        if (target->NeedToUpdateEntryPoint()) {
            target->SetEntryPoint(
                CalculateEntryPoint(
                    target->GetPosition(),
                    target->GetFilteredSpeed(),
                    Params.small_radar().radius()
                )
            );
            target->SetNeedToUpdateEntryPoint(false);
        }
        if (target->NeedToUpdateNearPoint()) {
            target->SetNearPoint(
                CalculateEntryPoint(
                    target->GetPosition(),
                    target->GetFilteredSpeed(),
                    Params.small_radar().radius() * 0.5
                )
            );
            target->SetNeedToUpdateNearPoint(false);
        }
        if (target->NeedToUpdateMeetPoint() && !target->IsRocketLaunched()) {
            const double timeToCalculatePrecizeSpeed =
                Params.general().small_radar_measure_cnt() / Params.small_radar().frequency() * 1000;
            double timeToHit = Params.defense().time_to_launch_rocket();
            if (IsTargetInRadarSector(target)) {
                timeToHit += target->GetMeasureCountToPreciseSpeed() / Params.small_radar().frequency() * 1000;
            } else {
                auto timeToRotate = TimeToRotateToTarget(
                    Pos,
                    TargetPos,
                    {target->GetPosAngle()},
                    Params.small_radar().max_angle_speed(),
                    Params.small_radar().max_eps(),
                    Params.small_radar().view_angle(),
                    Params.general().margin_angle()
                );
                if (
                    !target->CanBeInRadarSector()
                    && target->GetTimeToEntryPoint() + timeToCalculatePrecizeSpeed > timeToRotate
                ) {
                    timeToHit += target->GetTimeToEntryPoint() + timeToCalculatePrecizeSpeed;
                } else {
                    timeToHit += timeToRotate;
                }
            }
            target->SetApproximateMeetPoint(CalculateMeetPoint(
                target->GetPosition() + target->GetFilteredSpeed() * timeToHit,
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

    auto res = CalculateRadarPosImproved(
        Pos,
        TargetPos,
        ShipPos,
        ShipTargetPos,
        FollowedTargetIds,
        targetsInsideResponsible,
        targetsOutsideResponsible,
        Params
    );

    TargetPos = res.first.first;
    ShipTargetPos = res.first.second;
    FollowedTargetIds = res.second;

    for (auto* target : Targets) {
        auto id = target->GetId();
        if (IsInVector(FollowedTargetIds, id) && target->CanLaunchRocket() && !target->IsRocketLaunched()) {
            auto meetPoint = CalculateMeetPoint(
                target->GetPosition() + target->GetFilteredSpeed() * Params.defense().time_to_launch_rocket(),
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

    if (ShipTargetPos.Angle != -1 || TargetPos.Angle != -1) {
        auto PrevShipPosAngle = ShipPos.Angle;
        ShipPos = UpdateRadarPos(ShipPos, ShipTargetPos, Params.ship().max_eps(), ms);

        Pos.Angle += ShipPos.Angle - PrevShipPosAngle;
        Pos = UpdateRadarPos(Pos, TargetPos, Params.small_radar().max_eps(), ms);
    }

    RadarController::Result res{
        .RadarAngle = Pos.Angle,
        .ShipAngle = ShipPos.Angle,
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
    auto polarPos = CartesianToCylindrical(target->GetPosition());
    return polarPos.X <= Params.small_radar().radius() && startAng <= polarPos.Y && polarPos.Y <= endAng;
}

bool RadarController::IsTargetInResponsibleSector(const RC::Target* target) const {
    auto meetPoint = target->GetApproximateMeetPoint();
    if (meetPoint == Vector3d::Zero())
        return true;
    auto meetPointPolar = CartesianToCylindrical(meetPoint);
    return
        Params.small_radar().responsible_sector_start() <= meetPointPolar.Y
        && meetPointPolar.Y <= Params.small_radar().responsible_sector_end();
}

RadarController::~RadarController() {
    for (auto* target : Targets) {
        delete target;
    }
}
