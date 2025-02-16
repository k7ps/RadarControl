#include "radar_controller.h"
#include "util/points.h"
#include "util/util.h"
#include "lib/calculations.h"

#include <cmath>
#include <iostream>
#include <math.h>
#include <stdexcept>
#include <string>

using namespace RC;


Target::Target(int id, double priority, double deathTime)
    : Id(id), Priority(priority), DeathTime(deathTime)
{}

Target::Target(const BigRadarData& data, double deatTime)
    : Target(data.Id, data.Priority, deatTime)
{
    Update(data.Pos, data.Speed);
}

void Target::Update(Vector3d pos) {
    if (Pos == pos) {
        return;
    }

    int ms = Timer.GetElapsedTimeAsMs();
    Timer.Restart();

    Pos = pos;

    Positions.push_back(Pos);
    if (Positions.size() == 1) {
        DeltaTimes.push_back(0);
        return;
    }
    DeltaTimes.push_back(ms);
    Speed = CalculateSpeed(Positions, DeltaTimes);
    if (Positions.size() >= 40) {
        HavePreciseSpeedFlag = true;
    }
}

void Target::Update(Vector3d pos, Vector3d speed) {
    if (Pos == pos) {
        return;
    }

    Timer.Restart();
    Pos = pos;
    Speed = speed;
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

Vector3d Target::GetSpeed() const {
    return Speed;
}

bool Target::HavePreciseSpeed() const {
    return HavePreciseSpeedFlag;
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


RadarController::RadarController(const Proto::Parameters& params)
    : Params(params)
    , RadarAnglePos(M_PI_2)
{}

void RadarController::Process(
    const std::vector<BigRadarData>& bigDatas,
    const std::vector<SmallRadarData>& smallDatas
) {
    std::set<int> updatedTargets;
    for (const auto& data : smallDatas) {
        for (auto& target : Targets) {
            if (target.GetId() != data.Id) continue;

            target.Update(data.Pos);
            updatedTargets.insert(data.Id);
            break;
        }
    }
    for (const auto& data : bigDatas) {
        for (auto& target : Targets) {
            if (target.GetId() != data.Id || updatedTargets.count(data.Id)) continue;

            target.Update(data.Pos, data.Speed);
            updatedTargets.insert(data.Id);
            break;
        }
        if (!updatedTargets.count(data.Id)) {
            Targets.emplace_back(data, Params.general().death_time());
        }
    }
    RemoveDeadTargets();

    if (FollowedTargetIds.empty()) {
        SelectTargetToFollow();
    }
    if (!FollowedTargetIds.empty()) {
        int id = FollowedTargetIds.front();
        auto& target = GetTargetById(id);
        auto targetPos = target.GetPosition();
        auto cylindricalPos = CartesianToCylindrical(target.GetPosition());

        RadarAngleTarget = cylindricalPos.Y;

        if (!target.IsRocketLaunched() && target.HavePreciseSpeed()) {
            auto meetingPoint = CalculateMeetingPoint(
                targetPos + target.GetSpeed() * Params.defense().time_to_launch_rocket(),
                target.GetSpeed(),
                Vector3d(),
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
