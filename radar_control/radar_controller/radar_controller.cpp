#include "radar_controller.h"
#include "util/points.h"
#include "util/util.h"
#include "lib/calculations.h"

#include <cmath>
#include <iostream>
#include <stdexcept>

using namespace RC;


namespace {
    std::pair<double, double> ABFilter1d(double x, double prevX, double prevSpeed, double dt, int measureCount) {
        dt /= 1000.;
        if (measureCount == 0) {
            return {x, 0.};
        }
        if (measureCount == 1) {
            return {x, (x - prevX) / dt};
        }

        double alpha = 2. * (2. * measureCount - 1.) / (measureCount * (measureCount + 1.));
        double beta = 6. / (measureCount * (measureCount + 1.));

        double predictedX = prevX + prevSpeed * dt;
        double predictedSpeed = prevSpeed;

        double filteredX = predictedX + (alpha * (x - predictedX));
        double filteredSpeed = predictedSpeed + (beta / dt * (x - filteredX));

        return {filteredX, filteredSpeed};
    }
}


Target::Target(int id, double priority, double deathTime)
    : Id(id), Priority(priority), DeathTime(deathTime)
{}

Target::Target(const BigRadarData& data, double deatTime)
    : Target(data.Id, data.Priority, deatTime)
{
    BigRadarUpdate(data.Pos, data.Speed);
}

void Target::BigRadarUpdate(Vector3d pos, Vector3d speed) {
    Timer.Restart();

    FilteredPos = pos;
    FilteredSpeed = speed;
}

void Target::SmallRadarUpdate(Vector3d pos) {
    if (Pos == pos) {
        return;
    }

    int dt = Timer.GetElapsedTimeAsMs();
    Timer.Restart();

    Pos = pos;
    ABFilterIterate(dt);
}

void Target::ABFilterIterate(double dt) {
    auto prevPos = FilteredPos;
    auto prevSpeed = FilteredSpeed;

    auto filtered = ABFilter(Pos, prevPos, prevSpeed, dt, MeasureCount);
    FilteredPos = filtered.first;
    FilteredSpeed = filtered.second;

    ++MeasureCount;
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
    return MeasureCount > 200;
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
        auto targetPos = target.GetFilteredPosition();
        auto cylindricalPos = CartesianToCylindrical(targetPos);

        RadarAngleTarget = cylindricalPos.Y;

        if (!target.IsRocketLaunched() && target.HavePreciseSpeed()) {
            auto meetingPoint = CalculateMeetingPoint(
                targetPos + target.GetFilteredSpeed() * Params.defense().time_to_launch_rocket(),
                target.GetFilteredSpeed(),
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
