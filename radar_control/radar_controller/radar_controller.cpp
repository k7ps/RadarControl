#include "radar_controller.h"
#include "util/points.h"
#include "util/util.h"
#include "lib/calculations.h"

#include <iostream>
#include <math.h>

using namespace RC;


// namespace {
//     BigRadarData GetTargetDataById(const std::vector<BigRadarData>& datas, unsigned id) {
//         for (const auto& data : datas) {
//             if (data.Id == id) {
//                 return data;
//             }
//         }
//         return BigRadarData{
//             {
//                 .Id = -1
//             }
//         };
//     }

//     SmallRadarData GetTargetDataById(const std::vector<SmallRadarData>& datas, unsigned id) {
//         for (const auto& data : datas) {
//             if (data.Id == id) {
//                 return data;
//             }
//         }
//         return SmallRadarData{
//             .Id = -1
//         };
//     }
// }


Target::Target(int id, double priority, double deathTime)
    : Id(id), Priority(priority), DeathTime(deathTime)
{}

Target::Target(const BigRadarData& data, double deatTime)
    : Target(data.Id, data.Priority, deatTime)
{
    Update(CylindricalToCartesian(data.Rad, data.Ang, data.H), data.Speed);
}

void Target::Update(Vector3d pos) {
    if (Pos == pos) {
        return;
    }

    int ms = Timer.GetElapsedTimeAsMs();
    Timer.Restart();

    Positions.push_back(pos);
    if (Positions.size() == 1) {
        DeltaTimes.push_back(0);
        return;
    }
    DeltaTimes.push_back(ms);
    Speed = CalculateSpeed(Positions, DeltaTimes);
    if (Positions.size() >= 100) {
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


RadarController::RadarController(const Flat::Parameters& params)
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

            target.Update(CylindricalToCartesian(data.Rad, data.Ang, data.H));
            updatedTargets.insert(data.Id);
            break;
        }
    }
    for (const auto& data : bigDatas) {
        for (auto& target : Targets) {
            if (target.GetId() != data.Id || updatedTargets.count(data.Id)) continue;

            target.Update(CylindricalToCartesian(data.Rad, data.Ang, data.H), data.Speed);
            updatedTargets.insert(data.Id);
            break;
        }
        if (!updatedTargets.count(data.Id)) {
            Targets.push_back(Target(data, Params.general()->death_time()));
        }
    }

    if (!bigDatas.empty() && FollowedTargetIds.empty()) {
        FollowedTargetIds.push_back(bigDatas.front().Id);
    }
    if (!FollowedTargetIds.empty()) {
        int id = FollowedTargetIds.front();
        auto& target = GetTargetById(id);
        auto targetPos = target.GetPosition();
        auto cylindricalPos = CartesianToCylindrical(target.GetPosition());

        RadarAngleTarget = cylindricalPos.Y;

        if (!target.IsRocketLaunched() && target.HavePreciseSpeed()) {
            auto meetingPoint = CalculateMeetingPoint(
                targetPos,
                target.GetSpeed(),
                Vector3d(),
                Params.defense()->rocket_speed()
            );
            MeetingPointsAndTargetIds.emplace_back(meetingPoint, id);
            target.SetIsRocketLaunched(true);
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
}


RadarController::Result RadarController::GetAngleAndMeetingPoints() {
    double ms = Timer.GetElapsedTimeAsMs();
    Timer.Restart();

    if (RadarAngleTarget != -1) {
        double maxDelta = Params.small_radar()->angle_speed() * ms / 1000;

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

    auto res = RadarController::Result{
        .Angle = RadarAnglePos,
        .MeetingPointsAndTargetIds = MeetingPointsAndTargetIds
    };
    MeetingPointsAndTargetIds.clear();
    return res;
}
