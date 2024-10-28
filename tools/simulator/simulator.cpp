#include "simulator.h"

#include <math.h>


// Target::Target(unsigned int id, float priority, sf::Vector2f pos, sf::Vector2f speed)
//     : Id(id), Priority(priority), Position(pos), Speed(speed)
// {}

// void Target::UpdatePosition(unsigned int ms) {
//     Position += Speed * float(ms);
// }

// SmallRadarData Target::GetSmallData() const {
//     SmallRadarData res{
//         .Id = Id,
//         .Priority = Priority,
//         .X = Position.x,
//         .Y = Position.y
//     };
//     return res;
// }

// BigRadarData Target::GetBigData() const {
//     BigRadarData res {
//         GetSmallData(),
//         .SpeedX = Speed.x,
//         .SpeedY = Speed.y
//     };
//     return res;
// }

// unsigned int Target::GetId() const {
//     return Id;
// }

// bool Target::IsInSector(double rad, double angView, double angPos) const {
//     double ang1 = angPos - angView / 2;
//     double ang2 = angPos + angView / 2;
//     auto polarPos = ToPolarSystem(Position);
//     return polarPos.x <= rad && ang1 <= polarPos.y && polarPos.y <= ang2;
// }

// bool Target::IsOutOfView(double rad) const {
//     const float error = 1;
//     return Position.y > error || Position.x * Position.x + Position.y * Position.y > (rad + error) * (rad + error);
// }


Simulator::Simulator(const Flat::Parameters& params)
    : Params(params)
    , BigRadarUpdatePeriod(1000 / Params.big_radar()->frequency())
    , NewTargetProbability((float) Params.simulator()->targets_per_minute() / Params.small_radar()->frequency() / 60)
    , SmallRadarPosition(1.57079)
{
    // SmallRadarTimer.restart();
    // BigRadarTimer.restart();
}

// bool Simulator::IsTargetInSector(const Target& target) const {
//     return target.IsInSector(Params.small_radar().radius(), Params.small_radar().view_angle(), SmallRadarPosition);
// }

void Simulator::UpdateTargets() {
    // int smallTimeDelta = SmallRadarTimer.getElapsedTime().asMilliseconds();
    // SmallRadarTimer.restart();
    // for (auto& target : Targets) {
    //     if (IsTargetInSector(target)) {
    //         target.UpdatePosition(smallTimeDelta);
    //     }
    // }
    // int bigTimeDelta = BigRadarTimer.getElapsedTime().asMilliseconds();
    // if (bigTimeDelta >= BigRadarUpdatePeriod) {
    //     BigRadarTimer.restart();
    //     for (auto& target : Targets) {
    //         if (!IsTargetInSector(target)) {
    //             target.UpdatePosition(bigTimeDelta);
    //         }
    //     }
    // }

    // if (GetRandomTrue(NewTargetProbability)) {
    //     AddNewTarget();
    // }

    // std::vector<unsigned int> FlownAwayTargetIds;
    // for (const auto& target : Targets) {
    //     if (target.IsOutOfView(Params.big_radar().radius())) {
    //         FlownAwayTargetIds.push_back(target.GetId());
    //     }
    // }
    // RemoveTargets(FlownAwayTargetIds);
}

void Simulator::SetRadarPosition(double angPos) {
    SmallRadarPosition = angPos;
}

void Simulator::AddNewTarget() {
    // static int lastId = 0;

    // float priority = GetRandomFloat(0, 1);
    // float posAng = GetRandomFloat(0, M_PI);
    // float speedAbs = GetRandomFloat(Params.simulator().min_speed(), Params.simulator().max_speed());
    // float deviationAng = GetRandomFloat(0, 2 * Params.simulator().max_deviation_angle());
    // float speedAng = posAng - M_PI + Params.simulator().max_deviation_angle() - deviationAng;

    // auto pos = ToWindowXY(Params.big_radar().radius(), posAng);
    // auto speed = ToWindowXY(speedAbs, speedAng);

    // Targets.emplace_back(lastId, priority, pos, speed);
    // ++lastId;
}

void Simulator::RemoveTargets(std::vector<unsigned int> ids) {
    // for (auto id : ids) {
    //     for (int i = 0; i < Targets.size(); ++i) {
    //         if (Targets[i].GetId() == id) {
    //             Targets.erase(Targets.begin() + i);
    //             break;
    //         }
    //     }
    // }
}

std::vector<BigRadarData> Simulator::GetBigRadarTargets() const {
    // std::vector<BigRadarData> res;
    // for (const auto& target : Targets) {
    //     res.push_back(target.GetBigData());
    //     res.back().X += res.back().X * GetRandomFloat(-1, 1) * Params.big_radar().error();
    //     res.back().Y += res.back().Y * GetRandomFloat(-1, 1) * Params.big_radar().error();
    // }
    // return res;
}

std::vector<SmallRadarData> Simulator::GetSmallRadarTargets() const {
    // std::vector<SmallRadarData> res;
    // for (const auto target : Targets) {
    //     if (IsTargetInSector(target)) {
    //         res.emplace_back(target.GetSmallData());
    //         res.back().X += res.back().X * GetRandomFloat(-1, 1) * Params.small_radar().error();
    //         res.back().Y += res.back().Y * GetRandomFloat(-1, 1) * Params.small_radar().error();
    //     }
    // }
    // return res;
}
