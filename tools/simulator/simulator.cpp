#include "simulator.h"

#include <math.h>


Target::Target(unsigned int id, double priority, TripleDouble pos, TripleDouble speed)
    : Id(id), Priority(priority)
    , X(pos[0]), Y(pos[1]), Z(pos[2])
    , SpeedX(speed[0]), SpeedY(speed[1]), SpeedZ(speed[2])
{}

void Target::UpdatePosition(unsigned int ms) {
    // Rad -= SpeedRad * double(ms);
    // Ang += SpeedAng * double(ms);
    // H   += SpeedH   * double(ms);
    X += SpeedX * double(ms);
    Y += SpeedY * double(ms);
    Z += SpeedZ * double(ms);
}

SmallRadarData Target::GetSmallData() const {
    auto cylindrPos = CartesianToCylindrical(X, Y, Z);
    SmallRadarData res{
        .Id = Id,
        .Priority = Priority,
        .Rad = cylindrPos[0],
        .Ang = cylindrPos[1],
        .H = cylindrPos[2]
    };
    return res;
}

BigRadarData Target::GetBigData() const {
    BigRadarData res {
        GetSmallData(),
        .SpeedX = SpeedX,
        .SpeedY = SpeedY,
        .SpeedZ = SpeedZ
    };
    return res;
}

unsigned int Target::GetId() const {
    return Id;
}

bool Target::IsInSector(double rad, double angView, double angPos) const {
    double startAng = angPos - angView / 2;
    double endAng = angPos + angView / 2;
    auto polarPos = CartesianToPolar(X, Y);
    return polarPos[0] <= rad && startAng <= polarPos[1] && polarPos[1] <= endAng;
}

bool Target::IsOutOfView(double rad) const {
    const float error = 1;
    return Y < error || X*X + Y*Y > (rad + error) * (rad + error) || Z < error;
}


Simulator::Simulator(const Flat::Parameters& params)
    : Params(params)
    , BigRadarUpdatePeriod(1000 / Params.big_radar()->frequency())
    , NewTargetProbability((float) Params.simulator()->targets_per_minute() / Params.small_radar()->frequency() / 60)
    , SmallRadarPosition(1.57079)
{}

bool Simulator::IsTargetInSector(const Target& target) const {
    return target.IsInSector(Params.small_radar()->radius(), Params.small_radar()->view_angle(), SmallRadarPosition);
}

void Simulator::UpdateTargets() {
    int smallTimeDelta = SmallRadarTimer.GetElapsedTimeAsMs();
    SmallRadarTimer.Restart();
    for (auto& target : Targets) {
        if (IsTargetInSector(target)) {
            target.UpdatePosition(smallTimeDelta);
        }
    }
    int bigTimeDelta = BigRadarTimer.GetElapsedTimeAsMs();
    if (bigTimeDelta >= BigRadarUpdatePeriod) {
        BigRadarTimer.Restart();
        for (auto& target : Targets) {
            if (!IsTargetInSector(target)) {
                target.UpdatePosition(bigTimeDelta);
            }
        }
    }

    if (GetRandomTrue(NewTargetProbability)) {
        AddNewTarget();
    }

    std::vector<unsigned int> FlownAwayTargetIds;
    for (const auto& target : Targets) {
        if (target.IsOutOfView(Params.big_radar()->radius())) {
            FlownAwayTargetIds.push_back(target.GetId());
        }
    }
    RemoveTargets(FlownAwayTargetIds);
}

void Simulator::SetRadarPosition(double angPos) {
    SmallRadarPosition = angPos;
}

void Simulator::AddNewTarget() {
    static int lastId = 0;

    auto priority = GetRandomDouble(0, 1);
    auto posAng = GetRandomDouble(0, M_PI);
    auto posH = GetRandomDouble(0, Params.simulator()->max_height());

    auto speedAbs = GetRandomDouble(Params.simulator()->min_speed(), Params.simulator()->max_speed());
    double speedAngVertical, speedHorizontal;
    if (GetRandomTrue(Params.simulator()->probability_of_accurate_missile())) {
        speedAngVertical = posAng - M_PI;
        speedHorizontal = - speedAbs * posH / Params.big_radar()->radius();
    } else {
        auto deviationAngVertical = GetRandomDouble(0, 2 * Params.simulator()->max_deviation_angle_vertical());
        speedAngVertical = posAng - M_PI + Params.simulator()->max_deviation_angle_vertical() - deviationAngVertical;
        speedHorizontal = - speedAbs * posH / Params.big_radar()->radius() * GetRandomDouble(0.7, 1.2);
    }

    Targets.emplace_back(
        lastId,
        priority,
        CylindricalToCartesian(Params.big_radar()->radius(), posAng, posH),
        CylindricalToCartesian(speedAbs, speedAngVertical, speedHorizontal)
    );
    ++lastId;
}

void Simulator::RemoveTargets(std::vector<unsigned int> ids) {
    for (auto id : ids) {
        for (int i = 0; i < Targets.size(); ++i) {
            if (Targets[i].GetId() == id) {
                Targets.erase(Targets.begin() + i);
                break;
            }
        }
    }
}

std::vector<BigRadarData> Simulator::GetBigRadarTargets() const {
    std::vector<BigRadarData> res;
    for (const auto& target : Targets) {
        res.push_back(target.GetBigData());
        // res.back().Rad += GetRandomDouble(-1, 1) * Params.big_radar()->rad_error();
        // res.back().Ang += GetRandomDouble(-1, 1) * Params.big_radar()->ang_error();
        // res.back().H   += GetRandomDouble(-1, 1) * Params.big_radar()->h_error();
    }
    return res;
}

std::vector<SmallRadarData> Simulator::GetSmallRadarTargets() const {
    std::vector<SmallRadarData> res;
    for (const auto target : Targets) {
        if (IsTargetInSector(target)) {
            res.emplace_back(target.GetSmallData());
            // res.back().Rad += GetRandomDouble(-1, 1) * Params.small_radar()->rad_error();
            // res.back().Ang += GetRandomDouble(-1, 1) * Params.small_radar()->ang_error();
            // res.back().H   += GetRandomDouble(-1, 1) * Params.small_radar()->h_error();
        }
    }
    return res;
}
