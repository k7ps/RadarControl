#include "simulator.h"

#include <math.h>


Target::Target(unsigned int id, double priority, TripleDouble pos, TripleDouble speed, int bigPeriod)
    : Id(id), Priority(priority)
    , X(pos[0]), Y(pos[1]), Z(pos[2])
    , SpeedX(speed[0]), SpeedY(speed[1]), SpeedZ(speed[2])
    , BigRadarUpdatePeriodMs(bigPeriod)
{}

void Target::UpdatePosition(bool isInSector) {
    double ms = Timer.GetElapsedTimeAsMs();
    if (!isInSector && ms < BigRadarUpdatePeriodMs) {
        return;
    }

    Timer.Restart();

    X += SpeedX * ms;
    Y += SpeedY * ms;
    Z += SpeedZ * ms;

    IsSmallDataUpdated = true;
    IsBigDataUpdated   = true;
}

TripleDouble Target::GetCurrentPosition() const {
    double ms = Timer.GetElapsedTimeAsMs();
    return TripleDouble{
        X + SpeedX * ms,
        Y + SpeedY * ms,
        Z + SpeedZ * ms
    };
}

SmallRadarData Target::GetNoisedSmallData(TripleDouble errors) {
    if (IsSmallDataUpdated) {
        auto cylindrPos = CartesianToCylindrical(X, Y, Z);
        SmallData = SmallRadarData{
            .Id = Id,
            .Priority = Priority,
            .Rad = cylindrPos[0] + GetRandomDouble(-1, 1) * errors[0],
            .Ang = cylindrPos[1] + GetRandomDouble(-1, 1) * errors[1],
            .H   = cylindrPos[2] + GetRandomDouble(-1, 1) * errors[2]
        };
        IsSmallDataUpdated = false;
    }
    return SmallData;
}

BigRadarData Target::GetNoisedBigData(TripleDouble errors) {
    if (IsBigDataUpdated) {
        auto cylindrPos = CartesianToCylindrical(X, Y, Z);
        BigData = BigRadarData{
            {
                .Id = Id,
                .Priority = Priority,
                .Rad = cylindrPos[0] + GetRandomDouble(-1, 1) * errors[0],
                .Ang = cylindrPos[1] + GetRandomDouble(-1, 1) * errors[1],
                .H   = cylindrPos[2] + GetRandomDouble(-1, 1) * errors[2]
            },
            .SpeedX = SpeedX,
            .SpeedY = SpeedY,
            .SpeedZ = SpeedZ
        };
        IsBigDataUpdated = false;
    }
    return BigData;
}

unsigned int Target::GetId() const {
    return Id;
}

bool Target::IsInSector(double rad, double angView, double angPos) const {
    double startAng = angPos - angView / 2;
    double endAng = angPos + angView / 2;
    auto curPos = GetCurrentPosition();
    auto polarPos = CartesianToPolar(curPos[0], curPos[1]);
    return polarPos[0] <= rad && startAng <= polarPos[1] && polarPos[1] <= endAng;
}

bool Target::IsOutOfView(double rad) const {
    const float error = 1;
    auto curPos = GetCurrentPosition();
    return curPos[1] < error || curPos[0]*curPos[0] + curPos[1]*curPos[1] > (rad + error) * (rad + error) || Z < error;
}


Simulator::Simulator(const Flat::Parameters& params)
    : Params(params)
    , BigRadarUpdatePeriodMs(1000. / Params.big_radar()->frequency())
    , NewTargetProbability((float) Params.simulator()->targets_per_minute() / Params.small_radar()->frequency() / 60)
    , SmallRadarAngPosition(1.57079)
{}

bool Simulator::IsTargetInSector(const Target& target) const {
    return target.IsInSector(Params.small_radar()->radius(), Params.small_radar()->view_angle(), SmallRadarAngPosition);
}

void Simulator::UpdateTargets() {
    for (auto& target : Targets) {
        target.UpdatePosition(IsTargetInSector(target));
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
    SmallRadarAngPosition = angPos;
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
        CylindricalToCartesian(speedAbs, speedAngVertical, speedHorizontal),
        BigRadarUpdatePeriodMs
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

std::vector<BigRadarData> Simulator::GetBigRadarTargets() {
    std::vector<BigRadarData> res;
    for (auto& target : Targets) {
        res.push_back(target.GetNoisedBigData(TripleDouble{
            Params.big_radar()->rad_error(),
            Params.big_radar()->ang_error(),
            Params.big_radar()->h_error()
        }));
    }
    return res;
}

std::vector<SmallRadarData> Simulator::GetSmallRadarTargets() {
    std::vector<SmallRadarData> res;
    for (auto& target : Targets) {
        if (IsTargetInSector(target)) {
            res.emplace_back(target.GetNoisedSmallData(TripleDouble{
                Params.small_radar()->rad_error(),
                Params.small_radar()->ang_error(),
                Params.small_radar()->h_error()
            }));
        }
    }
    return res;
}
