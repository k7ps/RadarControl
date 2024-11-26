#include "simulator.h"

#include "util/util.h"

#include <iostream>
#include <math.h>

using namespace SIM;


Target::Target(unsigned int id, double priority, Vector3d pos, Vector3d speed, int bigPeriod)
    : Id(id), Priority(priority), Pos(pos), Speed(speed), BigRadarUpdatePeriodMs(bigPeriod)
{}

void Target::UpdatePosition(bool isInSector) {
    if (!isInSector && Timer.GetElapsedTimeAsMs() < BigRadarUpdatePeriodMs) {
        return;
    }

    double ms = Timer.GetElapsedTimeAsMs();
    Timer.Restart();

    Pos += Speed * ms;

    IsSmallDataUpdated = true;
    IsBigDataUpdated = true;
    WasUpdatedFlag = true;
}

Vector3d Target::GetCurrentPosition() const {
    return Pos + Speed * Timer.GetElapsedTimeAsMs();
}

SmallRadarData Target::GetNoisedSmallData(Vector3d errors) {
    if (IsSmallDataUpdated) {
        auto cylindrPos = CartesianToCylindrical(Pos);
        SmallData = SmallRadarData{
            .Id = Id,
            .Priority = Priority,
            .Rad = cylindrPos.X + GetRandomDouble(-1, 1) * errors.X,
            .Ang = cylindrPos.Y + GetRandomDouble(-1, 1) * errors.Y,
            .H   = cylindrPos.Z + GetRandomDouble(-1, 1) * errors.Z
        };
        IsSmallDataUpdated = false;
    }
    return SmallData;
}

BigRadarData Target::GetNoisedBigData(Vector3d errors) {
    if (IsBigDataUpdated) {
        auto cylindrPos = CartesianToCylindrical(Pos);
        BigData = BigRadarData{
            {
                .Id = Id,
                .Priority = Priority,
                .Rad = cylindrPos.X + GetRandomDouble(-1, 1) * errors.X,
                .Ang = cylindrPos.Y + GetRandomDouble(-1, 1) * errors.Y,
                .H   = cylindrPos.Z + GetRandomDouble(-1, 1) * errors.Z
            },
            .Speed = Speed
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
    auto polarPos = CartesianToCylindrical(GetCurrentPosition());
    return polarPos.X <= rad && startAng <= polarPos.Y && polarPos.Y <= endAng;
}

bool Target::IsOutOfView(double rad) const {
    const float error = 1;
    auto curPos = GetCurrentPosition();
    return curPos.Y < error
        || curPos.X * curPos.X + curPos.Y * curPos.Y > (rad + error) * (rad + error)
        || curPos.Z < error;
}

bool Target::WasUpdated() const {
    return WasUpdatedFlag;
}

void Target::SetWasUpdated(bool flag) {
    WasUpdatedFlag = flag;
}


Simulator::Simulator(const Proto::Parameters& params)
    : Params(params)
    , BigRadarUpdatePeriodMs(1000. / Params.big_radar().frequency())
    , NewTargetProbability((double) Params.simulator().targets_per_minute() / Params.small_radar().frequency() / 60)
    , SmallRadarAngPosition(1.57079)
{}

bool Simulator::IsTargetInSector(const Target& target) const {
    return target.IsInSector(Params.small_radar().radius(), Params.small_radar().view_angle(), SmallRadarAngPosition);
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
        if (target.IsOutOfView(Params.big_radar().radius())) {
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
    auto posH = GetRandomDouble(0, Params.simulator().max_height());

    auto speedAbs = GetRandomDouble(Params.simulator().min_target_speed(), Params.simulator().max_target_speed());
    double speedAngVertical, speedHorizontal;
    if (GetRandomTrue(Params.simulator().probability_of_accurate_missile())) {
        speedAngVertical = posAng - M_PI;
        speedHorizontal = - speedAbs * posH / Params.big_radar().radius();
    } else {
        auto deviationAngVertical = GetRandomDouble(0, 2 * Params.simulator().max_deviation_angle_vertical());
        speedAngVertical = posAng - M_PI + Params.simulator().max_deviation_angle_vertical() - deviationAngVertical;
        speedHorizontal = - speedAbs * posH / Params.big_radar().radius() * GetRandomDouble(0.7, 1.2);
    }

    Targets.emplace_back(
        lastId,
        priority,
        CylindricalToCartesian(Params.big_radar().radius(), posAng, posH),
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
        res.push_back(target.GetNoisedBigData(Vector3d(
            Params.big_radar().rad_error(),
            Params.big_radar().ang_error(),
            Params.big_radar().h_error()
        )));
    }
    return res;
}

std::vector<SmallRadarData> Simulator::GetSmallRadarTargets() {
    std::vector<SmallRadarData> res;
    for (auto& target : Targets) {
        if (IsTargetInSector(target)) {
            res.emplace_back(target.GetNoisedSmallData(Vector3d(
                Params.small_radar().rad_error(),
                Params.small_radar().ang_error(),
                Params.small_radar().h_error()
            )));
        }
    }
    return res;
}
