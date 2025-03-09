#include "simulator.h"

#include "proto/generated/scenario.pb.h"
#include "radar_control/lib/calculations.h"
#include "radar_control/lib/data.h"
#include "util/points.h"
#include "util/proto.h"
#include "util/util.h"

#include <algorithm>
#include <iostream>
#include <math.h>

using namespace SIM;


Target::Target(
    const Proto::Parameters& params,
    unsigned int id,
    double priority,
    Vector3d pos,
    Vector3d speed,
    double msFromStart
)
    : Params(params)
    , Id(id)
    , Priority(priority)
    , RealPos(pos)
    , RealSpeed(speed)
    , BigRadarUpdatePeriodMs(1000. / Params.big_radar().frequency())
{
    RealPos += RealSpeed * msFromStart;
    FilteredPos = RealPos;
}

void Target::UpdatePosition(bool isInSector) {
    if (!isInSector && Timer.GetElapsedTimeAsMs() < BigRadarUpdatePeriodMs) {
        return;
    }
    Vector3d stddev;
    if (isInSector) {
        stddev = Vector3d(
            Params.small_radar().rad_stddev(),
            Params.small_radar().ang_stddev(),
            Params.small_radar().h_stddev()
        );
    } else {
        stddev = Vector3d(
            Params.big_radar().rad_stddev(),
            Params.big_radar().ang_stddev(),
            Params.big_radar().h_stddev()
        );
    }

    double dt = Timer.GetElapsedTimeAsMs();
    Timer.Restart();

    RealPos += RealSpeed * dt;

    // NoisedPos = CylindricalToCartesian(CartesianToCylindrical(RealPos) + GetRandomVector3d(-1, 1) * errors);
    NoisedPos = CylindricalToCartesian(
        CartesianToCylindrical(RealPos) + GetRandomNormalVector3d(Vector3d::Zero(), stddev)
    );

    auto filtered = ABFilter(NoisedPos, FilteredPos, FilteredSpeed, dt, MeasureCount);
    FilteredPos = filtered.first;
    FilteredSpeed = filtered.second;

    WasUpdatedFlag = true;
    ++MeasureCount;
}

Vector3d Target::GetCurrentRealPosition() const {
    return RealPos + RealSpeed * Timer.GetElapsedTimeAsMs();
}

SmallRadarData Target::GetSmallRadarData() const {
    return SmallRadarData{
        .Id = Id,
        .Priority = Priority,
        .Pos = NoisedPos
    };
}

BigRadarData Target::GetBigRadarData() const {
    return BigRadarData{
        SmallRadarData{
            .Id = Id,
            .Priority = Priority,
            .Pos = FilteredPos
        },
        .Speed = FilteredSpeed
    };
}

unsigned int Target::GetId() const {
    return Id;
}

bool Target::IsInSector(double rad, double angView, double angPos) const {
    double startAng = angPos - angView / 2;
    double endAng = angPos + angView / 2;
    auto polarPos = CartesianToCylindrical(GetCurrentRealPosition());
    return polarPos.X <= rad && startAng <= polarPos.Y && polarPos.Y <= endAng;
}

bool Target::IsOutOfView(double rad) const {
    const float error = 1;
    auto curPos = GetCurrentRealPosition();
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


LaunchParams GetRandomLaunchParams(const Proto::Parameters& params, bool isAccurate) {
    double angDeviation = 0;
    double hSpeedCoef = 1;
    if (!isAccurate) {
        angDeviation = GetRandomDouble(
            -params.simulator().max_deviation_angle_vertical(),
            params.simulator().max_deviation_angle_vertical()
        );
        hSpeedCoef = GetRandomDouble(0.7, 1.2);
    }
    return LaunchParams{
        .AngPos = GetRandomDouble(0, M_PI),
        .HeightPos = GetRandomDouble(0, params.simulator().max_height()),
        .Priority = GetRandomDouble(0, 1),
        .SpeedAbs = GetRandomDouble(params.simulator().min_target_speed(), params.simulator().max_target_speed()),
        .AngDeviation = angDeviation,
        .HSpeedCoef = hSpeedCoef
    };
}


Simulator::Simulator(const Proto::Parameters& params, bool isUsingScenario)
    : Params(params)
    , NewTargetProbability((double) Params.simulator().targets_per_minute() / Params.small_radar().frequency() / 60)
    , IsUsingScenario(isUsingScenario)
    , SmallRadarAngPosition(1.57079)
{}

bool Simulator::IsTargetInSector(const Target& target) const {
    return target.IsInSector(Params.small_radar().radius(), Params.small_radar().view_angle(), SmallRadarAngPosition);
}

void Simulator::UpdateTargets() {
    for (auto* target : Targets) {
        target->UpdatePosition(IsTargetInSector(*target));
    }

    if (!IsUsingScenario && GetRandomTrue(NewTargetProbability)) {
        LaunchRandomTarget();
    }

    std::vector<unsigned int> FlownAwayTargetIds;
    for (const auto* target : Targets) {
        if (target->IsOutOfView(Params.big_radar().radius())) {
            FlownAwayTargetIds.push_back(target->GetId());
        }
    }
    RemoveTargets(FlownAwayTargetIds);
}

void Simulator::SetRadarPosition(double angPos) {
    SmallRadarAngPosition = angPos;
}

void Simulator::RemoveTargets(std::vector<unsigned int> ids) {
    for (auto id : ids) {
        for (int i = 0; i < Targets.size(); ++i) {
            if (Targets[i]->GetId() == id) {
                delete Targets[id];
                Targets.erase(Targets.begin() + i);
                break;
            }
        }
    }
}

std::vector<BigRadarData> Simulator::GetBigRadarTargets() {
    std::vector<BigRadarData> res;
    for (const auto* target : Targets) {
        res.push_back(target->GetBigRadarData());
    }
    return res;
}

std::vector<SmallRadarData> Simulator::GetSmallRadarTargets() {
    std::vector<SmallRadarData> res;
    for (const auto* target : Targets) {
        if (IsTargetInSector(*target)) {
            res.emplace_back(target->GetSmallRadarData());
        }
    }
    return res;
}

void Simulator::LaunchTarget(LaunchParams launchParams) {
    static int lastId = 0;

    double speedAngVertical = launchParams.AngPos - M_PI - launchParams.AngDeviation;
    double speedHorizontal = - launchParams.SpeedAbs * launchParams.HeightPos
                            / Params.big_radar().radius() * launchParams.HSpeedCoef;

    auto* targetPtr = new Target(
        Params,
        lastId,
        launchParams.Priority,
        CylindricalToCartesian(Params.big_radar().radius(), launchParams.AngPos, launchParams.HeightPos),
        CylindricalToCartesian(launchParams.SpeedAbs, speedAngVertical, speedHorizontal),
        launchParams.MsFromStart
    );

    Targets.push_back(targetPtr);
    ++lastId;
}

void Simulator::LaunchRandomTarget() {
    LaunchTarget(GetRandomLaunchParams(Params, GetRandomTrue(Params.simulator().probability_of_accurate_missile())));
}


TargetScheduler::TargetScheduler(const Proto::Parameters& params)
    : Params(params)
{}

void TargetScheduler::SetScenario(const std::string& filename, double playSpeed) {
    const auto scenario = ParseProtoFromFile<Proto::TargetScenario>(filename);
    for (const auto& launch : scenario.launches()) {
        Proto::TargetScenario::Launch correctLaunch(launch);
        correctLaunch.set_time(correctLaunch.time() * 1000. / playSpeed);
        correctLaunch.set_angle_pos(correctLaunch.angle_pos() * M_PI / 180);
        if (launch.has_angle_deviation()) {
            correctLaunch.set_angle_deviation(correctLaunch.angle_deviation() * M_PI / 180);
        }
        if (launch.has_abs_speed()) {
            correctLaunch.set_abs_speed(correctLaunch.abs_speed() * playSpeed / 1000);
        }

        TargetLaunches.push_back(correctLaunch);
    }

    std::sort(
        TargetLaunches.begin(),
        TargetLaunches.end(),
        [](const Proto::TargetScenario::Launch& l, const Proto::TargetScenario::Launch& r) {
            return l.time() < r.time();
        }
    );
}

void TargetScheduler::LaunchTargets(Simulator& simulator) {
    static int unlaunched = 0;
    auto currTime = Timer.GetElapsedTimeAsMs();

    for (; unlaunched < TargetLaunches.size(); ++unlaunched) {
        const auto& protoLaunchParams = TargetLaunches[unlaunched];
        if (protoLaunchParams.time() <= currTime) {
            LaunchParams launchParams;
            if (protoLaunchParams.has_is_accurate()) {
                launchParams = GetRandomLaunchParams(Params, protoLaunchParams.is_accurate());
            } else {
                launchParams = GetRandomLaunchParams(
                    Params,
                    GetRandomTrue(Params.simulator().probability_of_accurate_missile())
                );
            }

            launchParams.MsFromStart = currTime - protoLaunchParams.time();
            launchParams.AngPos = protoLaunchParams.angle_pos();
            if (protoLaunchParams.has_priority()) {
                launchParams.Priority = protoLaunchParams.priority();
            }
            if (protoLaunchParams.has_height()) {
                launchParams.HeightPos = protoLaunchParams.height();
            }
            if (protoLaunchParams.has_abs_speed()) {
                launchParams.SpeedAbs = protoLaunchParams.abs_speed();
            }
            if (protoLaunchParams.has_angle_deviation()) {
                launchParams.AngDeviation = protoLaunchParams.angle_deviation();
            }

            simulator.LaunchTarget(launchParams);
        } else {
            break;
        }
    }
}
