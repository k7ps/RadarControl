#include "simulator.h"
#include "proto/generated/scenario.pb.h"
#include "radar_control/calculations.h"
#include "util/points.h"
#include "util/proto.h"
#include "util/util.h"

#include <algorithm>
#include <cmath>
#include <math.h>

using namespace SIM;


Target::Target(
    const Proto::Parameters& params,
    unsigned int id,
    double presetPriority,
    Vector3d pos,
    Vector3d speed,
    double msFromStart
)
    : Params(params)
    , Id(id)
    , PresetPriority(presetPriority)
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
        if (IsInSector(
            Params.small_radar().radius(),
            Params.small_radar().responsible_sector_start(),
            Params.small_radar().responsible_sector_end()
        )) {
            WasInResponsibleFlag = true;
        }
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
        .Pos = NoisedPos
    };
}

BigRadarData Target::GetBigRadarData() const {
    return BigRadarData{
        SmallRadarData{
            .Id = Id,
            .Pos = FilteredPos
        },
        .Speed = FilteredSpeed,
        .PresetPriority = PresetPriority
    };
}

unsigned int Target::GetId() const {
    return Id;
}

bool Target::IsInSector(double rad, double sectorStart, double sectorEnd) const {
    auto polarPos = CartesianToCylindrical(GetCurrentRealPosition());
    return polarPos.X <= rad && sectorStart <= polarPos.Y && polarPos.Y <= sectorEnd;
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

bool Target::WasInResponsible() const {
    return WasInResponsibleFlag;
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
        .PresetPriority = -1,
        .AngPos = GetRandomDouble(0, M_PI),
        .HeightPos = GetRandomDouble(0, params.simulator().max_height()),
        .SpeedAbs = GetRandomDouble(params.simulator().min_target_speed(), params.simulator().max_target_speed()),
        .AngDeviation = angDeviation,
        .HSpeedCoef = hSpeedCoef
    };
}


Simulator::Simulator(const Proto::Parameters& params, double radarStartAngle, double shipStartAngle, bool isUsingScenario)
    : Params(params)
    , NewTargetProbability((double) Params.simulator().targets_per_minute() / Params.small_radar().frequency() / 60)
    , IsUsingScenario(isUsingScenario)
    , SmallRadarAngPosition(radarStartAngle)
    , ShipAngPosition(shipStartAngle)
{}

bool Simulator::IsTargetInDeadZone(const SIM::Target& target) const {
    auto deadZones = ShiftedSegmentsFromProto(Params.ship().dead_zones(), ShipAngPosition);
    for (const auto& seg : deadZones) {
        if (target.IsInSector(Params.small_radar().radius(), seg.first, seg.second)) {
            return true;
        }
    }
    return false;
}

bool Simulator::IsTargetInSector(const Target& target) const {
    return
        target.IsInSector(
            Params.small_radar().radius(),
            SmallRadarAngPosition - Params.small_radar().view_angle() / 2,
            SmallRadarAngPosition + Params.small_radar().view_angle() / 2
        )
        && !IsTargetInDeadZone(target);
}

void Simulator::UpdateTargets() {
    for (auto* target : Targets) {
        target->UpdatePosition(IsTargetInSector(*target));
    }

    if (!IsUsingScenario && GetRandomTrue(NewTargetProbability)) {
        LaunchRandomTarget();
    }

    std::vector<int> FlownAwayTargetIds;
    for (const auto* target : Targets) {
        if (target->IsOutOfView(Params.big_radar().radius())) {
            if (target->WasInResponsible()){
                ++ResponsibleTargetsCount;
            }
            FlownAwayTargetIds.push_back(target->GetId());
        }
    }
    RemoveTargets(FlownAwayTargetIds, false);
}

void Simulator::SetRadarPosition(double angPos) {
    SmallRadarAngPosition = angPos;
}

void Simulator::SetShipPosition(double angPos) {
    ShipAngPosition = angPos;
}

void Simulator::RemoveTargets(std::vector<int> ids, bool isDestroyed) {
    for (auto id : ids) {
        for (int i = 0; i < Targets.size(); ++i) {
            if (Targets[i]->GetId() == id) {
                if (isDestroyed) {
                    ++DestroyedTargetsCount;
                    if (Targets[i]->WasInResponsible()) {
                        ++DestroyedResponsibleTargetsCount;
                        ++ResponsibleTargetsCount;
                    }
                }

                delete Targets[i];
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
        launchParams.PresetPriority,
        CylindricalToCartesian(Params.big_radar().radius(), launchParams.AngPos, launchParams.HeightPos),
        CylindricalToCartesian(launchParams.SpeedAbs, speedAngVertical, speedHorizontal),
        launchParams.MsFromStart
    );

    Targets.push_back(targetPtr);
    ++lastId;
    ++TargetsCount;
}

void Simulator::LaunchRandomTarget() {
    LaunchTarget(GetRandomLaunchParams(Params, GetRandomTrue(Params.simulator().probability_of_accurate_missile())));
}

std::string Simulator::GetStatistics() const {
    std::ostringstream out;
    out << "Destroyed targets:             " << DestroyedTargetsCount << "/" << TargetsCount
        << " " << AsPercents((double) DestroyedTargetsCount / TargetsCount) << "\n"
        << "Destroyed responsible targets: " << DestroyedResponsibleTargetsCount << "/" << ResponsibleTargetsCount << " ";
    if (ResponsibleTargetsCount != 0) {
        out << AsPercents((double) DestroyedResponsibleTargetsCount / ResponsibleTargetsCount);
    } else {
        out << AsPercents(0);
    }
    return out.str();
}

Simulator::~Simulator() {
    for (auto* target : Targets) {
        delete target;
    }
}


TargetScheduler::TargetScheduler(const Proto::Parameters& params)
    : Params(params)
    , RadarStartAngle(M_PI_2)
    , ShipStartAngle(0)
    , IsScenarioEndedFlag(false)
    , Description("")
{}

void TargetScheduler::SetScenario(const std::string& filename) {
    const auto scenario = ParseProtoFromFile<Proto::TargetScenario>(filename);
    RadarStartAngle = DegToRad(scenario.radar_start_angle());
    ShipStartAngle = DegToRad(scenario.ship_start_angle());
    if (scenario.has_description()) {
        Description = scenario.description();
    }

    for (const auto& launch : scenario.launches()) {
        Proto::TargetScenario::Launch correctLaunch(launch);
        correctLaunch.set_time(correctLaunch.time() * 1000. / Params.general().play_speed());
        correctLaunch.set_angle_pos(correctLaunch.angle_pos() * M_PI / 180);
        if (launch.has_angle_deviation()) {
            correctLaunch.set_angle_deviation(correctLaunch.angle_deviation() * M_PI / 180);
        }
        if (launch.has_abs_speed()) {
            correctLaunch.set_abs_speed(correctLaunch.abs_speed() * Params.general().play_speed() / 1000);
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
                launchParams.PresetPriority = protoLaunchParams.priority();
            }
            if (protoLaunchParams.has_height_share()) {
                launchParams.HeightPos = protoLaunchParams.height_share() * Params.simulator().max_height();
            } else if (protoLaunchParams.has_height()) {
                launchParams.HeightPos = protoLaunchParams.height();
            }
            if (protoLaunchParams.has_abs_speed_share()) {
                launchParams.SpeedAbs = protoLaunchParams.abs_speed_share() * Params.simulator().max_target_speed();
            } else if (protoLaunchParams.has_abs_speed()) {
                launchParams.SpeedAbs = protoLaunchParams.abs_speed();
            }
            if (protoLaunchParams.has_angle_deviation()) {
                launchParams.AngDeviation = protoLaunchParams.angle_deviation();
            }

            simulator.LaunchTarget(launchParams);
        } else {
            return;
        }
    }
    IsScenarioEndedFlag = true;
}
