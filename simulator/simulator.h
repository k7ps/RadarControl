#ifndef SIMULATOR_H
#define SIMULATOR_H

#include "radar_control/lib/data.h"
#include "proto/generated/params.pb.h"
#include "proto/generated/scenario.pb.h"
#include "util/points.h"
#include "util/timer.h"


namespace SIM {

    class Target {
    public:
        Target(
            const Proto::Parameters& params,
            unsigned int id,
            double presetPriority,
            Vector3d pos,
            Vector3d speed,
            double msFromStart = 0
        );

        void UpdatePosition(bool isInSector);

        SmallRadarData GetSmallRadarData() const;
        BigRadarData GetBigRadarData() const;
        unsigned int GetId() const;

        bool IsInSector(double rad, double angView, double angPos) const;
        bool IsOutOfView(double rad) const;

        bool WasUpdated() const;
        void SetWasUpdated(bool flag);

    private:
        Vector3d GetCurrentRealPosition() const;

    private:
        const Proto::Parameters& Params;

        int Id;
        double PresetPriority = -1;

        Vector3d RealPos;
        Vector3d NoisedPos;
        Vector3d FilteredPos;

        Vector3d RealSpeed;
        Vector3d FilteredSpeed;

        SimpleTimer Timer;

        int BigRadarUpdatePeriodMs;

        bool WasUpdatedFlag = true;
        int MeasureCount = 1;
    };

}


struct LaunchParams {
    double PresetPriority;
    double AngPos;
    double HeightPos;
    double SpeedAbs;
    double AngDeviation;
    double HSpeedCoef;
    double MsFromStart;
};

LaunchParams GetRandomLaunchParams(const Proto::Parameters& params, bool isAccurate);


class Simulator {
public:
    Simulator(const Proto::Parameters& params, bool isUsingScenario);

    void UpdateTargets();
    void SetRadarPosition(double angPos);

    void RemoveTargets(std::vector<int> ids);

    std::vector<BigRadarData> GetBigRadarTargets();
    std::vector<SmallRadarData> GetSmallRadarTargets();

    void LaunchTarget(LaunchParams launchParams);
    void LaunchRandomTarget();

    ~Simulator();

private:
    bool IsTargetInSector(const SIM::Target& target) const;

private:
    const Proto::Parameters& Params;

    std::vector<SIM::Target*> Targets;

    const float NewTargetProbability;
    const bool IsUsingScenario;

    double SmallRadarAngPosition;
};


class TargetScheduler {
public:
    TargetScheduler(const Proto::Parameters& params);

    void SetScenario(const std::string& filename, double playSpeed);

    void LaunchTargets(Simulator& simulator);

private:
    const Proto::Parameters& Params;

    std::vector<Proto::TargetScenario::Launch> TargetLaunches;
    SimpleTimer Timer;
};


#endif // SIMULATOR_H
