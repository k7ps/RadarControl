#ifndef SIMULATOR_H
#define SIMULATOR_H

#include "radar_control/lib/data.h"
#include "proto/generated/params.pb.h"
#include "proto/generated/scenario.pb.h"
#include "util/timer.h"


namespace SIM {

    class Target {
    public:
        Target(unsigned int id, double priority, Vector3d pos, Vector3d speed, int bigPeriod, double msFromStart = 0);

        void UpdatePosition(bool isInSector);

        SmallRadarData GetNoisedSmallData(Vector3d errors);
        BigRadarData GetNoisedBigData(Vector3d errors);
        unsigned int GetId() const;

        bool IsInSector(double rad, double angView, double angPos) const;
        bool IsOutOfView(double rad) const;

        bool WasUpdated() const;
        void SetWasUpdated(bool flag);

    private:
        Vector3d GetCurrentPosition() const;

    private:
        int Id;
        double Priority;

        Vector3d Pos;
        Vector3d Speed;

        SimpleTimer Timer;

        int BigRadarUpdatePeriodMs;

        bool IsSmallDataUpdated = true;
        bool IsBigDataUpdated = true;

        SmallRadarData SmallData;
        BigRadarData BigData;

        bool WasUpdatedFlag = true;
    };

}


struct LaunchParams {
    double AngPos;
    double HeightPos;
    double Priority;
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

    void RemoveTargets(std::vector<unsigned int> ids);

    std::vector<BigRadarData> GetBigRadarTargets();
    std::vector<SmallRadarData> GetSmallRadarTargets();

    void LaunchTarget(LaunchParams launchParams);
    void LaunchRandomTarget();

private:
    bool IsTargetInSector(const SIM::Target& target) const;

private:
    const Proto::Parameters& Params;

    std::vector<SIM::Target> Targets;

    const int BigRadarUpdatePeriodMs;
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
