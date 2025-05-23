#ifndef SIMULATOR_H
#define SIMULATOR_H

#include "proto/generated/params.pb.h"
#include "proto/generated/scenario.pb.h"
#include "radar_control/data.h"
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

        bool IsInSector(double rad, double sectorStart, double sectorEnd) const;
        bool IsOutOfView(double rad) const;

        bool WasUpdated() const;
        void SetWasUpdated(bool flag);
        bool WasInResponsible() const;

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
        bool WasInResponsibleFlag = false;
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
    Simulator(const Proto::Parameters& params, double radarStartAngle, double shipStartAngle, bool isUsingScenario);

    void UpdateTargets();
    void SetRadarPosition(double angPos);
    void SetShipPosition(double angPos);

    void RemoveTargets(std::vector<int> ids, bool isDestroyed = true);

    std::vector<BigRadarData> GetBigRadarTargets();
    std::vector<SmallRadarData> GetSmallRadarTargets();

    void LaunchTarget(LaunchParams launchParams);
    void LaunchRandomTarget();

    bool IsThereAnyTargets() const { return !Targets.empty(); };
    std::string GetStatistics() const;

    ~Simulator();

private:
    bool IsTargetInSector(const SIM::Target& target) const;
    bool IsTargetInDeadZone(const SIM::Target& target) const;

private:
    const Proto::Parameters& Params;

    std::vector<SIM::Target*> Targets;

    const float NewTargetProbability;
    const bool IsUsingScenario;

    double SmallRadarAngPosition;
    double ShipAngPosition;

    int TargetsCount = 0;
    int ResponsibleTargetsCount = 0;
    int DestroyedTargetsCount = 0;
    int DestroyedResponsibleTargetsCount = 0;
};


class TargetScheduler {
public:
    TargetScheduler(const Proto::Parameters& params);

    void SetScenario(const std::string& filename);

    void LaunchTargets(Simulator& simulator);

    double GetRadarStartAngle() const { return RadarStartAngle; }
    double GetShipStartAngle() const { return ShipStartAngle; }

    bool IsScenarioEnded() const { return IsScenarioEndedFlag; }
    std::string GetScenarioDescription() const { return Description; };

private:
    const Proto::Parameters& Params;

    std::vector<Proto::TargetScenario::Launch> TargetLaunches;
    SimpleTimer Timer;

    double RadarStartAngle;
    double ShipStartAngle;

    bool IsScenarioEndedFlag;
    std::string Description;
};


#endif // SIMULATOR_H
