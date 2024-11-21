#ifndef SIMULATOR_H
#define SIMULATOR_H

#include "radar_control/lib/data.h"
#include "flat/generated/params.h"
#include "util/timer.h"


namespace SIM {

    class Target {
    public:
        Target(unsigned int id, double priority, Vector3d pos, Vector3d speed, int bigPeriod);

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


class Simulator {
public:
    Simulator(const Flat::Parameters& params);

    void UpdateTargets();
    void SetRadarPosition(double angPos);

    void RemoveTargets(std::vector<unsigned int> ids);

    std::vector<BigRadarData> GetBigRadarTargets();
    std::vector<SmallRadarData> GetSmallRadarTargets();

    std::vector<BigRadarData> GetOnlyUpdatedTargets();

private:
    void AddNewTarget();
    bool IsTargetInSector(const SIM::Target& target) const;

private:
    const Flat::Parameters& Params;

    std::vector<SIM::Target> Targets;

    const int BigRadarUpdatePeriodMs;
    const float NewTargetProbability;

    double SmallRadarAngPosition;
};


#endif // SIMULATOR_H
