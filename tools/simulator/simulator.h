#ifndef SIMULATOR_H
#define SIMULATOR_H

#include "radar_control/lib/data.h"
#include "flat/generated/params.h"
#include "util/timer.h"
#include "util/util.h"


class Target {
public:
    Target(unsigned int id, double priority, TripleDouble pos, TripleDouble speed, int bigPeriod);

    void UpdatePosition(bool isInSector);

    SmallRadarData GetNoisedSmallData(TripleDouble errors);
    BigRadarData GetNoisedBigData(TripleDouble errors);
    unsigned int GetId() const;

    bool IsInSector(double rad, double angView, double angPos) const;
    bool IsOutOfView(double rad) const;

private:
    TripleDouble GetCurrentPosition() const;

private:
    unsigned int Id;
    double Priority;

    double X;
    double Y;
    double Z;

    double SpeedX;
    double SpeedY;
    double SpeedZ;

    int BigRadarUpdatePeriodMs;

    bool IsSmallDataUpdated = true;
    bool IsBigDataUpdated = true;

    SmallRadarData SmallData;
    BigRadarData BigData;

    SimpleTimer Timer;
};


class Simulator {
public:
    Simulator(const Flat::Parameters& params);

    void UpdateTargets();
    void SetRadarPosition(double angPos);

    void RemoveTargets(std::vector<unsigned int> ids);

    std::vector<BigRadarData> GetBigRadarTargets();
    std::vector<SmallRadarData> GetSmallRadarTargets();

private:
    void AddNewTarget();
    bool IsTargetInSector(const Target& target) const;

private:
    const Flat::Parameters& Params;

    std::vector<Target> Targets;

    const int BigRadarUpdatePeriodMs;
    const float NewTargetProbability;

    double SmallRadarAngPosition;
};


#endif // SIMULATOR_H
