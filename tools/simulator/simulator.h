#ifndef SIMULATOR_H
#define SIMULATOR_H

#include "radar_control/lib/data.h"
#include "flat/generated/params.h"
#include "util/timer.h"
#include "util/util.h"


class Target {
public:
    Target(unsigned int id, double priority, TripleDouble pos, TripleDouble speed);

    void UpdatePosition(unsigned int ms);

    SmallRadarData GetSmallData() const;
    BigRadarData GetBigData() const;
    unsigned int GetId() const;

    bool IsInSector(double rad, double angView, double angPos) const;
    bool IsOutOfView(double rad) const;

private:
    unsigned int Id;
    double Priority;

    double X;
    double Y;
    double Z;

    double SpeedX;
    double SpeedY;
    double SpeedZ;
};


class Simulator {
public:
    Simulator(const Flat::Parameters& params);

    void UpdateTargets();
    void SetRadarPosition(double angPos);

    void RemoveTargets(std::vector<unsigned int> ids);

    std::vector<BigRadarData> GetBigRadarTargets() const;
    std::vector<SmallRadarData> GetSmallRadarTargets() const;

private:
    void AddNewTarget();
    bool IsTargetInSector(const Target& target) const;

private:
    const Flat::Parameters& Params;

    std::vector<Target> Targets;

    Timer SmallRadarTimer;
    Timer BigRadarTimer;

    const int BigRadarUpdatePeriod;
    const float NewTargetProbability;

    double SmallRadarPosition;
};


#endif // SIMULATOR_H
