#ifndef SIMULATOR_H
#define SIMULATOR_H

#include "radar_control/lib/data.h"
#include "proto/generated/params.pb.h"
#include "util/util.h"

#include <SFML/System/Vector2.hpp>
#include <SFML/System/Clock.hpp>


class Target {
public:
    Target(unsigned int id, float priority, sf::Vector2f pos, sf::Vector2f speed);

    void UpdatePosition(unsigned int ms);

    SmallRadarData GetSmallData() const;
    BigRadarData GetBigData() const;
    unsigned int GetId() const;

    bool IsInSector(double rad, double angView, double angPos) const;
    bool IsOutOfView(double rad) const;

private:
    unsigned int Id;
    float Priority;
    sf::Vector2f Position;
    sf::Vector2f Speed;
};


class Simulator {
public:
    Simulator(const Proto::Parameters& params);

    void UpdateTargets();
    void SetRadarPosition(double angPos);

    void RemoveTargets(std::vector<unsigned int> ids);

    std::vector<BigRadarData> GetBigRadarTargets() const;
    std::vector<SmallRadarData> GetSmallRadarTargets() const;

private:
    void AddNewTarget();
    bool IsTargetInSector(const Target& target) const;

private:
    const Proto::Parameters& Params;

    std::vector<Target> Targets;

    sf::Clock SmallRadarTimer;
    sf::Clock BigRadarTimer;

    const int BigRadarUpdatePeriod;
    const float NewTargetProbability;

    double SmallRadarPosition;
};


#endif // SIMULATOR_H
