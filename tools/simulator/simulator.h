#ifndef SIMULATOR_H
#define SIMULATOR_H

#include "radar_control/lib/data.h"
#include "proto/generated/params.pb.h"

#include <SFML/System/Vector2.hpp>


class Target {
public:
    Target(unsigned int id, float priority, sf::Vector2f pos, sf::Vector2f speed);

    void UpdatePosition(unsigned int ms);

    SmallRadarData GetSmallData() const;
    BigRadarData GetBigData() const;

private:
    const unsigned int Id;
    const float Priority;
    sf::Vector2f Position;
    sf::Vector2f Speed;
};


class Simulator {
public:
    Simulator(const Proto::Parameters& params);


private:
    const Proto::Parameters& Params;

    std::vector<Target> Targets;
};


#endif // SIMULATOR_H
