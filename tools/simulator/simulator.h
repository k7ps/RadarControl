#ifndef SIMULATOR_H
#define SIMULATOR_H

#include "radar_control/lib/data.h"
#include "util/json.h"

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

// class Simulator {
// public:
//     Simulator(const Json::Value&, const Json::Value&);
//     Simulator(
//         double bigRad, double smallRad, double viewAng, double bigErr,
//         double smallErr, unsigned int targetPerMin, double speedMin, double speedMax
//     );

// private:
//     const double BigRadarRadius;
//     const double SmallRadarRadius;
//     const double SmallRadarViewAngle;
//     const double BigRadarError;
//     const double

//     const unsigned int TargetsPerMinute;
//     const double speedMin;
//     const double speedMax;

//     std::vector<Target> Targets;
// };


#endif // SIMULATOR_H
