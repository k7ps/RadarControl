#include "simulator.h"


Target::Target(unsigned int id, float priority, sf::Vector2f pos, sf::Vector2f speed)
    : Id(id), Priority(priority), Position(pos), Speed(speed)
{}

void Target::UpdatePosition(unsigned int ms) {
    Position += Speed * float(ms);
}

SmallRadarData Target::GetSmallData() const {
    SmallRadarData res{
        .Id = Id,
        .Priority = Priority,
        .X = Position.x,
        .Y = Position.y
    };
    return res;
}

BigRadarData Target::GetBigData() const {
    BigRadarData res {
        GetSmallData(),
        .SpeedX = Speed.x,
        .SpeedY = Speed.y
    };
    return res;
}


// Simulator::Simulator(
//     double bigRad, double smallRad, double viewAng, double bigErr,
//     double smallErr, unsigned int targetPerMin, double speedMin, double speedMax
// )
//     :
// {

// }

// Simulator::Simulator(const Json::Value& radarParams, const Json::Value& params)
//     : Simulator(
//         radarParams["big_radar"]["radius"].asDouble(),
//         radarParams["small_radar"]["radius"].asDouble(),
//         radarParams["small_radar"]["view_angle"].asDouble(),
//         radarParams["big_radar"]["error"].asDouble(),
//         radarParams["small_radar"]["error"].asDouble(),
//         params["targets_per_minute"].asUInt(),
//         params["speed_min"].asDouble(),
//         params["speed_max"].asDouble(),
//     )
// {}
