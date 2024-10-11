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
