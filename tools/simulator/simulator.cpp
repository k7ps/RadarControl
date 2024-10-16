#include "simulator.h"

namespace {

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

    bool Target::IsInSector(double rad, double angView, double angPos) const {
        double ang1 = angPos - angView / 2;
        double ang2 = angPos + angView / 2;
        auto polarPos = ToPolarSystem(Position);
        return polarPos.x <= rad && ang1 <= polarPos.y && polarPos.y <= ang2;
    }

}

Simulator::Simulator(const Proto::Parameters& params)
    : Params(params)
    , SmallRadarPosition(1.57079)
{
    Timer.restart();
}

void Simulator::UpdateTargets() {
    int ms = Timer.getElapsedTime().asMilliseconds();
    Timer.restart();
    for (auto& target : Targets) {
        target.UpdatePosition(ms);
    }
}

void Simulator::SetRadarPosition(double angPos) {
    SmallRadarPosition = angPos;
}
