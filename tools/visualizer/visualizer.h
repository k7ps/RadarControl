#ifndef VISUALIZER_H
#define VISUALIZER_H

#include "radar_control/lib/data.h"

#include <SFML/Graphics.hpp>


class Visualizer {
public:
    Visualizer();

    bool IsWindowOpen();
    void ProcessEvents();

    void DrawFrame(const std::vector<BigRadarData>&, const std::vector<SmallRadarData>&, double);

private:
    void DrawTargets(const std::vector<BigRadarData>&, const std::vector<SmallRadarData>&);
    void DrawRadars(double);

private:
    sf::ContextSettings WindowSettings;
    const sf::Vector2i WindowSize;
    sf::RenderWindow Window;

    const double BigRadarRadius;
    const double SmallRadarRadius;
    const double SmallRadarViewAngle;
    const sf::Vector2f RadarPosition;
};

#endif // VISUALIZER_H
