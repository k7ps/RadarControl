#ifndef VISUALIZER_H
#define VISUALIZER_H

#include "radar_control/lib/data.h"
#include "util/json.h"

#include <SFML/Graphics.hpp>


class Visualizer {
public:
    Visualizer(const Json::Value&, const Json::Value&);
    Visualizer(double bigRad, double smallRad, double viewAng, int freq, int antialiasing = 0, int outline = 1);

    bool IsWindowOpen() const;
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

    const int RadarsOutlineThickness;
};


#endif // VISUALIZER_H
