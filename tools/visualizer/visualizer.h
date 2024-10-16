#ifndef VISUALIZER_H
#define VISUALIZER_H

#include "radar_control/lib/data.h"
#include "proto/generated/params.pb.h"

#include <SFML/Graphics.hpp>


class Visualizer {
public:
    Visualizer(const Proto::Parameters& params);

    bool IsWindowOpen() const;
    void ProcessEvents();

    void DrawFrame(const std::vector<BigRadarData>&, const std::vector<SmallRadarData>&, double);

private:
    void DrawTargets(const std::vector<BigRadarData>&, const std::vector<SmallRadarData>&);
    void DrawRadars(double);

private:
    const Proto::Parameters& Params;

    sf::ContextSettings WindowSettings;
    const sf::Vector2i WindowSize;
    sf::RenderWindow Window;

    const sf::Vector2f RadarPosition;
};


#endif // VISUALIZER_H
