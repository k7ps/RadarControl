#ifndef VISUALIZER_H
#define VISUALIZER_H

#include "radar_control/lib/data.h"
#include "flat/generated/params.h"

#include "raylib-cpp.hpp"


class Visualizer {
public:
    Visualizer(const Flat::Parameters& params);

    bool IsWindowOpen() const;

    void DrawFrame(const std::vector<BigRadarData>&, const std::vector<SmallRadarData>&, double);

private:
    void DrawTargets(const std::vector<BigRadarData>&, const std::vector<SmallRadarData>&);
    void DrawRadars(double);

private:
    const Flat::Parameters& Params;

    raylib::Vector2 WindowSize;
    raylib::Window Window;

    const raylib::Vector2 RadarPosition;
};


#endif // VISUALIZER_H
