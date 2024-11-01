#ifndef VISUALIZER_H
#define VISUALIZER_H

#include "radar_control/lib/data.h"
#include "flat/generated/params.h"
#include "util/points.h"

#include <raylib-cpp.hpp>


class Visualizer {
public:
    Visualizer(const Flat::Parameters& params);

    bool IsWindowOpen() const;

    void DrawFrame(
        const std::vector<BigRadarData>& bigDatas,
        const std::vector<SmallRadarData>& smallDatas,
        const std::vector<Vector3d>& rockets,
        double radarPosAngle
    );

private:
    void DrawTargets(const std::vector<BigRadarData>&, const std::vector<SmallRadarData>&);
    void DrawTargetsSideView(const std::vector<BigRadarData>&, const std::vector<SmallRadarData>&);

    void DrawRockets(const std::vector<Vector3d>&);
    void DrawRocketsSideView(const std::vector<Vector3d>&);

    void DrawRadars(double);
    void DrawRadarsSideView();

private:
    const Flat::Parameters& Params;

    raylib::Vector2 WindowSize;
    raylib::Window Window;

    const raylib::Vector2 RadarPosition;
    const raylib::Vector2 RadarPositionSideView;
};


#endif // VISUALIZER_H
