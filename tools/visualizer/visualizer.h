#ifndef VISUALIZER_H
#define VISUALIZER_H

#include "radar_control/lib/data.h"
#include "flat/generated/params.h"
#include "util/points.h"

#include <raylib-cpp.hpp>


class Visualizer {
public:
    enum View {
        STRAIGHT,
        SIDE,
    };

    Visualizer(const Flat::Parameters& params);

    bool IsWindowOpen() const;

    void DrawFrame(
        const std::vector<BigRadarData>& bigDatas,
        const std::vector<SmallRadarData>& smallDatas,
        const std::vector<Vector3d>& rockets,
        const std::vector<Vector3d>& meetingPoints,
        double radarPosAngle
    );

private:
    raylib::Vector2 ToWindowCoords(const Vector3d&, View) const;

    void DrawTarget(const SmallRadarData&, View);
    void DrawTargets(const std::vector<BigRadarData>&, const std::vector<SmallRadarData>&, View);
    void DrawRockets(const std::vector<Vector3d>&, View);
    void DrawMeetingPoints(const std::vector<Vector3d>&, View);
    void DrawRadars(double, View);

private:
    const Flat::Parameters& Params;

    raylib::Vector2 WindowSize;
    raylib::Window Window;

    const raylib::Vector2 RadarPositionStraight;
    const raylib::Vector2 RadarPositionSide;
};


#endif // VISUALIZER_H
