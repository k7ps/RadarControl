#ifndef VISUALIZER_H
#define VISUALIZER_H

#include "radar_control/data.h"
#include "proto/generated/params.pb.h"
#include "util/points.h"

#include <raylib-cpp.hpp>


class Visualizer {
public:
    enum View {
        STRAIGHT,
        SIDE,
    };

    Visualizer(const Proto::Parameters& params);

    bool IsWindowOpen() const;

    void DrawFrame(
        const std::vector<BigRadarData>& bigDatas,
        const std::vector<SmallRadarData>& smallDatas,
        const std::map<int, double>& priorities,
        const std::vector<int>& followedTargetIds,
        const std::vector<Vector3d>& rockets,
        const std::vector<Vector3d>& entryPoints,
        const std::vector<Vector3d>& approximateMeetingPoints,
        double radarPosAngle
    );

private:
    raylib::Vector2 ToWindowCoords(const Vector3d& p, View view) const;
    void DrawPoints(const std::vector<Vector3d>& points, View view, raylib::Color color);

    void DrawTarget(const SmallRadarData& data, double priority, bool isFollowed, View view);
    void DrawTargets(
        const std::vector<BigRadarData>& bigDatas,
        const std::vector<SmallRadarData>& smallDatas,
        const std::map<int, double>& priorities,
        const std::vector<int>& followedTargetIds,
        View view
    );
    void DrawRockets(const std::vector<Vector3d>& rockets, View view);
    void DrawEntryPoints(const std::vector<Vector3d>& entryPoints, View view);
    void DrawApproximateMeetingPoints(const std::vector<Vector3d>& approximateMeetingPoints, View view);
    void DrawRadars(double radarPosAngle, View view);

private:
    const Proto::Parameters& Params;

    raylib::Vector2 WindowSize;
    raylib::Window Window;

    const raylib::Vector2 RadarPositionStraight;
    const raylib::Vector2 RadarPositionSide;
};


#endif // VISUALIZER_H
