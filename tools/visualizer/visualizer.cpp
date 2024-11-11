#include "visualizer.h"
#include "util/util.h"

#include <set>
#include <iostream>


namespace {

    raylib::Color GetTargetColor(float priority) {
        const float start = 0; // red
        const float end = 120; // green
        return raylib::Color::FromHSV(start + priority * (end - start), 1.f, 1.f);
    }

    void DrawTarget(const SmallRadarData& data, raylib::Vector2 radarPos, int targetRad) {
        auto targetPos = PolarToCartesian(data.Rad, data.Ang);
        auto targetPosV = raylib::Vector2(targetPos.X, -targetPos.Y);
        DrawCircleV(radarPos + targetPosV, targetRad, GetTargetColor(data.Priority));
        DrawCircleLinesV(radarPos + targetPosV, targetRad, raylib::Color::Black());
    }

    void DrawTargetSideView(const SmallRadarData& data, raylib::Vector2 radarPos, int targetRad) {
        auto targetPosV = radarPos + raylib::Vector2(data.Rad * std::cos(data.Ang), -data.H);
        DrawCircleV(targetPosV, targetRad, GetTargetColor(data.Priority));
        DrawCircleLinesV(targetPosV, targetRad, raylib::Color::Black());
    }

}


Visualizer::Visualizer(const Flat::Parameters& params)
    : Params(params)
    , WindowSize(
        2 * Params.big_radar()->radius() + 100,
        Params.big_radar()->radius() + 5 + Params.simulator()->max_height() + 30
    )
    , Window(WindowSize.x, WindowSize.y + 5, "RadarControl")
    , RadarPosition(WindowSize.x / 2, WindowSize.y - Params.simulator()->max_height() - 30)
    , RadarPositionSideView(WindowSize.x / 2, WindowSize.y)
{
    SetTargetFPS(Params.small_radar()->frequency());
}

bool Visualizer::IsWindowOpen() const {
    return !WindowShouldClose();
}

void Visualizer::DrawFrame(
    const std::vector<BigRadarData>& bigDatas,
    const std::vector<SmallRadarData>& smallDatas,
    const std::vector<Vector3d>& rockets,
    double radarPosAngle
) {
    // std::cout << radarPosAngle << '\n';
    BeginDrawing();
    {
        Window.ClearBackground(raylib::Color::RayWhite());

        DrawTargets(bigDatas, smallDatas);
        DrawRockets(rockets);
        DrawRadars(radarPosAngle);

        DrawTargetsSideView(bigDatas, smallDatas);
        DrawRocketsSideView(rockets);
        DrawRadarsSideView();
    }
    EndDrawing();
}

void Visualizer::DrawTargets(const std::vector<BigRadarData>& bigDatas, const std::vector<SmallRadarData>& smallDatas) {
    std::set<uint32_t> drawnTargets;
    for (const auto& data : smallDatas) {
        drawnTargets.insert(data.Id);
        DrawTarget(data, RadarPosition, Params.visualizer()->target_radius());
    }
    for (const auto& data : bigDatas) {
        if (!drawnTargets.count(data.Id)) {
            DrawTarget(data, RadarPosition, Params.visualizer()->target_radius());
        }
    }
}

void Visualizer::DrawTargetsSideView(const std::vector<BigRadarData>& bigDatas, const std::vector<SmallRadarData>& smallDatas) {
    std::set<uint32_t> drawnTargets;
    for (const auto& data : smallDatas) {
        drawnTargets.insert(data.Id);
        DrawTargetSideView(data, RadarPositionSideView, Params.visualizer()->target_radius());
    }
    for (const auto& data : bigDatas) {
        if (!drawnTargets.count(data.Id)) {
            DrawTargetSideView(data, RadarPositionSideView, Params.visualizer()->target_radius());
        }
    }
}

void Visualizer::DrawRadars(double radarPosAngle) {
    float radarPosAngleDeg = RadToDeg(radarPosAngle);
    // std::cout << radarPosAngle << ' ' << radarPosAngleDeg << '\n';
    float radarViewAngleDeg = RadToDeg(Params.small_radar()->view_angle());
    float radarStartAngle = radarPosAngleDeg - radarViewAngleDeg / 2;
    float radarEndAngle   = radarPosAngleDeg + radarViewAngleDeg / 2;

    DrawCircleSectorLines(
        RadarPosition,
        Params.small_radar()->radius(),
        -radarStartAngle,
        -radarEndAngle,
        30,
        raylib::Color::Black()
    );
    DrawCircleSectorLines(RadarPosition, Params.big_radar()->radius(), 0, -180, 50, raylib::Color::Black());
    DrawCircleV(RadarPosition, 7, raylib::Color::Blue());
}

void Visualizer::DrawRadarsSideView() {
    float width = 2 * Params.big_radar()->radius();
    float height = Params.simulator()->max_height();

    DrawRectangleLines(RadarPositionSideView.x - width/2, RadarPositionSideView.y - height, width, height, raylib::Color::Black());
    DrawCircleV(RadarPositionSideView, 7, raylib::Color::Blue());
}

void Visualizer::DrawRockets(const std::vector<Vector3d>& rockets) {
    for (const auto& rocketPos : rockets) {
        auto targetPosV = RadarPosition + raylib::Vector2(rocketPos.X, - rocketPos.Y);
        DrawPoly(targetPosV, 3, 7, 0, raylib::Color::Red());
    }
}

void Visualizer::DrawRocketsSideView(const std::vector<Vector3d>& rockets) {
    for (const auto& rocketPos : rockets) {
        auto cylindPos = CartesianToCylindrical(rocketPos);
        auto targetPosV = RadarPositionSideView + raylib::Vector2(cylindPos.X * std::cos(cylindPos.Y), -cylindPos.Z);
        DrawPoly(targetPosV, 3, 7, 0, raylib::Color::Red());
    }
}
