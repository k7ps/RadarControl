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

}


Visualizer::Visualizer(const Flat::Parameters& params)
    : Params(params)
    , WindowSize(2 * (float) Params.big_radar()->radius() + 100, (float) Params.big_radar()->radius() + 100)
    , Window(WindowSize.x, WindowSize.y, "RadarControl")
    , RadarPosition(WindowSize.x / 2, WindowSize.y)
{
    SetTargetFPS(Params.small_radar()->frequency());
}

bool Visualizer::IsWindowOpen() const {
    return !WindowShouldClose();
}

void Visualizer::DrawFrame(
    const std::vector<BigRadarData>& bigDatas,
    const std::vector<SmallRadarData>& smallDatas,
    double radarPosAngle
) {
    BeginDrawing();
    {
        Window.ClearBackground(raylib::Color::RayWhite());

        DrawTargets(bigDatas, smallDatas);
        DrawRadars(radarPosAngle);
    }
    EndDrawing();
}

void Visualizer::DrawTargets(const std::vector<BigRadarData>& bigDatas, const std::vector<SmallRadarData>& smallDatas) {
    std::set<uint32_t> drawnTargets;
    for (const auto& data : smallDatas) {
        drawnTargets.insert(data.Id);
        DrawCircle(RadarPosition.x + data.X, RadarPosition.y - data.Y, Params.visualizer()->target_radius(), GetTargetColor(data.Priority));
    }
    for (const auto& data : bigDatas) {
        if (!drawnTargets.count(data.Id)) {
            DrawCircle(RadarPosition.x + data.X, RadarPosition.y - data.Y, Params.visualizer()->target_radius(), GetTargetColor(data.Priority));
        }
    }
}

void Visualizer::DrawRadars(double radarPosAngle) {
    float radarPosAngleDeg = RadToDeg(radarPosAngle);
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
