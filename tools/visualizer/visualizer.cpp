#include "visualizer.h"
#include "util/util.h"

#include <set>
#include <math.h>
#include <iostream>
#include <typeinfo>

namespace {

    // // gradient between green and red
    // sf::Color GetTargetColor(float priority) {
    //     sf::Color color(0, 255, 0);
    //     int shift = priority * 510;
    //     color.r += std::min(shift, 255);
    //     if (shift > 255) {
    //         color.g -= shift - 255;
    //     }
    //     return color;
    // }

    // void DrawTarget(const SmallRadarData& data, float rad, sf::RenderWindow& window) {
    //     sf::CircleShape target;
    //     target.setRadius(rad);
    //     target.setPosition(data.X, data.Y);
    //     target.setFillColor(GetTargetColor(data.Priority));
    //     window.draw(target);
    // }

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
    float radarPosAngle
) {
    BeginDrawing();
    {
        Window.ClearBackground(raylib::Color::RayWhite());

        DrawRadars(radarPosAngle);
    }
    EndDrawing();
}

void Visualizer::DrawTargets(const std::vector<BigRadarData>& bigDatas, const std::vector<SmallRadarData>& smallDatas) {
    // std::set<uint32_t> drawnTargets;
    // for (const auto& data : smallDatas) {
    //     drawnTargets.insert(data.Id);
    //     DrawTarget(data, Params.visualizer().target_radius(), Window);
    // }
    // for (const auto& data : bigDatas) {
    //     if (!drawnTargets.count(data.Id)) {
    //         DrawTarget(data, Params.visualizer().target_radius(), Window);
    //     }
    // }
}

void Visualizer::DrawRadars(float radarPosAngle) {
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
