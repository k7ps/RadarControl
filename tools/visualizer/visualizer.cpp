#include "visualizer.h"
#include "util/util.h"

#include <iostream>
#include <set>


namespace {
    raylib::Color GetTargetColor(float priority) {
        const float start = 0; // red
        const float end = 120; // green
        return raylib::Color::FromHSV(start + priority * (end - start), 1.f, 1.f);
    }
}


Visualizer::Visualizer(const Flat::Parameters& params)
    : Params(params)
    , WindowSize(
        2 * Params.big_radar()->radius() + 100,
        Params.big_radar()->radius() + 5 + Params.simulator()->max_height() + 30
    )
    , Window(WindowSize.x, WindowSize.y + 5, "RadarControl")
    , RadarPositionStraight(WindowSize.x / 2, WindowSize.y - Params.simulator()->max_height() - 30)
    , RadarPositionSide(WindowSize.x / 2, WindowSize.y)
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
    const std::vector<Vector3d>& meetingPoints,
    double radarPosAngle
) {
    BeginDrawing();
    {
        Window.ClearBackground(raylib::Color::RayWhite());

        for (auto view : {View::STRAIGHT, View::SIDE}) {
            DrawTargets(bigDatas, smallDatas, view);
            DrawRockets(rockets, view);
            DrawMeetingPoints(meetingPoints, view);
            DrawRadars(radarPosAngle, view);
        }
    }
    EndDrawing();
}

raylib::Vector2 Visualizer::ToWindowCoords(const Vector3d& pos, View view) const {
    switch (view) {
        case STRAIGHT:
            return RadarPositionStraight + raylib::Vector2(pos.X, - pos.Y);
        case SIDE: {
            auto cylindPos = CartesianToCylindrical(pos);
            return RadarPositionSide + raylib::Vector2(cylindPos.X * std::cos(cylindPos.Y), -cylindPos.Z);
        }
    }
}

void Visualizer::DrawTarget(const SmallRadarData& data, View view) {
    auto targetPos = ToWindowCoords(CylindricalToCartesian(data.Rad, data.Ang, data.H), view);
    DrawCircleV(targetPos, Params.visualizer()->target_radius(), GetTargetColor(data.Priority));
    DrawCircleLinesV(targetPos, Params.visualizer()->target_radius(), raylib::Color::Black());
}

void Visualizer::DrawTargets(
    const std::vector<BigRadarData>& bigDatas,
    const std::vector<SmallRadarData>& smallDatas,
    View view
) {
    std::set<uint32_t> drawnTargets;
    for (const auto& data : smallDatas) {
        drawnTargets.insert(data.Id);
        DrawTarget(data, view);
    }
    for (const auto& data : bigDatas) {
        if (!drawnTargets.count(data.Id)) {
            DrawTarget(data, view);
        }
    }
}

void Visualizer::DrawRadars(double radarPosAngle, View view) {
    switch (view) {
        case STRAIGHT: {
            float radarPosAngleDeg = RadToDeg(radarPosAngle);
            float radarViewAngleDeg = RadToDeg(Params.small_radar()->view_angle());
            float radarStartAngle = std::max(0.f, radarPosAngleDeg - radarViewAngleDeg / 2);
            float radarEndAngle   = std::min(180.f, radarPosAngleDeg + radarViewAngleDeg / 2);

            DrawCircleSectorLines(
                RadarPositionStraight,
                Params.small_radar()->radius(),
                -radarStartAngle,
                -radarEndAngle,
                30,
                raylib::Color::Black()
            );
            DrawCircleSectorLines(
                RadarPositionStraight,
                Params.big_radar()->radius(),
                0,
                -180,
                50,
                raylib::Color::Black()
            );
            DrawCircleV(RadarPositionStraight, 7, raylib::Color::Blue());
            break;
        }
        case SIDE: {
            float width = 2 * Params.big_radar()->radius();
            float height = Params.simulator()->max_height();

            DrawRectangleLines(
                RadarPositionSide.x - width/2,
                RadarPositionSide.y - height,
                width,
                height,
                raylib::Color::Black()
            );
            DrawCircleV(RadarPositionSide, 7, raylib::Color::Blue());
            break;
        }
    }
}

void Visualizer::DrawRockets(const std::vector<Vector3d>& rockets, View view) {
    for (const auto& rocketPos : rockets) {
        DrawPoly(ToWindowCoords(rocketPos, view), 3, 7, 0, raylib::Color::Red());
    }
}

void Visualizer::DrawMeetingPoints(const std::vector<Vector3d>& meetingPoints, View view) {
    for (const auto& meetingPoint : meetingPoints) {
        DrawCircleLinesV(
            ToWindowCoords(meetingPoint, view),
            Params.visualizer()->target_radius(),
            raylib::Color::Gray()
        );
    }
}
