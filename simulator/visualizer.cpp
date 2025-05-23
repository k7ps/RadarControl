#include "visualizer.h"
#include "util/points.h"
#include "util/util.h"
#include "util/proto.h"

#include <raylib.h>
#include <Color.hpp>
#include <Vector2.hpp>

#include <set>
#include <string>
#include <vector>


namespace {
    raylib::Color GetTargetColor(float priority) {
        if (priority == -1) {
            return raylib::Color::Gray();
        }

        const float start = 0; // red
        const float end = 120; // green
        return raylib::Color::FromHSV(start + (1 - priority) * (end - start), 1.f, 1.f);
    }

    void DrawCircleDashedLines(
        raylib::Vector2 pos,
        float rad,
        raylib::Color col,
        int segmentsCnt = 6,
        float dashProportion = 0.7f
    ) {
        float segmentAng = 360.f / segmentsCnt;
        for (int i = 0; i < segmentsCnt; ++i) {
            float startAng = i * segmentAng;
            float endAng = startAng + segmentAng * dashProportion;
            DrawRingLines(pos, rad, rad, startAng, endAng, 10, col);
        }
    }

    void DrawDashedLine(
        raylib::Vector2 start,
        raylib::Vector2 end,
        raylib::Color col,
        float dashLength = 15,
        float dashProportion = 0.4
    ) {
        auto direction = (end - start).Normalize();
        auto length = (end - start).Length();
        auto period = dashLength / dashProportion;
        auto segmentCount = std::ceil(length / period);

        for (int i = 0; i < segmentCount; ++i) {
            auto dashStart = start + direction * period * i;
            auto dashEnd = dashStart + direction * dashLength;
            if (i * period + dashLength > length) {
                dashEnd = end;
            }
            DrawLineEx(dashStart, dashEnd, 2, col);
        }
    }

    auto DrawDashedRadius(raylib::Vector2 center, float radius, float ang, raylib::Color col) {
        auto end = center + raylib::Vector2(radius * std::cos(ang), - radius * std::sin(ang));
        DrawDashedLine(center, end, col);
    }

    void DrawCross(raylib::Vector2 center, float size, raylib::Color col, float thick = 2) {
        auto halfsize = size * 0.5;
        DrawLineEx(
            center + raylib::Vector2(-halfsize, +halfsize),
            center + raylib::Vector2(+halfsize, -halfsize),
            thick,
            col
        );
        DrawLineEx(
            center + raylib::Vector2(+halfsize, +halfsize),
            center + raylib::Vector2(-halfsize, -halfsize),
            thick,
            col
        );
    }
}


Visualizer::Visualizer(const Proto::Parameters& params)
    : Params(params)
    , WindowSize(
        2 * Params.big_radar().radius() + 100,
        Params.big_radar().radius() + 5 + Params.simulator().max_height() + 30
    )
    , Window(WindowSize.x, WindowSize.y + 5, "RadarControl")
    , RadarPositionStraight(WindowSize.x / 2, WindowSize.y - Params.simulator().max_height() - 30)
    , RadarPositionSide(WindowSize.x / 2, WindowSize.y)
{
    SetTargetFPS(Params.small_radar().frequency());
}

bool Visualizer::IsWindowOpen() const {
    return !WindowShouldClose();
}

void Visualizer::DrawFrame(
    const std::vector<BigRadarData>& bigDatas,
    const std::vector<SmallRadarData>& smallDatas,
    const std::map<int, double>& priorities,
    const std::vector<int>& followedTargetIds,
    const std::vector<Vector3d>& rockets,
    const std::vector<Vector3d>& entryPoints,
    const std::vector<Vector3d>& approximateMeetPoints,
    double radarPosAngle,
    double shipPosAngle
) {
    BeginDrawing();
    {
        Window.ClearBackground(raylib::Color::RayWhite());

        for (auto view : {View::STRAIGHT, View::SIDE}) {
            DrawDeadZones(shipPosAngle, view);
            DrawRadars(radarPosAngle, view);
            DrawTargets(bigDatas, smallDatas, priorities, followedTargetIds, view);
            DrawRockets(rockets, view);
            DrawEntryPoints(entryPoints, view);
            DrawApproximateMeetPoints(approximateMeetPoints, view);
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
        default:
            throw "Visualizer::ToWindowCoords() not implemented for view = " + std::to_string(view);
    }
}

void Visualizer::DrawTarget(const SmallRadarData& data, double priority, bool isFollowed, View view) {
    auto targetPos = ToWindowCoords(data.Pos, view);
    DrawCircleV(targetPos, Params.visualizer().target_radius(), GetTargetColor(priority));
    DrawCircleLinesV(targetPos, Params.visualizer().target_radius(), raylib::Color::Black());
    if (isFollowed) {
        DrawCircleDashedLines(targetPos, Params.visualizer().target_radius() + 5, raylib::Color::Black());
    }
}

void Visualizer::DrawTargets(
    const std::vector<BigRadarData>& bigDatas,
    const std::vector<SmallRadarData>& smallDatas,
    const std::map<int, double>& priorities,
    const std::vector<int>& followedTargetIds,
    View view
) {
    std::set<uint32_t> drawnTargets;
    std::set<int> followed(followedTargetIds.begin(), followedTargetIds.end());
    for (const auto& data : smallDatas) {
        drawnTargets.insert(data.Id);
        DrawTarget(data, priorities.at(data.Id), followed.count(data.Id), view);
    }
    for (const auto& data : bigDatas) {
        if (!drawnTargets.count(data.Id)) {
            DrawTarget(data, priorities.at(data.Id), followed.count(data.Id), view);
        }
    }
}

void Visualizer::DrawRadars(double radarPosAngle, View view) {
    switch (view) {
        case STRAIGHT: {
            DrawDashedRadius(
                RadarPositionStraight,
                Params.big_radar().radius(),
                Params.small_radar().responsible_sector_start(),
                raylib::Color::Gray()
            );
            DrawDashedRadius(
                RadarPositionStraight,
                Params.big_radar().radius(),
                Params.small_radar().responsible_sector_end(),
                raylib::Color::Gray()
            );
            DrawCircleSector(
                RadarPositionStraight,
                Params.big_radar().radius(),
                -Params.small_radar().responsible_sector_start() * 180 / M_PI,
                -Params.small_radar().responsible_sector_end() * 180 / M_PI,
                30,
                raylib::Color(0, 0, 0, 15)
            );
\
            float radarPos = RadToDeg(radarPosAngle);
            float halfview = RadToDeg(Params.small_radar().view_angle()) / 2;
            float start = std::max(0.f, radarPos - halfview);
            float end   = std::min(180.f, radarPos + halfview);

            DrawCircleSectorLines(
                RadarPositionStraight,
                Params.small_radar().radius(),
                -start,
                -end,
                30,
                raylib::Color::Black()
            );

            DrawCircleSectorLines(
                RadarPositionStraight,
                Params.big_radar().radius(),
                0,
                -180,
                50,
                raylib::Color::Black()
            );
            DrawCircleV(RadarPositionStraight, 7, raylib::Color::Blue());
            break;
        }
        case SIDE: {
            float width = 2 * Params.big_radar().radius();
            float height = Params.simulator().max_height();

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

void Visualizer::DrawDeadZones(double shipPosAngle, View view) {
    switch (view) {
        case STRAIGHT: {
            auto deadZones = ShiftedSegmentsFromProto(Params.ship().dead_zones(), shipPosAngle);

            for (const auto& seg : deadZones) {
                float start = std::max(0.f, (float) RadToDeg(seg.first));
                float end = std::min(180.f, (float) RadToDeg(seg.second));
                if (std::abs(end - start) > 0.1) {
                    DrawCircleSector(
                        RadarPositionStraight,
                        Params.small_radar().radius(),
                        -start,
                        -end,
                        30,
                        raylib::Color(0, 0, 0, 35)
                    );
                }
            }
            break;
        }
        case SIDE:
            break;
    }
}


void Visualizer::DrawRockets(const std::vector<Vector3d>& rockets, View view) {
    for (const auto& rocketPos : rockets) {
        DrawPoly(ToWindowCoords(rocketPos, view), 3, 7, 0, raylib::Color::Red());
    }
}

void Visualizer::DrawEntryPoints(const std::vector<Vector3d>& entryPoints, View view) {
    if (Params.visualizer().draw_entry_points()) {
        for (const auto& point : entryPoints) {
            DrawCircleLinesV(
                ToWindowCoords(point, view),
                Params.visualizer().target_radius(),
                raylib::Color::Gray()
            );
        }
    }
}

void Visualizer::DrawApproximateMeetPoints(const std::vector<Vector3d>& approximateMeetPoints, View view) {
    for (const auto& point : approximateMeetPoints) {
        DrawCross(
            ToWindowCoords(point, view),
            Params.visualizer().target_radius() * 1.5,
            raylib::Color::Red()
        );
    }
}
