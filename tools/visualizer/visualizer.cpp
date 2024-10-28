#include "visualizer.h"
#include "util/util.h"

#include <set>
#include <math.h>
#include <iostream>
#include <typeinfo>

namespace {

    // gradient between green and red
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

    // sf::ConvexShape GetSector(float rad, double viewAngle, double posAngle, sf::Vector2f pos = {0, 0}) {
    //     const double angleStep = 0.02;
    //     double startAngle = posAngle - viewAngle / 2;
    //     double endAngle = posAngle + viewAngle / 2;
    //     std::vector<sf::Vector2f> points;

    //     double angle = startAngle;
    //     for (double i = 1; angle < endAngle; i++) {
    //         points.emplace_back(pos + ToWindowXY(rad, angle));
    //         angle = startAngle + angleStep * i;
    //     }
    //     points.emplace_back(pos + ToWindowXY(rad, endAngle));
    //     points.emplace_back(pos);

    //     sf::ConvexShape res(points.size());
    //     for (int i = 0; i < points.size(); ++i) {
    //         res.setPoint(i, points[i]);
    //     }
    //     return res;
    // }

}


Visualizer::Visualizer(const Flat::Parameters& params)
    : Params(params)
    // , WindowSize(2 * Params.big_radar().radius() + 100, Params.big_radar().radius() + 100)
    // , WindowSettings(0, 0, 4)
    // , Window(sf::VideoMode(WindowSize.x, WindowSize.y), "RadarControl", sf::Style::Default, WindowSettings)
{
    // sf::View centerView = Window.getView();
    // centerView.setCenter(0, -WindowSize.y / 2);
    // Window.setView(centerView);
    // Window.setFramerateLimit(Params.small_radar().frequency());
}

bool Visualizer::IsWindowOpen() const {
    // return Window.isOpen();
    return false;
}

void Visualizer::ProcessEvents() {
    // sf::Event event;
    // while (Window.pollEvent(event)) {
    //     if (event.type == sf::Event::Closed) {
    //         Window.close();
    //     }
    // }
}

void Visualizer::DrawFrame(
    const std::vector<BigRadarData>& bigDatas,
    const std::vector<SmallRadarData>& smallDatas,
    double radarAngle
) {
    // Window.clear(sf::Color::White);
    // DrawTargets(bigDatas, smallDatas);
    // DrawRadars(radarAngle);
    // Window.display();
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

void Visualizer::DrawRadars(double radarPosAngle) {
    // const auto fillColor = sf::Color::Transparent;
    // const auto outlineColor = sf::Color::Black;
    // const auto centerRadius = 7;

    // sf::CircleShape bigRadar(Params.big_radar().radius(), 100);
    // bigRadar.setOrigin(Params.big_radar().radius(), Params.big_radar().radius());
    // bigRadar.setFillColor(fillColor);
    // bigRadar.setOutlineColor(outlineColor);
    // bigRadar.setOutlineThickness(Params.visualizer().radars_outline_thickness());

    // auto smallRadar = GetSector(Params.small_radar().radius(), Params.small_radar().view_angle(), radarPosAngle);
    // smallRadar.setFillColor(fillColor);
    // smallRadar.setOutlineColor(outlineColor);
    // smallRadar.setOutlineThickness(Params.visualizer().radars_outline_thickness());

    // sf::CircleShape center(centerRadius);
    // center.setOrigin(centerRadius, centerRadius);
    // center.setFillColor(sf::Color::Blue);

    // Window.draw(bigRadar);
    // Window.draw(smallRadar);
    // Window.draw(center);
}
