#include "visualizer.h"

#include <set>
#include <math.h>
#include <iostream>

#include <SFML/Graphics.hpp>


namespace {

    sf::Vector2f ToWindowXY(double rad, double fi) {
        return sf::Vector2f(rad * std::cos(fi), -rad * std::sin(fi));
    }

    void DrawTarget(const SmallRadarData& data, sf::RenderWindow& window) {
        sf::CircleShape target;
        target.setRadius(2);
        target.setPosition(ToWindowXY(data.Rad, data.Angle));
        target.setFillColor(sf::Color::Red);
        window.draw(target);
    }

    sf::ConvexShape GetSector(float rad, double viewAngle, double posAngle, sf::Vector2f pos = {0, 0}) {
        const double angleStep = 0.02;
        double startAngle = posAngle - viewAngle / 2;
        double endAngle = posAngle + viewAngle / 2;
        std::vector<sf::Vector2f> points;

        double angle = startAngle;
        for (double i = 1; angle < endAngle; i++) {
            points.emplace_back(pos + ToWindowXY(rad, angle));
            angle = startAngle + angleStep * i;
        }
        points.emplace_back(pos + ToWindowXY(rad, endAngle));
        points.emplace_back(pos);

        sf::ConvexShape res(points.size());
        for (int i = 0; i < points.size(); ++i) {
            res.setPoint(i, points[i]);
        }
        return res;
    }

}


Visualizer::Visualizer(double bigRad, double smallRad, double viewAng, int freq, int antialiasing, int outline)
    : WindowSize(2 * bigRad + 100, bigRad + 100)
    , WindowSettings(0, 0, antialiasing)
    , Window(sf::VideoMode(WindowSize.x, WindowSize.y), "RadarControl", sf::Style::Default, WindowSettings)
    , BigRadarRadius(bigRad)
    , SmallRadarRadius(smallRad)
    , SmallRadarViewAngle(viewAng)
    , RadarsOutlineThickness(outline)
{
    sf::View view = Window.getView();
    view.setCenter(0, -WindowSize.y / 2);
    Window.setView(view);
    Window.setFramerateLimit(freq);
}

Visualizer::Visualizer(const Json::Value& radarParams, const Json::Value& params)
    : Visualizer(
        radarParams["big_radar"]["radius"].asDouble(),
        radarParams["small_radar"]["radius"].asDouble(),
        radarParams["small_radar"]["view_angle"].asDouble(),
        radarParams["small_radar"]["frequency"].asUInt(),
        params["antialiasing_level"].asUInt(),
        params["radars_vision_outline_thickness"].asUInt()
    )
{}

bool Visualizer::IsWindowOpen() {
    return Window.isOpen();
}

void Visualizer::ProcessEvents() {
    sf::Event event;
    while (Window.pollEvent(event)) {
        if (event.type == sf::Event::Closed) {
            Window.close();
        }
    }
}

void Visualizer::DrawFrame(
    const std::vector<BigRadarData>& bigDatas,
    const std::vector<SmallRadarData>& smallDatas,
    double radarAngle
) {
    Window.clear(sf::Color::White);
    DrawTargets(bigDatas, smallDatas);
    DrawRadars(radarAngle);
    Window.display();
}

void Visualizer::DrawTargets(const std::vector<BigRadarData>& bigDatas, const std::vector<SmallRadarData>& smallDatas) {
    std::set<uint32_t> drawnTargets;
    for (const auto& data : smallDatas) {
        drawnTargets.insert(data.Id);
        DrawTarget(data, Window);
    }
    for (const auto& data : bigDatas) {
        if (!drawnTargets.count(data.Id)) {
            DrawTarget(data, Window);
        }
    }
}

void Visualizer::DrawRadars(double radarPosAngle) {
    const auto fillColor = sf::Color::White;
    const auto outlineColor = sf::Color::Black;
    const auto centerRadius = 5;

    sf::CircleShape bigRadar(BigRadarRadius, 100);
    bigRadar.setOrigin(BigRadarRadius, BigRadarRadius);
    bigRadar.setFillColor(fillColor);
    bigRadar.setOutlineColor(outlineColor);
    bigRadar.setOutlineThickness(RadarsOutlineThickness);

    auto smallRadar = GetSector(SmallRadarRadius, SmallRadarViewAngle, radarPosAngle);
    smallRadar.setFillColor(fillColor);
    smallRadar.setOutlineColor(outlineColor);
    smallRadar.setOutlineThickness(RadarsOutlineThickness);

    sf::CircleShape center(centerRadius);
    center.setOrigin(centerRadius, centerRadius);
    center.setFillColor(sf::Color::Blue);

    Window.draw(bigRadar);
    Window.draw(smallRadar);
    Window.draw(center);
}
