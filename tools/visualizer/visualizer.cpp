#include "visualizer.h"
#include "util/util.h"

#include <set>
#include <math.h>
#include <iostream>
#include <typeinfo>

namespace {

    void DrawTarget(const SmallRadarData& data, sf::RenderWindow& window) {
        sf::CircleShape target;
        target.setRadius(2);
        target.setPosition(data.X, data.Y);
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


Visualizer::Visualizer(const Proto::Parameters& params)
    : Params(params)
    , WindowSize(2 * Params.big_radar().radius() + 100, Params.big_radar().radius() + 100)
    , WindowSettings(0, 0, 4)
    , Window(sf::VideoMode(WindowSize.x, WindowSize.y), "RadarControl", sf::Style::Default, WindowSettings)
{
    std::cout << WindowSize.x << ' ' <<  WindowSize.y << std::endl;
    sf::View view = Window.getView();
    view.setCenter(0, -WindowSize.y / 2);
    Window.setView(view);
    Window.setFramerateLimit(Params.small_radar().frequency());
}

bool Visualizer::IsWindowOpen() const {
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

    sf::CircleShape bigRadar(Params.big_radar().radius(), 100);
    bigRadar.setOrigin(Params.big_radar().radius(), Params.big_radar().radius());
    bigRadar.setFillColor(fillColor);
    bigRadar.setOutlineColor(outlineColor);
    bigRadar.setOutlineThickness(Params.visualizer().radars_outline_thickness());

    auto smallRadar = GetSector(Params.small_radar().radius(), Params.small_radar().view_angle(), radarPosAngle);
    smallRadar.setFillColor(fillColor);
    smallRadar.setOutlineColor(outlineColor);
    smallRadar.setOutlineThickness(Params.visualizer().radars_outline_thickness());

    sf::CircleShape center(centerRadius);
    center.setOrigin(centerRadius, centerRadius);
    center.setFillColor(sf::Color::Blue);

    Window.draw(bigRadar);
    Window.draw(smallRadar);
    Window.draw(center);
}
