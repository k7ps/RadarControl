#include "radar_control/lib/radar_controller.h"
#include "visualizer/visualizer.h"
#include "util/json.h"

#include <math.h>
#include <iostream>


int main() {
    auto radarsParams = ParseJsonFile("../params/radars.json");
    radarsParams["small_radar"]["view_angle"] = radarsParams["small_radar"]["view_angle"].asDouble() * M_PI / 180;
    auto simulatorParams = ParseJsonFile("../params/simulator.json");
    auto visualizerParams = ParseJsonFile("../params/visualizer.json");

    RadarController radarController;
    Visualizer visualizer(radarsParams, visualizerParams);

    double angle = M_PI_2;

    while (visualizer.IsWindowOpen()) {
        visualizer.ProcessEvents();

        auto res = radarController.GetDeltaAngleAndTargets();
        angle += res.AngleDelta;

        visualizer.DrawFrame({}, {}, angle);
    }

    return 0;
}
