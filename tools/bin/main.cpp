#include "radar_control/lib/radar_controller.h"
#include "simulator/simulator.h"
#include "util/proto.h"
#include "visualizer/visualizer.h"

#include <math.h>
#include <iostream>


int main() {
    auto params = ParseFromFile<Proto::Parameters>("../params/params.pb.txt");
    params.mutable_small_radar()->set_view_angle(params.small_radar().view_angle() * M_PI / 180);

    RadarController radarController(params);
    Visualizer visualizer(params);

    double angle = M_PI_2;

    while (visualizer.IsWindowOpen()) {
        visualizer.ProcessEvents();

        auto res = radarController.GetDeltaAngleAndTargets();
        angle += res.AngleDelta;

        visualizer.DrawFrame({}, {}, angle);
    }

    return 0;
}
