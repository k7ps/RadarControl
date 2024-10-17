#include "radar_control/lib/radar_controller.h"
#include "simulator/simulator.h"
#include "util/proto.h"
#include "visualizer/visualizer.h"

#include <math.h>
#include <iostream>


int main() {
    auto params = ParseFromFile<Proto::Parameters>("../params/params.pb.txt");
    params.mutable_small_radar()->set_view_angle(params.small_radar().view_angle() * M_PI / 180);
    params.mutable_simulator()->set_max_deviation_angle(params.simulator().max_deviation_angle() * M_PI / 180);

    if (params.simulator().has_random_seed()) {
        srand(params.simulator().random_seed());
    } else {
        srand(time(NULL));
    }

    RadarController radarController(params);
    Simulator simulator(params);
    Visualizer visualizer(params);

    double angle = M_PI_2;

    while (visualizer.IsWindowOpen()) {
        visualizer.ProcessEvents();

        auto bigRadarTargets = simulator.GetBigRadarTargets();
        auto smallRadarTargets = simulator.GetSmallRadarTargets();

        radarController.Process(bigRadarTargets);
        radarController.Process(smallRadarTargets);

        auto res = radarController.GetDeltaAngleAndTargets();
        angle += res.AngleDelta;

        visualizer.DrawFrame(bigRadarTargets, smallRadarTargets, angle);
        simulator.UpdateTargets();
    }

    return 0;
}
