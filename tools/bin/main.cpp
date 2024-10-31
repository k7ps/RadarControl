#include "radar_control/lib/radar_controller.h"
#include "simulator/simulator.h"
#include "util/flat.h"
#include "visualizer/visualizer.h"

#include <math.h>
#include <iostream>


int main() {
    SetTraceLogCallback([](int, const char*, va_list) {
        std::cout << '\0';
    });
    SetConfigFlags(FLAG_MSAA_4X_HINT);

    const auto& params = *ParseParameters("../params/params.json", "../flat/params.fbs");

    if (params.simulator()->random_seed() != -1) {
        srand(params.simulator()->random_seed());
    } else {
        srand(time(NULL));
    }

    RadarController radarController(params);
    Simulator simulator(params);
    Visualizer visualizer(params);

    double angle = M_PI_2;

    while (visualizer.IsWindowOpen()) {
        auto smallRadarTargets = simulator.GetSmallRadarTargets();
        auto bigRadarTargets = simulator.GetBigRadarTargets();

        radarController.Process(smallRadarTargets);
        radarController.Process(bigRadarTargets);

        auto res = radarController.GetDeltaAngleAndTargets();
        angle += res.AngleDelta;

        visualizer.DrawFrame(bigRadarTargets, smallRadarTargets, angle);
        simulator.SetRadarPosition(angle);
        simulator.UpdateTargets();
    }

    return 0;
}
