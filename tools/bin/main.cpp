#include "radar_control/lib/radar_controller.h"
#include "visualizer/visualizer.h"

#include <math.h>


int main() {
    RadarController radarController;
    Visualizer visualizer;

    double angle = M_PI_2;

    while (visualizer.IsWindowOpen()) {
        visualizer.ProcessEvents();

        auto res = radarController.GetDeltaAngleAndTargets();
        angle += res.AngleDelta;

        visualizer.DrawFrame({}, {}, angle);
    }

    return 0;
}
