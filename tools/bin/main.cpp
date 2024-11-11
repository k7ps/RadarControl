#include "defense/defense.h"
#include "radar_control/lib/radar_controller.h"
#include "simulator/simulator.h"
#include "util/flat.h"
#include "visualizer/visualizer.h"

#include <iostream>


int main() {
    SetTraceLogCallback([](int, const char*, va_list) {
        std::cout << '\0';
    });
    SetConfigFlags(FLAG_MSAA_4X_HINT);

    flatbuffers::Parser parser;
    const auto& params = *ParseParameters(parser, "../params/params.json", "../flat/params.fbs");

    if (params.simulator()->random_seed() != -1) {
        srand(params.simulator()->random_seed());
    } else {
        srand(time(NULL));
    }

    RadarController radarController(params);
    Simulator simulator(params);
    Defense defense(params);
    Visualizer visualizer(params);

    while (visualizer.IsWindowOpen()) {
        const auto& smallRadarTargets = simulator.GetSmallRadarTargets();
        const auto& bigRadarTargets = simulator.GetBigRadarTargets();
        const auto& updatedTargets = simulator.GetOnlyUpdatedTargets();

        radarController.Process(updatedTargets, smallRadarTargets);

        auto res = radarController.GetAngleAndMeetingPoints();

        defense.LaunchRockets(res.MeetingPointsAndTargetIds);

        visualizer.DrawFrame(bigRadarTargets, smallRadarTargets, defense.GetRocketsPositions(), res.Angle);

        simulator.RemoveTargets(defense.GetDestroyedTargetsId());
        simulator.SetRadarPosition(res.Angle);
        simulator.UpdateTargets();
    }

    return 0;
}
