#include "defense/defense.h"
#include "proto/generated/params.pb.h"
#include "radar_control/radar_controller/radar_controller.h"
#include "simulator/simulator.h"
#include "util/proto.h"
#include "visualizer/visualizer.h"

#include <iomanip>
#include <iostream>


int main() {
    std::cout << std::fixed << std::setprecision(6);
    SetTraceLogCallback([](int, const char*, va_list) {
        std::cout << '\0';
    });
    SetConfigFlags(FLAG_MSAA_4X_HINT);


    auto params = ParseProtoFromFile<Proto::Parameters>("../config/params.pbtxt");
    PrepareParams(params);

    if (params.simulator().has_random_seed()) {
        srand(params.simulator().random_seed());
    } else {
        srand(time(NULL));
    }


    RadarController radarController(params);
    Simulator simulator(params, true);
    TargetScheduler targetScheduler(params);
    Defense defense(params);
    Visualizer visualizer(params);

    targetScheduler.SetScenario("../config/scenario_simple.pbtxt", params.general().play_speed());


    while (visualizer.IsWindowOpen()) {
        targetScheduler.LaunchTargets(simulator);

        const auto& smallRadarTargets = simulator.GetSmallRadarTargets();
        const auto& bigRadarTargets = simulator.GetBigRadarTargets();

        radarController.Process(bigRadarTargets, smallRadarTargets);

        auto res = radarController.GetAngleAndMeetingPoints();

        defense.LaunchRockets(res.MeetingPointsAndTargetIds);

        visualizer.DrawFrame(
            bigRadarTargets,
            smallRadarTargets,
            res.FollowedTargetIds,
            defense.GetRocketsPositions(),
            defense.GetMeetingPoints(),
            res.Angle
        );

        simulator.RemoveTargets(defense.GetDestroyedTargetsId());
        simulator.SetRadarPosition(res.Angle);
        simulator.UpdateTargets();
    }

    return 0;
}
