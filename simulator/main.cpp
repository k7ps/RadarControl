#include "defense.h"
#include "proto/generated/params.pb.h"
#include "radar_control/radar_controller.h"
#include "simulator.h"
#include "util/proto.h"
#include "util/util.h"
#include "visualizer.h"

#include <argparse/argparse.hpp>

#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>


int main(int argc, char* argv[]) {
    std::cout << std::fixed << std::setprecision(9);
    SetTraceLogCallback([](int, const char*, va_list) {
        std::cout << '\0';
    });
    SetConfigFlags(FLAG_MSAA_4X_HINT);

    std::string scenariod_dir =
        getenv("RADARCONTROL_SCENARIOS_DIR") ? getenv("RADARCONTROL_SCENARIOS_DIR") : "../scenarios";
    std::string scenario_extension = ".pbtxt";

    std::vector<std::string> available_scenarios;
    for (const auto& file : std::filesystem::recursive_directory_iterator(scenariod_dir)) {
        if (file.is_regular_file() && file.path().extension() == scenario_extension) {
            available_scenarios.emplace_back(file.path().stem());
        }
    }
    std::sort(available_scenarios.begin(), available_scenarios.end());


    argparse::ArgumentParser program("Tools");
    program.add_argument("-s", "--scenario")
           .help("name of scenario file, if not specified random scenario will be used.\nAvailable scenarios: "
           + VectorToString(available_scenarios) + ".")
           .default_value<std::string>("");
    program.parse_args(argc, argv);

    if (program["--help"] == true) {
        std::cout << program.help().str();
        return 0;
    }
    auto scenario_name = program.get<std::string>("--scenario");


    std::string config_dir = getenv("RADARCONTROL_CONFIG_DIR") ? getenv("RADARCONTROL_CONFIG_DIR") : "../config";
    std::string config_path = config_dir + "/default.pbtxt";

    auto params = ParseProtoFromFile<Proto::Parameters>(config_path);
    PrepareParams(params);

    if (params.simulator().has_random_seed()) {
        srand(params.simulator().random_seed());
    } else {
        srand(time(NULL));
    }

    TargetScheduler targetScheduler(params);
    if (!scenario_name.empty()) {
        targetScheduler.SetScenario(scenariod_dir + "/" + scenario_name + scenario_extension);
    }

    RadarController radarController(params, targetScheduler.GetRadarStartAngle());
    Simulator simulator(params, targetScheduler.GetRadarStartAngle(), !scenario_name.empty());
    Defense defense(params);
    Visualizer visualizer(params);


    while (visualizer.IsWindowOpen()) {
        targetScheduler.LaunchTargets(simulator);

        const auto& bigRadarTargets = simulator.GetBigRadarTargets();
        const auto& smallRadarTargets = simulator.GetSmallRadarTargets();

        radarController.Process(bigRadarTargets, smallRadarTargets);

        auto res = radarController.GetAngleAndMeetPoints();

        defense.LaunchRockets(res.MeetPointsAndTargetIds);

        visualizer.DrawFrame(
            bigRadarTargets,
            smallRadarTargets,
            radarController.GetPriorities(),
            res.FollowedTargetIds,
            defense.GetRocketsPositions(),
            radarController.GetEntryPoints(),
            radarController.GetApproximateMeetPoints(),
            res.Angle
        );

        simulator.RemoveTargets(defense.GetDestroyedTargetsId());
        simulator.SetRadarPosition(res.Angle);
        simulator.UpdateTargets();
    }

    return 0;
}
