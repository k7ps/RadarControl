#include "radar_controller.h"

#include <random>


RadarController::RadarController() {
    srand(time(0));
}

void RadarController::Process(BigRadarData data) {}

void RadarController::Process(SmallRadarData data) {}

Result RadarController::GetDeltaAngleAndTargets() {
    Result res {
        .AngleDelta = 2 * (double) rand() / RAND_MAX - 1
    };
    return res;
}
