#include "radar_controller.h"

#include <random>

namespace {
    double GetRandomDouble(double min, double max) {
        return min + (double) rand() / RAND_MAX * (max - min);
    }
}

RadarController::RadarController() {
    srand(time(0));
}

void RadarController::Process(const std::vector<BigRadarData>& data) {}

void RadarController::Process(const std::vector<SmallRadarData>& data) {}

Result RadarController::GetDeltaAngleAndTargets() {
    Result res {
        .AngleDelta = GetRandomDouble(-0.1, 0.1)
    };
    return res;
}
