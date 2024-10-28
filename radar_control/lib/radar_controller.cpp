#include "radar_controller.h"
#include "util/util.h"


RadarController::RadarController(const Flat::Parameters& params)
    : Params(params)
{}

void RadarController::Process(const std::vector<BigRadarData>& data) {}

void RadarController::Process(const std::vector<SmallRadarData>& data) {}

Result RadarController::GetDeltaAngleAndTargets() {
    Result res {
        .AngleDelta = GetRandomFloat(-0.01, 0.01)
    };
    return res;
}
