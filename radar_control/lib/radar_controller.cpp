#include "radar_controller.h"
#include "util/util.h"


RadarController::RadarController(const Proto::Parameters& params)
    : Params(params)
{}

void RadarController::Process(const std::vector<BigRadarData>& data) {}

void RadarController::Process(const std::vector<SmallRadarData>& data) {}

Result RadarController::GetDeltaAngleAndTargets() {
    Result res {
        .AngleDelta = GetRandomFloat(-0.1, 0.1)
    };
    return res;
}
