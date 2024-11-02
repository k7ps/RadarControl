#include "radar_controller.h"
#include "util/util.h"

#include <iostream>


RadarController::RadarController(const Flat::Parameters& params)
    : Params(params)
    , RadarAnglePos(1.57079)
{}

void RadarController::Process(const std::vector<BigRadarData>& datas) {
    if (datas.size() > 0 && !isTested) {
        test = CylindricalToCartesian(datas[0].Rad, datas[0].Ang, datas[0].H);
    }
}

void RadarController::Process(const std::vector<SmallRadarData>& datas) {}

RadarController::Result RadarController::GetAngleAndMeetingPoints() {
    RadarAnglePos += GetRandomDouble(-0.01, 0.01);
    RadarController::Result res {
        .Angle = RadarAnglePos
    };
    if (test.X != -1 && !isTested) {
        res.MeetingPointsAndTargetIds = {{test, 0}};
        isTested = true;
    }
    return res;
}
