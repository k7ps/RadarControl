#include "radar_controller.h"
#include "util/util.h"

#include <iostream>
#include <math.h>

using namespace RC;


namespace {
    BigRadarData GetTargetDataById(const std::vector<BigRadarData>& datas, unsigned id) {
        for (const auto& data : datas) {
            if (data.Id == id) {
                return data;
            }
        }
        return BigRadarData{
            {
                .Id = -1
            }
        };
    }

    SmallRadarData GetTargetDataById(const std::vector<SmallRadarData>& datas, unsigned id) {
        for (const auto& data : datas) {
            if (data.Id == id) {
                return data;
            }
        }
        return SmallRadarData{
            .Id = -1
        };
    }
}


RadarController::RadarController(const Flat::Parameters& params)
    : Params(params)
    , RadarAnglePos(M_PI_2)
{}

void RadarController::Process(const std::vector<BigRadarData>& bigDatas, const std::vector<SmallRadarData>& smallDatas) {
    if (!bigDatas.empty() && FollowedTargetIds.empty()) {
        FollowedTargetIds.push_back(bigDatas.front().Id);
    }
    if (!FollowedTargetIds.empty()) {
        int id = FollowedTargetIds.front();
        auto bigData = GetTargetDataById(bigDatas, id);
        if (bigData.Id == -1) return;

        RadarAngleTarget = bigData.Ang;

        auto smallData = GetTargetDataById(smallDatas, id);
        if (smallData.Id == -1) return;

        RadarAngleTarget = smallData.Ang;

        // static int frameCount = 0;
        // ++frameCount;
        // if (frameCount >= 5) {
        //     MeetingPointsAndTargetIds
        // }
    }
}

RadarController::Result RadarController::GetAngleAndMeetingPoints() {
    double ms = Timer.GetElapsedTimeAsMs();
    Timer.Restart();

    if (RadarAngleTarget != -1) {
        double maxDelta = Params.small_radar()->angle_speed() * ms / 1000;

        if (std::abs(RadarAnglePos - RadarAngleTarget) <= maxDelta) {
            RadarAnglePos = RadarAngleTarget;
        } else {
            if (RadarAnglePos < RadarAngleTarget) {
                RadarAnglePos += maxDelta;
            } else {
                RadarAnglePos -= maxDelta;
            }
        }
    }

    auto res = RadarController::Result{
        .Angle = RadarAnglePos,
        .MeetingPointsAndTargetIds = MeetingPointsAndTargetIds
    };
    MeetingPointsAndTargetIds.clear();
    return res;
}
