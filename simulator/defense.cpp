#include "defense.h"
#include "util/points.h"

#include <iostream>


DefRocket::DefRocket(Vector3d meetPoint, double speed, unsigned timeToLaunchMs)
    : Pos()
    , MeetPoint(meetPoint)
    , Speed(MeetPoint * speed / SqrtOfSumSquares(meetPoint)), TimeToLaunchMs(timeToLaunchMs)
{}

void DefRocket::UpdatePosition() {
    double ms = Timer.GetElapsedTimeAsMs();
    if (!IsLaunchedFlag || IsExplodedFlag) {
        if (ms < TimeToLaunchMs || IsExplodedFlag) {
            return;
        } else {
            ms -= TimeToLaunchMs;
            TestTimer.Restart();
            IsLaunchedFlag = true;
        }
    }
    Timer.Restart();

    Pos += Speed * ms;

    if (IsSignsEqual(Pos - MeetPoint, Speed)) {
        Pos = MeetPoint;
        IsExplodedFlag = true;
    }
}

Vector3d DefRocket::GetPosition() const {
    return Pos;
}

Vector3d DefRocket::GetMeetPoint() const {
    return MeetPoint;
}

bool DefRocket::IsLaunched() const {
    return IsLaunchedFlag;
}

bool DefRocket::IsExploded() const {
    return IsExplodedFlag;
}


Defense::Defense(const Proto::Parameters& params)
    : Params(params)
{}

void Defense::LaunchRockets(const std::vector<std::pair<Vector3d, int>>& meetPointsAndTargetIds) {
    for (const auto& [point, targetId] : meetPointsAndTargetIds) {
        Rockets.emplace_back(
            DefRocket(point, Params.defense().rocket_speed(), Params.defense().time_to_launch_rocket()),
            targetId
        );
    }
}

std::vector<int> Defense::GetDestroyedTargetsId() {
    std::vector<int> res;
    for (int i = 0; i < Rockets.size(); i++) {
        auto& rocket = Rockets[i].first;
        rocket.UpdatePosition();

        if (rocket.IsExploded()) {
            res.push_back(Rockets[i].second);
            Rockets.erase(Rockets.begin() + i);
            i--;
        }
    }
    return res;
}

std::vector<Vector3d> Defense::GetRocketsPositions() {
    std::vector<Vector3d> res;
    for (auto& [rocket, _] : Rockets) {
        if (rocket.IsLaunched() && !rocket.IsExploded()) {
            rocket.UpdatePosition();
            res.push_back(rocket.GetPosition());
        }
    }
    return res;
}

std::vector<Vector3d> Defense::GetMeetPoints() {
    std::vector<Vector3d> res;
    for (auto& [rocket, _] : Rockets) {
        if (!rocket.IsExploded()) {
            res.push_back(rocket.GetMeetPoint());
        }
    }
    return res;
}
