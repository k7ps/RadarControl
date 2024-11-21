#include "defense.h"

#include <iostream>


DefRocket::DefRocket(Vector3d meetingPoint, double speed, unsigned timeToLaunchMs)
    : Pos()
    , MeetingPoint(meetingPoint)
    , Speed(MeetingPoint * speed / GetSqrtOfSquareSum(meetingPoint)), TimeToLaunchMs(timeToLaunchMs)
{
    std::cout << "Rocket Speed: " << Speed.DebugString() << '\n';
    std::cout << "Rocket to: " << MeetingPoint.DebugString() << '\n';
}

void DefRocket::UpdatePosition() {
    double ms = Timer.GetElapsedTimeAsMs();
    if (!IsLaunchedFlag || IsExplodedFlag) {
        if (ms < TimeToLaunchMs || IsExplodedFlag) {
            return;
        } else {
            ms -= TimeToLaunchMs;
            IsLaunchedFlag = true;
        }
    }
    Timer.Restart();

    Pos += Speed * ms;

    // if (Pos.Z > MeetingPoint.Z) {
    if (Pos.Z >= MeetingPoint.Z || Pos.Y >= MeetingPoint.Y || std::abs(Pos.X) >= std::abs(MeetingPoint.X)) {
        std::cout << "Rocket flown to " << Pos.DebugString() << '\n';
        Pos = MeetingPoint;
        IsExplodedFlag = true;
    }
}

Vector3d DefRocket::GetPosition() const {
    return Pos;
}

bool DefRocket::IsLaunched() const {
    return IsLaunchedFlag;
}

bool DefRocket::IsExploded() const {
    auto curr = Pos + Speed * Timer.GetElapsedTimeAsMs();
    return curr.Z >= MeetingPoint.Z || curr.Y >= MeetingPoint.Y || std::abs(curr.X) >= std::abs(MeetingPoint.X);
}


Defense::Defense(const Flat::Parameters& params)
    : Params(params)
{}

void Defense::LaunchRockets(const std::vector<std::pair<Vector3d, unsigned>>& meetingPointsAndTargetIds) {
    for (const auto& [point, targetId] : meetingPointsAndTargetIds) {
        Rockets.emplace_back(
            DefRocket(point, Params.defense()->rocket_speed(), Params.defense()->time_to_launch_rocket()),
            targetId
        );
    }
}

std::vector<unsigned> Defense::GetDestroyedTargetsId() {
    std::vector<unsigned> res;
    for (int i = 0; i < Rockets.size(); i++) {
        Rockets[i].first.UpdatePosition();

        if (Rockets[i].first.IsExploded()) {
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
