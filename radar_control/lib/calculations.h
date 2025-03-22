#ifndef CALCULATIONS_H
#define CALCULATIONS_H


#include <vector>
#include <utility>

#include "util/points.h"


struct TargetCalculationInfo {
    int Id;
    double Priority;
    double MeetingPointAngle;
    double EntryPointAngle;
};


std::pair<Vector3d, Vector3d> ABFilter(Vector3d x, Vector3d prevX, Vector3d prevSpeed, double dt, int measureCount);

Vector3d CalculateMeetingPoint(
    const Vector3d& targetPos,
    const Vector3d& targetSpeed,
    const Vector3d& rocketPos,
    double rocketSpeed
);

Vector3d CalculateEntryPoint(
    const Vector3d& targetPos,
    const Vector3d& targetSpeed,
    const Vector3d& radarPos,
    double radius
);

double CalculateRadarAngleOneTarget(
    double currRadarAngle,
    double currRadarTargetAngle,
    double entryPointAngle,
    double meetingPointAngle,
    double viewAngle,
    double margin
);

std::pair<double, std::vector<int>> CalculateRadarAngleMultiTarget(
    double currRadarAngle,
    double currRadarTargetAngle,
    const std::vector<int> followedTargets,
    const std::vector<TargetCalculationInfo>& targets,
    double viewAngle,
    double margin
);

double CalculatePriority(Vector3d pos, Vector3d speed, double maxSpeed, Vector3d radarPoint = Vector3d::Zero());


#endif // CALCULATIONS_H
