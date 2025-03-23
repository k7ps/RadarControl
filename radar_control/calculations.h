#ifndef CALCULATIONS_H
#define CALCULATIONS_H


#include <vector>
#include <utility>

#include "util/points.h"


std::pair<Vector3d, Vector3d> ABFilter(Vector3d x, Vector3d prevX, Vector3d prevSpeed, double dt, int measureCount);

Vector3d CalculateMeetingPoint(
    const Vector3d& targetPos,
    const Vector3d& targetSpeed,
    double rocketSpeed,
    const Vector3d& radarPos = Vector3d::Zero()
);

Vector3d CalculateEntryPoint(
    const Vector3d& targetPos,
    const Vector3d& targetSpeed,
    double radius,
    const Vector3d& radarPos = Vector3d::Zero()
);

double CalculateRadarAngleOneTarget(
    double currRadarAngle,
    double currRadarTargetAngle,
    double entryPointAngle,
    double meetingPointAngle,
    double viewAngle,
    double margin
);

double CalculateRadarAngleMultiTarget(
    double currRadarAngle,
    double currRadarTargetAngle,
    const std::vector<double>& angles,
    double viewAngle,
    double margin
);

double CalculatePriority(Vector3d pos, Vector3d speed, double maxSpeed, Vector3d radarPoint = Vector3d::Zero());


#endif // CALCULATIONS_H
