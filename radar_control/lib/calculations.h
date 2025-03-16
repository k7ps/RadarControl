#ifndef CALCULATIONS_H
#define CALCULATIONS_H

#include "util/points.h"


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

double CalculateRadarAngle1Target(
    double currRadarAngle,
    double entryPointAngle,
    double meetingPointAngle,
    double viewAngle,
    double margin
);


#endif // CALCULATIONS_H
