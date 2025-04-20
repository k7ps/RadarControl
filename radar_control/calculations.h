#ifndef CALCULATIONS_H
#define CALCULATIONS_H


#include "util/points.h"

#include <utility>
#include <vector>


std::pair<Vector3d, Vector3d> ABFilter(Vector3d x, Vector3d prevX, Vector3d prevSpeed, double dt, int measureCount);

Vector3d CalculateMeetPoint(
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
    double meetPointAngle,
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

std::vector<double> SolveQuadraticEquation(double a, double b, double c);


#endif // CALCULATIONS_H
