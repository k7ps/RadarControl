#ifndef CALCULATIONS_H
#define CALCULATIONS_H


#include "data.h"
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

double CalculateRadarAngleMultiTarget(
    double currRadarAngle,
    double currRadarTargetAngle,
    const std::vector<double>& angles,
    const std::vector<std::pair<double, double>>& radarSegments
);

double CalculatePriority(Vector3d pos, Vector3d speed, double maxSpeed, Vector3d radarPoint = Vector3d::Zero());

std::vector<double> SolveQuadraticEquation(double a, double b, double c);

double TimeToRotate(RadarPos curr, RadarTargetPos target, double maxEps);
double TimeToRotateToTarget(
    RadarPos pos,
    RadarTargetPos targetPos,
    const std::vector<double>& targetAngles,
    double maxAngleSpeed,
    double maxEps,
    double viewAngle,
    double margin
);

RadarPos UpdateRadarPos(RadarPos curr, RadarTargetPos target, double maxEps, double deltaTime);

bool CanAddToAngleArray(double maxDiff, const std::vector<double>& angles, const std::vector<double>& newAngles);

bool CanAddTargetToFollow(
    const std::vector<std::pair<double, double>>& radarSegments, // sorted
    std::vector<double> angles,
    const std::vector<double>& newAngles
);

bool IsInAnySegment(
    const std::vector<std::pair<double, double>>& segments,
    const std::vector<double>& points
);

int InWhichSegment(
    const std::vector<std::pair<double, double>>& segments,
    double point
);

std::vector<std::pair<double, double>> ShiftSegments(
    const std::vector<std::pair<double, double>>& segments,
    double shift
);


#endif // CALCULATIONS_H
