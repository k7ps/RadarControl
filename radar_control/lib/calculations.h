#ifndef CALCULATIONS_H
#define CALCULATIONS_H

#include "util/points.h"

#include <vector>


Vector3d CalculateSpeed(std::vector<Vector3d> positions, std::vector<int> times);

std::pair<Vector3d, Vector3d> ABFilter(Vector3d x, Vector3d prevX, Vector3d prevSpeed, double dt, int measureCount);

Vector3d CalculateMeetingPoint(
    const Vector3d& targetPos,
    const Vector3d& targetSpeed,
    const Vector3d& rocketPos,
    double rocketSpeed
);


#endif // CALCULATIONS_H
