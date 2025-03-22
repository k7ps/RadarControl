#include "calculations.h"
#include "util/points.h"
#include "util/util.h"

#include <cmath>
#include <iostream>
#include <vector>


namespace {

    std::pair<double, double> ABFilter1d(double measuredX, double prevX, double prevSpeed, double dt, int measureCount) {
        if (measureCount == 0) {
            return {measuredX, 0.};
        }
        if (measureCount == 1) {
            return {measuredX, (measuredX - prevX) / dt};
        }

        double alpha = 2. * (2. * measureCount - 1.) / (measureCount * (measureCount + 1.));
        double beta = 6. / (measureCount * (measureCount + 1.));

        double predictedX = prevX + prevSpeed * dt;
        double predictedSpeed = prevSpeed;

        double filteredX = predictedX + (alpha * (measuredX - predictedX));
        double filteredSpeed = predictedSpeed + (beta / dt * (measuredX - filteredX));

        return {filteredX, filteredSpeed};
    }

    std::vector<Vector3d> FindIntersectionsOfCircleAndLine(Vector3d center, double rad, Vector3d pos, Vector3d speed) {
        Vector3d p1 = pos, p2 = pos + speed;
        double a = p1.Y - p2.Y;
        double b = p2.X - p1.X;
        double c = p1.X * p2.Y - p1.Y * p2.X;

        if (c*c > rad*rad * (a*a + b*b) + EPS) {
            return {};
        }

        Vector3d p0(-a * c / (a*a + b*b), -b * c / (a*a + b*b));
        if (std::abs(c*c - rad*rad * (a*a + b*b)) < EPS) {
            return {p0};
        } else {
            double d = rad*rad - c*c / (a*a + b*b);
            double mult = std::sqrt(d / (a*a + b*b));
            return {
                Vector3d(p0.X + b * mult, p0.Y - a * mult),
                Vector3d(p0.X - b * mult, p0.Y + a * mult),
            };
        }
    }

    // m0 - point for distance, a - vector of line, m1 - point on line
    double DistancePoint2Line(Vector3d m0, Vector3d a, Vector3d m1) {
        auto m1m0 = m1 - m0;
        auto vecProd = Vector3d(
              a.Y * m1m0.Z - a.Z * m1m0.Y,
            - a.X * m1m0.Z + a.Z * m1m0.X,
              a.X * m1m0.Y - a.Y * m1m0.X
        );
        return SqrtOfSumSquares(vecProd) / SqrtOfSumSquares(a);
    }

}

std::pair<Vector3d, Vector3d> ABFilter(
    Vector3d measuredX,
    Vector3d prevX,
    Vector3d prevSpeed,
    double dt,
    int measureCount
) {
    auto x = ABFilter1d(measuredX.X, prevX.X, prevSpeed.X, dt, measureCount);
    auto y = ABFilter1d(measuredX.Y, prevX.Y, prevSpeed.Y, dt, measureCount);
    auto z = ABFilter1d(measuredX.Z, prevX.Z, prevSpeed.Z, dt, measureCount);

    return {
        Vector3d(x.first, y.first, z.first),
        Vector3d(x.second, y.second, z.second)
    };
}

Vector3d CalculateMeetingPoint(
    const Vector3d& targetPos,
    const Vector3d& targetSpeed,
    const Vector3d& rocketPos,
    double rocketSpeed
) {
    auto check = [&targetPos, &targetSpeed, &rocketPos, &rocketSpeed](double time) {
        auto targetNewPos = targetPos + targetSpeed * time;
        auto dist = Distance(targetNewPos, rocketPos);
        return rocketSpeed * time >= dist;
    };
    double timeL = 0, timeR = 1e5;
    while (timeR - timeL > 1e-7) {
        double t = (timeL + timeR) * 0.5;
        if (check(t)) {
            timeR = t;
        } else {
            timeL = t;
        }
    }
    return targetPos + targetSpeed * timeR;
}

Vector3d CalculateEntryPoint(
    const Vector3d& targetPos,
    const Vector3d& targetSpeed,
    const Vector3d& radarPos,
    double radius
) {
    auto points = FindIntersectionsOfCircleAndLine(radarPos, radius, targetPos, targetSpeed);
    if (points.empty()) {
        return Vector3d::Zero();
    } else if (points.size() == 1) {
        return points[0];
    } else {
        std::vector<double> distances;
        for (const auto& p : points) {
            distances.push_back(Distance(p, targetPos));
        }
        Vector3d res;
        double minDist = 1e9;
        for (int i = 0; i < distances.size(); ++i) {
            if (distances[i] < minDist) {
                minDist = distances[i];
                res = points[i];
            }
        }
        double timeTo = minDist / std::sqrt(targetSpeed.X * targetSpeed.X + targetSpeed.Y * targetSpeed.Y);
        res.Z = targetPos.Z + targetSpeed.Z * timeTo;
        return res;
    }
}

double CalculateRadarAngleOneTarget(
    double currRadarAngle,
    double currRadarTargetAngle,
    double entryPointAngle,
    double meetingPointAngle,
    double viewAngle,
    double margin
) {
    auto halfView = viewAngle / 2;

    auto willAngL = currRadarTargetAngle - halfView + margin;
    auto willAngR = currRadarTargetAngle + halfView - margin;

    auto angL = currRadarAngle - halfView + margin;
    auto angR = currRadarAngle + halfView - margin;
    auto entryIn = entryPointAngle == -1 || InSegment(entryPointAngle, angL, angR);
    auto meetingIn = meetingPointAngle == -1 || InSegment(meetingPointAngle, angL, angR);

    if (
        currRadarTargetAngle != -1
        && (entryPointAngle == -1 || InSegment(entryPointAngle, willAngL, willAngR))
        && (entryPointAngle == -1 || InSegment(meetingPointAngle, willAngL, willAngR))
    ) {
        return currRadarTargetAngle;
    } else if (entryIn && meetingIn) {
        return currRadarAngle;
    } else if (std::abs(entryPointAngle - meetingPointAngle) >= viewAngle - 2 * margin) {
        if (entryPointAngle > meetingPointAngle) {
            return entryPointAngle - halfView + margin;
        } else {
            return entryPointAngle + halfView - margin;
        }
    } else if (entryIn || meetingIn) {
        auto ang = (entryIn ? meetingPointAngle : entryPointAngle);
        if (ang < angL) {
            return ang + halfView - margin;
        } else {
            return ang - halfView + margin;
        }
    } else {
        if (entryPointAngle < angL) {
            return std::min(entryPointAngle, meetingPointAngle) + halfView - margin;
        } else {
            return std::max(entryPointAngle, meetingPointAngle) - halfView + margin;
        }
    }
}


// double CalculateRadarAngleMultiTarget(
//     double currRadarAngle,
//     const std::vector<double>& entryPointAngles,
//     const std::vector<double>& meetingPointAngles,
//     const std::vector<float>& priorities,
//     double viewAngle,
//     double margin
// ) {

// }


double CalculatePriority(Vector3d pos, Vector3d speed, double maxSpeed, Vector3d radarPoint) {
    auto dist2radar = DistancePoint2Line(radarPoint, speed, pos);
    auto absSpeed = SqrtOfSumSquares(speed);
    return 0.9 * std::pow(M_E, - 0.01 * dist2radar) + 0.1 * absSpeed / maxSpeed;
}
