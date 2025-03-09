#include "calculations.h"
#include "util/points.h"

#include <vector>
#include <cmath>


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
