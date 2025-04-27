#include "calculations.h"
#include "radar_control/data.h"
#include "util/points.h"
#include "util/util.h"

#include <cmath>
#include <vector>


namespace {

    std::pair<double, double> ABFilter1d(double measuredX, double prevX, double prevSpeed, double dt, int measureCount) {
        if (measureCount == 0) {
            return {measuredX, 0.};
        }
        if (measureCount == 1) {
            return {measuredX, (measuredX - prevX) / dt};
        }
        if (measureCount > 50) {
            measureCount = 50;
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

Vector3d CalculateMeetPoint(
    const Vector3d& targetPos,
    const Vector3d& targetSpeed,
    double rocketSpeed,
    const Vector3d& radarPos
) {
    auto check = [&targetPos, &targetSpeed, &radarPos, &rocketSpeed](double time) {
        auto targetNewPos = targetPos + targetSpeed * time;
        auto dist = Distance(targetNewPos, radarPos);
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
    double radius,
    const Vector3d& radarPos
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
    double meetPointAngle,
    double viewAngle,
    double margin
) {
    auto halfView = viewAngle / 2;
    auto willAngL = currRadarTargetAngle - halfView + margin;
    auto willAngR = currRadarTargetAngle + halfView - margin;

    auto angL     = currRadarAngle - halfView + margin;
    auto angR     = currRadarAngle + halfView - margin;
    auto entryIn  = entryPointAngle == -1 || IsInSegment(entryPointAngle, angL, angR);
    auto meetIn   = meetPointAngle == -1 || IsInSegment(meetPointAngle, angL, angR);

    if (
        currRadarTargetAngle != -1
        && (entryPointAngle == -1 || IsInSegment(entryPointAngle, willAngL, willAngR))
        && (meetPointAngle == -1 || IsInSegment(meetPointAngle, willAngL, willAngR))
    ) {
        return currRadarTargetAngle;
    } else if (entryIn && meetIn) {
        return currRadarAngle;
    } else if (std::abs(entryPointAngle - meetPointAngle) >= viewAngle - 2 * margin) {
        if (entryPointAngle > meetPointAngle) {
            return entryPointAngle - halfView + margin;
        } else {
            return entryPointAngle + halfView - margin;
        }
    } else if (entryIn || meetIn) {
        auto ang = (entryIn ? meetPointAngle : entryPointAngle);
        if (ang < angL) {
            return ang + halfView - margin;
        } else {
            return ang - halfView + margin;
        }
    } else {
        if (entryPointAngle < angL) {
            return std::min(entryPointAngle, meetPointAngle) + halfView - margin;
        } else {
            return std::max(entryPointAngle, meetPointAngle) - halfView + margin;
        }
    }
}

double CalculateRadarAngleMultiTarget(
    double currRadarAngle,
    double currRadarTargetAngle,
    const std::vector<double>& angles,
    double viewAngle,
    double margin
) {
    return CalculateRadarAngleOneTarget(
        currRadarAngle,
        currRadarTargetAngle,
        *std::min_element(angles.begin(), angles.end()),
        *std::max_element(angles.begin(), angles.end()),
        viewAngle,
        margin
    );
}

double CalculatePriority(Vector3d pos, Vector3d speed, double maxSpeed, Vector3d radarPoint) {
    auto dist2radar = DistancePoint2Line(radarPoint, speed, pos);
    auto absSpeed = SqrtOfSumSquares(speed);
    return 0.9 * std::pow(M_E, - 0.01 * dist2radar) + 0.1 * absSpeed / maxSpeed;
}

std::vector<double> SolveQuadraticEquation(double a, double b, double c) {
    if (a == 0) {
        if (b != 0) return {-c / b};
        return {};
    }
    auto discriminant = b*b - 4*a*c;
    if (discriminant < 0) return {};
    if (discriminant == 0) return {-b / (2 * a)};
    return {
        (-b + std::sqrt(discriminant)) / (2 * a),
        (-b - std::sqrt(discriminant)) / (2 * a),
    };
}

double TimeToRotate(RadarPos curr, RadarTargetPos target, double maxEps) {
    auto time = 0.;
    auto distToTarget = target.Angle - curr.Angle;
    auto direction = (distToTarget > 0 ? 1. : -1.);
    if (std::abs(distToTarget) < 1e-3 && std::abs(curr.Speed) < maxEps * 400) return time;

    auto stoppingDist = 0.5 * curr.Speed * curr.Speed / maxEps;
    if (curr.Speed * direction < 0 || std::abs(distToTarget) < stoppingDist) {
        time += std::abs(curr.Speed) / maxEps;
        curr.Angle += (curr.Speed > 0 ? 1. : -1.) * curr.Speed * curr.Speed / (2 * maxEps);
        curr.Speed = 0;
        distToTarget = target.Angle - curr.Angle;
    }
    distToTarget = std::abs(distToTarget);

    auto accDist = 0.;
    auto decDist = 0.;
    auto currAbsSpeed = std::abs(curr.Speed);

    if (currAbsSpeed < target.Speed) {
        auto timeToMaxSpeed = (target.Speed - currAbsSpeed) / maxEps;
        accDist = currAbsSpeed * timeToMaxSpeed + 0.5 * maxEps * timeToMaxSpeed * timeToMaxSpeed;
    }
    decDist = 0.5 * target.Speed * target.Speed / maxEps;

    if (accDist + decDist < distToTarget) {
        time += (2 * target.Speed - currAbsSpeed) / maxEps + (distToTarget - accDist - decDist) / target.Speed;
    } else {
        auto times = SolveQuadraticEquation(
            maxEps,
            2 * currAbsSpeed,
            0.5 * currAbsSpeed * currAbsSpeed / maxEps - distToTarget
        );
        if (!times.empty()) {
            time += *std::max_element(times.begin(), times.end()) * 2;
        }
        time += currAbsSpeed / maxEps;
    }
    return time;
}

double TimeToRotateToTarget(
    RadarPos pos,
    RadarTargetPos targetPos,
    const std::vector<double>& targetAngles,
    double maxAngleSpeed,
    double maxEps,
    double viewAngle,
    double margin
) {
    auto to = CalculateRadarAngleMultiTarget(
        pos.Angle,
        targetPos.Angle,
        targetAngles,
        viewAngle,
        margin
    );
    return TimeToRotate(pos, RadarTargetPos{.Angle=to, .Speed=maxAngleSpeed}, maxEps);
}

RadarPos UpdateRadarPos(RadarPos curr, RadarTargetPos target, double maxEps, double deltaTime) {
    double distToTarget = target.Angle - curr.Angle;

    if (target.TimeToReach == -1) {
        double stoppingDist = 0.5 * curr.Speed * curr.Speed / maxEps;

        double eps = 0.;
        if (std::abs(distToTarget) <= stoppingDist || distToTarget * curr.Speed < 0) {
            eps = (curr.Speed > 0 ? -1. : 1.) * maxEps;
        } else {
            if (std::abs(curr.Speed) < target.Speed) {
                eps = (curr.Speed > 0 ? 1. : -1.) * maxEps;
            } else {
                eps = 0;
            }
        }
        double newSpeed = curr.Speed + eps * deltaTime;
        newSpeed = Clip(newSpeed, -target.Speed, target.Speed);

        double newAngle = curr.Angle + newSpeed * deltaTime;
        if (std::abs(newAngle - target.Angle) < 1e-3 && std::abs(newSpeed) < maxEps * 400) {
            newAngle = target.Angle;
            newSpeed = 0;
        }
        return RadarPos{
            .Angle = newAngle,
            .Speed = newSpeed
        };
    } else {
        if (distToTarget * curr.Speed < 0 || std::abs(curr.Speed * target.TimeToReach) > std::abs(distToTarget)) {
            double newSpeed = curr.Speed + (curr.Speed > 0 ? -maxEps * deltaTime : maxEps * deltaTime);
            newSpeed = Clip(newSpeed, -target.Speed, target.Speed);
            double newAngle = curr.Angle + newSpeed * deltaTime;
            return RadarPos{
                .Angle = newAngle,
                .Speed = newSpeed
            };
        }
        double optimalEps =
            2 * (distToTarget - curr.Speed * target.TimeToReach) / (target.TimeToReach * target.TimeToReach);
        optimalEps = Clip(optimalEps, -maxEps, maxEps);
        double newSpeed = curr.Speed + optimalEps * deltaTime;
        newSpeed = Clip(newSpeed, -target.Speed, target.Speed);
        double newAngle = curr.Angle + newSpeed * deltaTime;
        return RadarPos{
            .Angle = newAngle,
            .Speed = newSpeed
        };
    }
}

bool CanAddToAngleArray(
    double maxDiff,
    const std::vector<double>& angles,
    const std::vector<double>& newAngles
) {
    if (angles.empty()) return true;
    auto min = *std::min_element(angles.begin(), angles.end());
    auto max = *std::max_element(angles.begin(), angles.end());
    for (auto newAngle : newAngles) {
        if (newAngle != -1 && (std::abs(newAngle - min) > maxDiff || std::abs(newAngle - max) > maxDiff)) {
            return false;
        }
    }
    return true;
}
