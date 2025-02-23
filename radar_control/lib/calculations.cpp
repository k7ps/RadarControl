#include "calculations.h"
#include "util/points.h"


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
