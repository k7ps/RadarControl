#include "calculations.h"
#include "util/points.h"

#include <stdexcept>
#include <iostream>


namespace {

    double LeastSquaresSlope(const std::vector<double>& x, const std::vector<double>& y) {
        if (x.size() != y.size()) {
            throw std::invalid_argument(
                "LeastSquaresSlope(): x and y must be the same size, but got x.size() = "
                + std::to_string(x.size()) + " and y.size() = " + std::to_string(y.size())
            );
        }
        double n = x.size();
        double sumX = 0;
        double sumY = 0;
        double sumXY = 0;
        double sumXX = 0;
        for (int i = 0; i < n; ++i) {
            sumX += x[i];
            sumY += y[i];
            sumXY += x[i] * y[i];
            sumXX += x[i] * x[i];
        }
        return (n * sumXY - sumX * sumY) / (n * sumXX - sumX * sumX);
    }

}


Vector3d CalculateSpeed(std::vector<Vector3d> positions, std::vector<int> times) {
    std::vector<double> x, y, z, t;
    for (int i = 0; i < positions.size(); ++i) {
        x.push_back(positions[i].X);
        y.push_back(positions[i].Y);
        z.push_back(positions[i].Z);

        if (i == 0) {
            t.push_back(times[i]);
        } else {
            t.push_back(t.back() + times[i]);
        }
    }
    return Vector3d(
        LeastSquaresSlope(t, x),
        LeastSquaresSlope(t, y),
        LeastSquaresSlope(t, z)
    );
}

Vector3d CalculateMeetingPoint(
    const Vector3d& targetPos,
    const Vector3d& targetSpeed,
    const Vector3d& rocketPos,
    double rocketSpeed
) {
    // std::cout << "CalculateMeetingPoint(): started" << std::endl;
    std::cout << targetSpeed.DebugString() << "\n";
    std::cout << "Pos: " << targetPos.DebugString() << "\n";
    // std::cout << "RocketSpeed = " << rocketSpeed << "\n";
    // auto targetSpeed = Vector3d(-0.008202, -0.003823, -0.000134);

    auto check = [&targetPos, &targetSpeed, &rocketPos, &rocketSpeed](double time) {
        auto targetNewPos = targetPos + targetSpeed * time;
        auto dist = Distance(targetNewPos, rocketPos);
        // std::cout << "Time  = " << time << '\n';
        // std::cout << "\tTargetPos  = " << targetNewPos.DebugString() << '\n';
        // std::cout << "\tDistance   = " << dist << '\n';
        // std::cout << "\tRocketDist = " << rocketSpeed * time << '\n';
        // std::cout << "\tReturn     = " << int(rocketSpeed * time >= dist) << '\n';
        return rocketSpeed * time >= dist;
    };

    double timeL = 0, timeR = 1e5;
    int i=0;
    while (timeR - timeL > 1e-6) {
        double t = (timeL + timeR) * 0.5;
        if (check(t)) {
            timeR = t;
        } else {
            timeL = t;
        }
        // std::cout << timeL << ' ' << timeR << ' ' << timeR - timeL << '\n';
        // i++;
        // if (i > 50) break;
    }
    // std::cout << timeL << ' ' << timeR << ' ' << timeR - timeR << '\n';
    auto res = targetPos + targetSpeed * timeR;
    std::cout << targetPos.DebugString() << '\n';
    std::cout << res.DebugString() << '\n';
    return targetPos + targetSpeed * timeR;
}
