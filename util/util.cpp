#include "util.h"
#include "util/points.h"

#include <math.h>
#include <random>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <ctime>


Vector2d PolarToCartesian(double rad, double ang) {
    return {rad * std::cos(ang), rad * std::sin(ang)};
}

Vector2d PolarToCartesian(const Vector2d& p) {
    return PolarToCartesian(p.X, p.Y);
}

Vector2d CartesianToPolar(double x, double y) {
    return {std::sqrt(x*x + y*y), std::atan2(y, x)};
}

Vector2d CartesianToPolar(const Vector2d& p) {
    return CartesianToPolar(p.X, p.Y);
}

Vector3d CylindricalToCartesian(double rad, double ang, double h) {
    auto polar = PolarToCartesian(rad, ang);
    return {polar.X, polar.Y, h};
}

Vector3d CylindricalToCartesian(const Vector3d& p) {
    return CylindricalToCartesian(p.X, p.Y, p.Z);
}

Vector3d CartesianToCylindrical(double x, double y, double z) {
    auto cartesian = CartesianToPolar(x, y);
    return {cartesian.X, cartesian.Y, z};
}

Vector3d CartesianToCylindrical(const Vector3d& p) {
    return CartesianToCylindrical(p.X, p.Y, p.Z);
}

double GetPhi(Vector3d p, Vector3d center) {
    p = p - center;
    return CartesianToCylindrical(p).Y;
}

bool GetRandomTrue(float probability) {
    float x = (float) rand() / RAND_MAX;
    return x <= probability;
}

double GetRandomDouble(double min, double max) {
    return min + (double) rand() / RAND_MAX * (max - min);
}

Vector3d GetRandomVector3d(double min, double max) {
    return Vector3d(
        GetRandomDouble(min, max),
        GetRandomDouble(min, max),
        GetRandomDouble(min, max)
    );
}

double GetRandomNormal(double mean, double std) {
    static std::random_device rd;
    static std::mt19937 gen(rd());

    std::normal_distribution d(mean, std);
    return d(gen);
}

Vector3d GetRandomNormalVector3d(double mean, double std) {
    return Vector3d(
        GetRandomNormal(mean, std),
        GetRandomNormal(mean, std),
        GetRandomNormal(mean, std)
    );
}

Vector3d GetRandomNormalVector3d(Vector3d mean, Vector3d std) {
    return Vector3d(
        GetRandomNormal(mean.X, std.X),
        GetRandomNormal(mean.Y, std.Y),
        GetRandomNormal(mean.Z, std.Z)
    );
}


double RadToDeg(double angle) {
    return angle * 180 / M_PI;
}

float RadToDeg(float angle) {
    return angle * 180 / M_PI;
}

double DegToRad(double angle) {
    return angle * M_PI / 180;
}

float DegToRad(float angle) {
    return angle * M_PI / 180;
}

void PrintCurrentTime() {
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    auto tm = *std::localtime(&time_t);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
    std::cout << std::put_time(&tm, "%H:%M:%S") << '.' << std::setfill('0') << std::setw(3) << ms.count() << std::endl;
}

bool IsInSegment(double c, double a, double b) {
    return (a <= b && a <= c && c <= b) || (b <= a && b <= c && c <= a);
}

bool IsInSegment(const std::vector<double>& c, double a, double b) {
    for (auto i : c) {
        if (!IsInSegment(i, a ,b)) {
            return false;
        }
    }
    return true;
}
