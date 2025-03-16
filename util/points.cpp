#include "points.h"

#include <math.h>


std::string Vector2d::DebugString() const {
    return "X = " + std::to_string(X) + ", Y = " + std::to_string(Y);
}

std::string Vector3d::DebugString() const {
    return "X = " + std::to_string(X) + ", Y = " + std::to_string(Y) + ", Z = " + std::to_string(Z);
}



Vector3d operator+(const Vector3d& p1, const Vector3d& p2) {
    return Vector3d(p1.X + p2.X, p1.Y + p2.Y, p1.Z + p2.Z);
}

Vector3d& operator+=(Vector3d& p1, const Vector3d& p2) {
    p1 = p1 + p2;
    return p1;
}

Vector3d operator-(const Vector3d& p1, const Vector3d& p2) {
    return Vector3d(p1.X - p2.X, p1.Y - p2.Y, p1.Z - p2.Z);
}

Vector3d& operator-=(Vector3d& p1, const Vector3d& p2){
    p1 = p1 - p2;
    return p1;
}

Vector3d operator*(const Vector3d& p1, double p2) {
    return Vector3d(p1.X * p2, p1.Y * p2, p1.Z * p2);
}

Vector3d& operator*=(Vector3d& p1, double p2) {
    p1 = p1 * p2;
    return p1;
}

Vector3d operator*(const Vector3d& p1, const Vector3d& p2) {
    return Vector3d(p1.X * p2.X, p1.Y * p2.Y, p1.Z * p2.Z);
}

Vector3d& operator*=(Vector3d& p1, const Vector3d& p2) {
    p1 = p1 * p2;
    return p1;
}

Vector3d operator/(const Vector3d& p1, double p2) {
    return Vector3d(p1.X / p2, p1.Y / p2, p1.Z / p2);
}

Vector3d& operator/=(Vector3d& p1, double p2) {
    p1 = p1 / p2;
    return p1;
}

bool operator==(const Vector3d& p1, const Vector3d& p2) {
    return std::abs(p1.X - p2.X) < EPS
        && std::abs(p1.Y - p2.Y) < EPS
        && std::abs(p1.Z - p2.Z) < EPS;
}

bool operator!=(const Vector3d& p1, const Vector3d& p2) {
    return !(p1 == p2);
}


double SqrtOfSumSquares(const Vector3d& v) {
    return std::sqrt(v.X * v.X + v.Y * v.Y + v.Z * v.Z);
}

double Distance(const Vector3d& p1, const Vector3d& p2) {
    return SqrtOfSumSquares(p2 - p1);
}

bool IsSignsEqual(const Vector3d& p1, const Vector3d& p2) {
    auto isSignEqual = [](double p1, double p2) {
        return (p1 > 0 && p2 >0) || (p1 < 0 && p2 < 0);
    };
    return isSignEqual(p1.X, p2.X) && isSignEqual(p1.Y, p2.Y) && isSignEqual(p1.Z, p2.Z);
}
