#include "points.h"

#include <math.h>


std::string Vector2d::DebugString() const {
    return "X = " + std::to_string(X) + ", Y = " + std::to_string(Y) + ";";
}

std::string Vector3d::DebugString() const {
    return "X = " + std::to_string(X) + ", Y = " + std::to_string(Y) + ", Z = " + std::to_string(Z) + ";";
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

Vector3d operator/(const Vector3d& p1, double p2) {
    return Vector3d(p1.X / p2, p1.Y / p2, p1.Z / p2);
}

Vector3d& operator/=(Vector3d& p1, double p2) {
    p1 = p1 / p2;
    return p1;
}

bool operator==(const Vector3d& p1, const Vector3d& p2) {
    const double error = 1e-6;
    return std::abs(p1.X - p2.X) < error
        && std::abs(p1.Y - p2.Y) < error
        && std::abs(p1.Z - p2.Z) < error;
}

double GetSqrtOfSquareSum(const Vector3d& v) {
    return std::sqrt(v.X*v.X + v.Y*v.Y + v.Z*v.Z);
}

double Distance(const Vector3d& p1, const Vector3d& p2) {
    return GetSqrtOfSquareSum(p2 - p1);
}
