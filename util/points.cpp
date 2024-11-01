#include "points.h"

#include <math.h>


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


double GetSqrtOfSquareSum(const Vector3d& v) {
    return std::sqrt(v.X*v.X + v.Y*v.Y + v.Z*v.Z);
}
