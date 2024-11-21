#ifndef POINTS_H
#define POINTS_H

#include <string>

struct Vector2d {
    Vector2d() = default;
    Vector2d(double x, double y)
        : X(x), Y(y) {}

    std::string DebugString() const;

    double X;
    double Y;
};

struct Vector3d {
    Vector3d() = default;
    Vector3d(double x, double y, double z)
        : X(x), Y(y), Z(z) {}

    std::string DebugString() const;

    double X;
    double Y;
    double Z;
};

Vector3d operator+(const Vector3d& p1, const Vector3d& p2);
Vector3d& operator+=(Vector3d& p1, const Vector3d& p2);
Vector3d operator-(const Vector3d& p1, const Vector3d& p2);
Vector3d& operator-=(Vector3d& p1, const Vector3d& p2);
Vector3d operator*(const Vector3d& p1, double p2);
Vector3d& operator*=(Vector3d& p1, const Vector3d& p2);
Vector3d operator/(const Vector3d& p1, double p2);
Vector3d& operator/=(Vector3d& p1, const Vector3d& p2);
bool operator==(const Vector3d& p1, const Vector3d& p2);

std::string Vector3dAsStr(const Vector3d& p);

double GetSqrtOfSquareSum(const Vector3d& v);
double Distance(const Vector3d& p1, const Vector3d& p2);


#endif // POINTS_H
