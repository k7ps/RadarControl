#ifndef POINTS_H
#define POINTS_H

#include <string>


const double EPS = 1e-6;

struct Vector2d {
    Vector2d()
        : X(0), Y(0) {}
    Vector2d(double x, double y)
        : X(x), Y(y) {}

    std::string DebugString() const;
    static Vector2d Zero() {
        return Vector2d(0, 0);
    };

    double X;
    double Y;
};

struct Vector3d {
    Vector3d()
        : X(0), Y(0), Z(0) {}
    Vector3d(double x, double y, double z)
        : X(x), Y(y), Z(z) {}
    Vector3d(double x, double y)
        : X(x), Y(y), Z(0) {}

    std::string DebugString() const;
    static Vector3d Zero() {
        return Vector3d(0, 0, 0);
    };

    double X;
    double Y;
    double Z;
};

Vector3d operator+(const Vector3d& p1, const Vector3d& p2);
Vector3d& operator+=(Vector3d& p1, const Vector3d& p2);
Vector3d operator-(const Vector3d& p1, const Vector3d& p2);
Vector3d& operator-=(Vector3d& p1, const Vector3d& p2);
Vector3d operator*(const Vector3d& p1, double p2);
Vector3d& operator*=(Vector3d& p1, double p2);
Vector3d operator*(const Vector3d& p1, const Vector3d& p2);
Vector3d& operator*=(Vector3d& p1, const Vector3d& p2);
Vector3d operator/(const Vector3d& p1, double p2);
Vector3d& operator/=(Vector3d& p1, const Vector3d& p2);
bool operator==(const Vector3d& p1, const Vector3d& p2);
bool operator!=(const Vector3d& p1, const Vector3d& p2);

std::string Vector3dAsStr(const Vector3d& p);

double SqrtOfSumSquares(const Vector3d& v);
double Distance(const Vector3d& p1, const Vector3d& p2);
bool IsSignsEqual(const Vector3d& p1, const Vector3d& p2);


#endif // POINTS_H
