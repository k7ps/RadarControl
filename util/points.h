#ifndef POINTS_H
#define POINTS_H


struct Vector2d {
    Vector2d() = default;
    Vector2d(double x, double y)
        : X(x), Y(y) {}

    double X;
    double Y;
};

struct Vector3d {
    Vector3d() = default;
    Vector3d(double x, double y, double z)
        : X(x), Y(y), Z(z) {}

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

double GetSqrtOfSquareSum(const Vector3d& v);


#endif // POINTS_H
