#ifndef UTIL_H
#define UTIL_H

#include "points.h"


Vector2d PolarToCartesian(double rad, double ang);
Vector2d PolarToCartesian(const Vector2d& p);
Vector2d CartesianToPolar(double x, double y);
Vector2d CartesianToPolar(const Vector2d& p);

Vector3d CylindricalToCartesian(double rad, double ang, double h);
Vector3d CylindricalToCartesian(const Vector3d& p);
Vector3d CartesianToCylindrical(double x, double y, double z);
Vector3d CartesianToCylindrical(const Vector3d& p);

bool GetRandomTrue(float probability);
double GetRandomDouble(double min, double max);
Vector3d GetRandomVector3d(double min, double max);

double RadToDeg(double angle);
float RadToDeg(float angle);

double DegToRad(double angle);
float DegToRad(float angle);


#endif // UTIL_H
