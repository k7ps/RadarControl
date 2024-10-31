#ifndef UTIL_H
#define UTIL_H

#include <array>

using PairDouble = std::array<double, 3>;
using TripleDouble = std::array<double, 3>;


PairDouble PolarToCartesian(double rad, double ang);
PairDouble CartesianToPolar(double x, double y);

TripleDouble CylindricalToCartesian(double rad, double ang, double h);
TripleDouble CartesianToCylindrical(double x, double y, double z);

bool GetRandomTrue(float probability);
double GetRandomDouble(double min, double max);

double RadToDeg(double angle);
float RadToDeg(float angle);

double DegToRad(double angle);
float DegToRad(float angle);


#endif // UTIL_H
