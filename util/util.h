#ifndef UTIL_H
#define UTIL_H

#include <utility>

using PairDouble = std::pair<double, double>;


PairDouble PolarToCartesian(double rad, double ang);
PairDouble CartesianToPolar(double x, double y);

bool GetRandomTrue(float probability);
float GetRandomFloat(float min, float max);

double RadToDeg(double angle);
float RadToDeg(float angle);

double DegToRad(double angle);
float DegToRad(float angle);


#endif // UTIL_H
