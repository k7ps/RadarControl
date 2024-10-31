#include "util.h"

#include <math.h>


PairDouble PolarToCartesian(double rad, double ang) {
    return {rad * std::cos(ang), rad * std::sin(ang)};
}

PairDouble CartesianToPolar(double x, double y) {
    return {std::sqrt(x*x + y*y), std::atan2(y, x)};
}

TripleDouble CylindricalToCartesian(double rad, double ang, double h) {
    auto polar = PolarToCartesian(rad, ang);
    return {polar[0], polar[1], h};
}

TripleDouble CartesianToCylindrical(double x, double y, double z) {
    auto cartesian = CartesianToPolar(x, y);
    return {cartesian[0], cartesian[1], z};
}

bool GetRandomTrue(float probability) {
    float x = (float) rand() / RAND_MAX;
    return x <= probability;
}

double GetRandomDouble(double min, double max) {
    return min + (double) rand() / RAND_MAX * (max - min);
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
