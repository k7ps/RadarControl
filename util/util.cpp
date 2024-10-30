#include "util.h"

#include <math.h>


PairDouble PolarToCartesian(double rad, double ang) {
    return {rad * std::cos(ang), rad * std::sin(ang)};
}

PairDouble CartesianToPolar(double x, double y) {
    return {std::sqrt(x*x + y*y), std::atan2(y, x)};
}

bool GetRandomTrue(float probability) {
    float x = (float) rand() / RAND_MAX;
    return x <= probability;
}

float GetRandomFloat(float min, float max) {
    return min + (float) rand() / RAND_MAX * (max - min);
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
