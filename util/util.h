#ifndef UTIL_H
#define UTIL_H

#include "points.h"

#include <algorithm>
#include <sstream>
#include <vector>


Vector2d PolarToCartesian(double rad, double ang);
Vector2d PolarToCartesian(const Vector2d& p);
Vector2d CartesianToPolar(double x, double y);
Vector2d CartesianToPolar(const Vector2d& p);

Vector3d CylindricalToCartesian(double rad, double ang, double h);
Vector3d CylindricalToCartesian(const Vector3d& p);
Vector3d CartesianToCylindrical(double x, double y, double z);
Vector3d CartesianToCylindrical(const Vector3d& p);

double CalculateAngle(Vector3d p, Vector3d center = Vector3d::Zero());

bool GetRandomTrue(float probability);
double GetRandomDouble(double min, double max);
Vector3d GetRandomVector3d(double min, double max);

double GetRandomNormal(double mean, double std);
Vector3d GetRandomNormalVector3d(double mean, double std);
Vector3d GetRandomNormalVector3d(Vector3d mean, Vector3d std);

double RadToDeg(double angle);
float RadToDeg(float angle);

double DegToRad(double angle);
float DegToRad(float angle);

std::string CurrentTimeToString();
std::string MillisecondsToString(double ms);

bool IsInSegment(double c, double a, double b);
bool IsInSegment(const std::vector<double>& c, double a, double b);

bool IsInAnySegment(const std::vector<std::pair<double, double>>& segments, const std::vector<double>& points);
int InWhichSegment(const std::vector<std::pair<double, double>>& segments, double point);
std::vector<std::pair<double, double>> ShiftSegments(
    const std::vector<std::pair<double, double>>& segments,
    double shift
);

template<class T>
bool IsInVector(const std::vector<T>& vec, const T& i) {
    return std::find(vec.begin(), vec.end(), i) != vec.end();
}

template<class T>
std::string VectorToString(const std::vector<T>& vec, const std::string& sep = ", ") {
    std::ostringstream res;
    for (int i = 0; i < vec.size(); ++i) {
        if (i != 0) res << sep;
        res << vec[i];
    }
    return res.str();
}

template<class T>
void JoinToVector(std::vector<T>& a, const std::vector<T>& b) {
    for (auto i : b) {
        a.push_back(i);
    }
}

template<class T>
std::vector<T> ConcatenateVectors(const std::vector<T>& a, const std::vector<T>& b) {
    std::vector<T> res;
    for (auto i : a) {
        res.push_back(i);
    }
    for (auto i : b) {
        res.push_back(i);
    }
    return res;
}

template<class T>
T Clip(T x, T low, T up) {
    return std::max(low, std::min(up, x));
}


#endif // UTIL_H
