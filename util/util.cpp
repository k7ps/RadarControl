#include "util.h"

#include <math.h>


sf::Vector2f ToWindowXY(double rad, double ang) {
    return sf::Vector2f(rad * std::cos(ang), -rad * std::sin(ang));
}
