#include "util.h"

#include <math.h>


sf::Vector2f ToWindowXY(float rad, float ang) {
    return sf::Vector2f(rad * std::cos(ang), -rad * std::sin(ang));
}

sf::Vector2f ToWindowXY(sf::Vector2f pos) {
    return ToWindowXY(pos.x, pos.y);
}

sf::Vector2f ToPolarSystem(float x, float y) {
    return sf::Vector2f(std::sqrt(x*x + y*y), std::atan2(-y, x));
}

sf::Vector2f ToPolarSystem(sf::Vector2f pos) {
    return ToPolarSystem(pos.x, pos.y);
}
