#ifndef UTIL_H
#define UTIL_H

#include <SFML/System/Vector2.hpp>


sf::Vector2f ToWindowXY(float rad, float ang);
sf::Vector2f ToWindowXY(sf::Vector2f);

sf::Vector2f ToPolarSystem(float x, float y);
sf::Vector2f ToPolarSystem(sf::Vector2f);


#endif // UTIL_H
