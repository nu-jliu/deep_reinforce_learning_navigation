#include <iostream>

#include "geometry2d.hpp"
#include "se2d.hpp"

namespace turtlelib
{
    std::ostream &operator<<(std::ostream &os, const Twist2D &tw)
    {
        os << "[" << tw.omega << " " << tw.x << " " << tw.y << "]";
        return os;
    }

};
