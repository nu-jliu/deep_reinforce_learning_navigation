#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"

#ifndef TURTLELIB_SVG_HPP_INCLUDE_GUARD
#define TURTLELIB_SVG_HPP_INCLUDE_GUARD

namespace turtlelib
{
    class Svg
    {
    private:
        /* data */
    public:
        /// @brief Reads in a default svg file.
        Svg();

        /// @brief
        /// @param filename
        Svg(std::string filename);
    };

};

#endif