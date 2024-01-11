#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"

#include <iostream>
#include <fstream>

#ifndef TURTLELIB_SVG_HPP_INCLUDE_GUARD
#define TURTLELIB_SVG_HPP_INCLUDE_GUARD
namespace turtlelib
{
    /// @brief
    class Svg
    {

    private:
        std::ofstream __ofs;
        Transform2D __tf_origin;
        double __origin_x;
        double __origin_y;
        /// @brief
        void init_svg()
        {

            __origin_x = 408.0;
            __origin_y = 528.0;

            __tf_origin = Transform2D(Vector2D{__origin_x, __origin_y}, 0.0);

            __ofs << "<svg width=\"8.500000in\" height=\"11.000000in\" viewBox=\"0 0 816.000000 1056.000000\" xmlns=\"http://www.w3.org/2000/svg\">" << std::endl;
            __ofs << "<defs>" << std::endl;
            __ofs << "<marker" << std::endl;
            __ofs << "style=\"overflow:visible\"" << std::endl;
            __ofs << "id=\"Arrow1Sstart\"" << std::endl;
            __ofs << "refX=\"0.0\"" << std::endl;
            __ofs << "refY=\"0.0\"" << std::endl;
            __ofs << "orient=\"auto\">" << std::endl;
            __ofs << "<path" << std::endl;
            __ofs << "transform=\"scale(0.2) translate(6,0)\"" << std::endl;
            __ofs << "style=\"fill-rule:evenodd;fill:context-stroke;stroke:context-stroke;stroke-width:1.0pt\"" << std::endl;
            __ofs << "d=\"M 0.0,0.0 L 5.0,-5.0 L -12.5,0.0 L 5.0,5.0 L 0.0,0.0 z \"" << std::endl;
            __ofs << "/>" << std::endl;
            __ofs << "</marker>" << std::endl;
            __ofs << "</defs>" << std::endl;

            __ofs << "<circle cx=\"504.2\" cy=\"403.5\" r=\"3\" stroke=\"purple\" fill=\"purple\" stroke-width=\"1\" />" << std::endl;
        }

    public:
        /// @brief Opens a default svg file.
        Svg();

        /// @brief Opens the svg file with specified filename
        /// @param filename the name of the svg file.
        explicit Svg(std::string filename);

        /// @brief Draw a with for the specified vector
        /// @param v the vector to be drawn.
        void draw_line(Vector2D v, std::string color);

        /// @brief
        /// @param p
        /// @param color
        void draw_point(Point2D p, std::string color);

        /// @brief
        /// @param tf
        /// @param color_x
        /// @param color_y
        /// @param color_z
        void draw_frame(Transform2D tf, std::string color_x, std::string color_y, std::string color_z);

        /// @brief
        void finish();
    };

};

#endif