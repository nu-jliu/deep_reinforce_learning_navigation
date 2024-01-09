#include "geometry2d.hpp"
#include <iostream>
#include <string>
#include <iomanip>

// using namespace std;
namespace turtlelib
{
    double normalize_angle(double rad)
    {
        if (rad > -turtlelib::PI && rad <= turtlelib::PI)
            return rad;

        else if (rad > turtlelib::PI)
            return turtlelib::normalize_angle(rad - 2 * turtlelib::PI);

        else
            return turtlelib::normalize_angle(rad + 2 * turtlelib::PI);
    }

    std::ostream &operator<<(std::ostream &os, const Point2D &p)
    {
        os << std::setprecision(15) << "[" << p.x << " " << p.y << "]";
        return os;
    }

    std::istream &operator>>(std::istream &is, Point2D &p)
    {
        std::string x_str;
        std::string y_str;

        char first = is.peek();
        bool has_brk = (first == '[');

        is >> x_str;
        is >> y_str;

        if (has_brk)
        {
            x_str = x_str.substr(1, x_str.length() - 1);
            y_str = y_str.substr(0, y_str.length() - 1);
        }

        p.x = stod(x_str);
        p.y = stod(y_str);

        return is;
    }

    Vector2D operator-(const Point2D &head, const Point2D &tail)
    {
        struct Vector2D *result = (struct Vector2D *)malloc(sizeof(struct Vector2D));
        result->x = head.x - tail.x;
        result->y = head.y - tail.y;

        return *result;
    }

    Point2D operator+(const Point2D &head, const Vector2D &disp)
    {
        struct Point2D *result = (struct Point2D *)malloc(sizeof(struct Point2D));
        result->x = head.x + disp.x;
        result->y = head.y + disp.y;

        return *result;
    }

    std::ostream &operator<<(std::ostream &os, const Vector2D &v)
    {
        os << "[" << v.x << " " << v.y << "]";

        return os;
    }

    std::istream &operator>>(std::istream &is, Vector2D &v)
    {
        std::string x_str;
        std::string y_str;

        char first = is.peek();
        bool has_brk = (first == '[');

        is >> x_str;
        is >> y_str;

        if (has_brk)
        {
            x_str = x_str.substr(1, x_str.length() - 1);
            y_str = y_str.substr(0, y_str.length() - 1);
        }

        v.x = stod(x_str);
        v.y = stod(y_str);

        return is;
    }
};