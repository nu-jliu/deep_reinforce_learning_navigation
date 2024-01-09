#include <iostream>
#include <cmath>

#include "turtlelib/se2d.hpp"

namespace turtlelib
{
    std::ostream &operator<<(std::ostream &os, const Twist2D &tw)
    {
        os << "[" << tw.omega << " " << tw.x << " " << tw.y << "]";
        return os;
    }

    std::istream &operator>>(std::istream &is, Twist2D &tw)
    {
        char first = is.peek();
        bool has_brk = (first == '[');

        std::string str_w;
        std::string str_x;
        std::string str_y;

        is >> str_w;
        is >> str_x;
        is >> str_y;

        if (has_brk)
        {
            str_w = str_w.substr(1, str_w.length() - 1);
            str_y = str_y.substr(0, str_y.length() - 1);
        }

        tw.omega = stod(str_w);
        tw.x = stod(str_x);
        tw.y = stod(str_y);

        return is;
    }

    inline Transform2D::Transform2D()
    {
        __twist.omega = 0.0;
        __twist.x = 0.0;
        __twist.y = 0.0;
    }

    inline Transform2D::Transform2D(Vector2D trans)
    {
        __twist.omega = 0.0;
        __twist.x = trans.x;
        __twist.y = trans.y;
    }

    inline Transform2D::Transform2D(double radians)
    {
        __twist.omega = radians;
        __twist.x = 0.0;
        __twist.y = 0.0;
    }

    inline Transform2D::Transform2D(Vector2D trans, double radians)
    {
        __twist.omega = radians;
        __twist.x = trans.x;
        __twist.y = trans.y;
    }

    inline Point2D Transform2D::operator()(Point2D p) const
    {
        double matrix[3][3];
        matrix[0][0] = cos(__twist.omega);
        matrix[0][1] = -sin(__twist.omega);
        matrix[1][0] = sin(__twist.omega);
        matrix[1][1] = cos(__twist.omega);

        matrix[0][2] = __twist.x;
        matrix[1][2] = __twist.y;
        matrix[2][2] = 1.0;

        struct Point2D *result = (struct Point2D *)malloc(sizeof(struct Point2D));

        result->x = matrix[0][0] * p.x + matrix[0][1] * p.y + matrix[0][2];
        result->y = matrix[1][0] * p.x + matrix[1][1] * p.y + matrix[1][2];

        return *result;
    }

    inline Vector2D Transform2D::operator()(Vector2D v) const
    {
        struct Vector2D *result = (struct Vector2D *)malloc(sizeof(struct Vector2D));

        result->x = __trans_matrix[0][0] * v.x + __trans_matrix[0][1] * v.y + __trans_matrix[0][2];
        result->y = __trans_matrix[1][0] * v.x + __trans_matrix[1][1] * v.y + __trans_matrix[1][2];

        return *result;
    }

    inline Twist2D Transform2D::operator()(Twist2D v) const
    {
        struct Twist2D *result = (struct Twist2D *)malloc(sizeof(struct Twist2D));

        double angle = v.omega;

        double cos_omg = __trans_matrix[0][0] * cos(angle) + __trans_matrix[0][1] * sin(angle);
        double tw_x = __trans_matrix[0][0] * v.x + __trans_matrix[0][1] * v.y + __trans_matrix[0][2];
        double tw_y = __trans_matrix[1][0] * v.x + __trans_matrix[1][1] * v.y + __trans_matrix[1][2];

        result->omega = acos(cos_omg);
        result->x = tw_x;
        result->y = tw_y;

        return *result;
    }

    inline Transform2D Transform2D::inv() const
    {
        double new_tran[3][3];

        new_tran[0][0] = __trans_matrix[0][0];
        new_tran[0][1] = __trans_matrix[1][0];
        new_tran[1][0] = __trans_matrix[0][1];
        new_tran[1][1] = __trans_matrix[1][1];

        new_tran[0][2] = -(__trans_matrix[0][0] * __trans_matrix[0][2] + __trans_matrix[0][1] * __trans_matrix[1][2]);
        new_tran[1][2] = -(__trans_matrix[1][0] * __trans_matrix[0][2] + __trans_matrix[1][1] * __trans_matrix[1][2]);

        new_tran[2][0] = 0.0;
        new_tran[2][1] = 0.0;
        new_tran[2][2] = 1.0;

        double radian = atan2(new_tran[0][0], new_tran[1][0]);
        struct Vector2D v;
        v.x = new_tran[0][2];
        v.y = new_tran[1][2];

        return Transform2D(v, radian);
    }

    inline Transform2D &Transform2D::operator*=(const Transform2D &rhs)
    {
        double c1 = cos(__twist.omega);
        double s1 = sin(__twist.omega);
        double x1 = __twist.x;
        double y1 = __twist.y;

        double c2 = cos(rhs.rotation());
        double s2 = sin(rhs.rotation());
        double x2 = rhs.translation().x;
        double y2 = rhs.translation().y;

        double cos_omg = c1 * c2 - s1 * s2;
        double sin_omg = s1 * c2 + c1 * s2;
        double omega = atan2(cos_omg, sin_omg);

        double x = c1 * x2 - s1 * y2 + x1;
        double y = s1 * x2 + c1 * y2 + y1;

        __twist.omega = omega;
        __twist.x = x;
        __twist.y = y;

        return *this;
    }

    inline Vector2D Transform2D::translation() const
    {
        return {__twist.x, __twist.y};
    }

    inline double Transform2D::rotation() const
    {
        return __twist.omega;
    }

    std::ostream &operator<<(std::ostream &os, const Transform2D &tf)
    {
        os << "deg: " << rad2deg(tf.rotation());
        os << " x: " << tf.translation().x;
        os << " y: " << tf.translation().y;

        return os;
    }

    std::istream &operator>>(std::istream &is, Transform2D &tf)
    {
        std::string deg_str;
        std::string x_str;
        std::string y_str;
        std::string temp;

        is >> temp;
        is >> deg_str;
        is >> temp;
        is >> x_str;
        is >> temp;
        is >> y_str;

        double deg = stod(deg_str);
        double x = stod(x_str);
        double y = stod(y_str);

        struct Vector2D trans = {x, y};
        tf = Transform2D(trans, deg2rad(deg));

        return is;
    }

    Transform2D operator*(Transform2D lhs, const Transform2D &rhs)
    {
        lhs *= rhs;
        return lhs;
    }

};
