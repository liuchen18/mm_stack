//
// Created by hilsys on 2021/1/20.
//

#ifndef MM_PLANNING_POINT3D_H
#define MM_PLANNING_POINT3D_H

#include <cmath>

class Point3d {
public:
    //! Constructor which takes x- ,y and z-coordinates.
    Point3d(const double x, const double y, const double t) : x_(x), y_(y), z_(t) {}

    //! Constructor returning the zero vector.
    Point3d() : Point3d(0, 0, 0) {}

    //! Getter for x component
    double x() const { return x_; }

    //! Getter for y component
    double y() const { return y_; }

    double z() const { return z_; }

    //! Setter for x component
    void set_x(const double x) { x_ = x; }

    //! Setter for y component
    void set_y(const double y) { y_ = y; }

    void set_z(const double t) { z_ = t; }

    //! Gets the length of the point3d
    double Length() const { return sqrt(x_*x_+y_*y_+z_*z_); }

    //! Gets the squared length of the point3d
    double LengthSquare() const { return x_ * x_ + y_ * y_+z_*z_; }


    //! Returns the distance to the given vector
    double DistanceTo(const Point3d &other) const  {
        const double dx = x_ - other.x_;
        const double dy = y_ - other.y_;
        const double dz = z_ - other.z_;
        return sqrt(dx * dx + dy * dy+dz*dz);
    }

    //! Returns the squared distance to the given vector
    double DistanceSquareTo(const Point3d &other) const  {
        const double dx = x_ - other.x_;
        const double dy = y_ - other.y_;
        const double dz = z_ - other.z_;
        return dx * dx + dy * dy+dz*dz;
    }

protected:
    double x_ = 0.0;
    double y_ = 0.0;
    double z_ = 0.0;
};



#endif //MM_PLANNING_POINT3D_H
