#ifndef CORE_H
#define CORE_H

#include <cmath>
#include <algorithm>
#include <limits>
#include <iostream>

constexpr double kEPS = 1e-6;
constexpr double kINF = std::numeric_limits<double>::infinity();

//
// Base 3D vector math
//
struct Vec3 {
    double x{}, y{}, z{};
    Vec3() = default;
    Vec3(double xx, double yy, double zz): x(xx), y(yy), z(zz) {}

    Vec3 operator+(const Vec3& v) const { return {x+v.x, y+v.y, z+v.z}; }
    Vec3 operator-(const Vec3& v) const { return {x-v.x, y-v.y, z-v.z}; }
    Vec3 operator*(double s)     const { return {x*s, y*s, z*s}; }
    Vec3 operator/(double s)     const { return {x/s, y/s, z/s}; }
    Vec3& operator+=(const Vec3& v){ x+=v.x; y+=v.y; z+=v.z; return *this; }
    Vec3& operator*=(double s){ x*=s; y*=s; z*=s; return *this; }

    double dot(const Vec3& v) const { return x*v.x + y*v.y + z*v.z; }
    Vec3 cross(const Vec3& v) const {
        return { y*v.z - z*v.y, z*v.x - x*v.z, x*v.y - y*v.x };
    }
    double length() const { return std::sqrt(dot(*this)); }
    Vec3 normalized() const {
        double L = length();
        return (L>0)? (*this)/L : *this;
    }
};

inline Vec3 operator*(double s, const Vec3& v){ return v*s; }

//
// Distinguish between Point3 and Dir3
//
struct Point3 : public Vec3 {
    using Vec3::Vec3;   // inherit constructors
    Point3(const Vec3& v) : Vec3(v){}
};

struct Dir3 : public Vec3 {
    using Vec3::Vec3;   // inherit constructors
    Dir3(const Vec3& v) : Vec3(v){}
    Dir3 normalized() const { return Dir3(Vec3::normalized()); } // aded this line to get rid of 
};

//
// Ray type
//
struct Ray {
    Point3 o;  // origin (point)
    Dir3 d;    // direction
    Ray() = default;
    Ray(const Point3& oo, const Dir3& dd): o(oo), d(dd.normalized()) {}
    Point3 at(double t) const { return Point3(o + d*t); }
};

//
// Color type
//
struct Color {
    double r{}, g{}, b{};
    Color() = default;
    Color(double rr, double gg, double bb): r(rr), g(gg), b(bb) {}
    Color operator+(const Color& c) const { return {r+c.r, g+c.g, b+c.b}; }
    Color operator*(double s) const { return {r*s, g*s, b*s}; }
    Color& operator+=(const Color& c){ r+=c.r; g+=c.g; b+=c.b; return *this; }
};

inline Color hadamard(const Color& a, const Color& b){
    return {a.r*b.r, a.g*b.g, a.b*b.b};
}

inline double clamp01(double v){ return std::max(0.0, std::min(1.0, v)); }
inline int toByte(double v){ return (int)std::round(clamp01(v)*255.0); }

#endif
