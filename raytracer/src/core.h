#ifndef CORE_H
#define CORE_H

#include <cmath>
#include <algorithm>
#include <limits>
#include <iostream>

constexpr double kEPS = 1e-6;
constexpr double kINF = std::numeric_limits<double>::infinity();

//
// Base 4D homogeneous coordinate math
//
struct Vec4 {
    double x{}, y{}, z{}, w{};
    Vec4() = default;
    Vec4(double xx, double yy, double zz, double ww): x(xx), y(yy), z(zz), w(ww) {}

    Vec4 operator+(const Vec4& v) const { return {x+v.x, y+v.y, z+v.z, w+v.w}; }
    Vec4 operator-(const Vec4& v) const { return {x-v.x, y-v.y, z-v.z, w-v.w}; }
    Vec4 operator*(double s)     const { return {x*s, y*s, z*s, w*s}; }
    Vec4 operator/(double s)     const { return {x/s, y/s, z/s, w/s}; }
    Vec4& operator+=(const Vec4& v){ x+=v.x; y+=v.y; z+=v.z; w+=v.w; return *this; }
    Vec4& operator*=(double s){ x*=s; y*=s; z*=s; w*=s; return *this; }
    Vec4 operator-() const { return {-x, -y, -z, -w}; }

    // Scalar product (ignores w for geometric vectors)
    double dot(const Vec4& v) const { return x*v.x + y*v.y + z*v.z; }
    
    // Cross product (only for 3D part)
    Vec4 cross(const Vec4& v) const {
        return { y*v.z - z*v.y, z*v.x - x*v.z, x*v.y - y*v.x, 0.0 };
    }
    
    double length() const { return std::sqrt(dot(*this)); }
    Vec4 normalized() const {
        double L = length();
        if (L > kEPS) {
            return {x/L, y/L, z/L, w}; // Don't normalize w component
        }
        return {0, 1, 0, w}; // Fallback direction
    }
};

inline Vec4 operator*(double s, const Vec4& v){ return v*s; }

//
// Point3 and Dir3 using homogeneous coordinates
//
struct Point3 : public Vec4 {
    Point3() : Vec4(0, 0, 0, 1) {}
    Point3(double xx, double yy, double zz) : Vec4(xx, yy, zz, 1) {}
    Point3(const Vec4& v) : Vec4(v.x, v.y, v.z, 1) {} // Force w=1 for points
};

struct Dir3 : public Vec4 {
    Dir3() : Vec4(0, 1, 0, 0) {}
    Dir3(double xx, double yy, double zz) : Vec4(xx, yy, zz, 0) {}
    Dir3(const Vec4& v) : Vec4(v.x, v.y, v.z, 0) {} // Force w=0 for directions
    
    Dir3 normalized() const { 
        double L = length();
        if (L > kEPS) {
            return Dir3(x/L, y/L, z/L);
        }
        return Dir3(0, 1, 0); // Fallback direction
    }
};

// For backward compatibility, also define Vec3 as alias to the 3D part
struct Vec3 {
    double x{}, y{}, z{};
    Vec3() = default;
    Vec3(double xx, double yy, double zz): x(xx), y(yy), z(zz) {}
    Vec3(const Vec4& v) : x(v.x), y(v.y), z(v.z) {}
    
    operator Vec4() const { return Vec4(x, y, z, 0); }
};

//
// 4x4 Matrix for transformations
//
struct Matrix4 {
    double m[4][4];
    
    Matrix4() {
        // Initialize as identity
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                m[i][j] = (i == j) ? 1.0 : 0.0;
            }
        }
    }
    
    Matrix4(double m00, double m01, double m02, double m03,
            double m10, double m11, double m12, double m13,
            double m20, double m21, double m22, double m23,
            double m30, double m31, double m32, double m33) {
        m[0][0] = m00; m[0][1] = m01; m[0][2] = m02; m[0][3] = m03;
        m[1][0] = m10; m[1][1] = m11; m[1][2] = m12; m[1][3] = m13;
        m[2][0] = m20; m[2][1] = m21; m[2][2] = m22; m[2][3] = m23;
        m[3][0] = m30; m[3][1] = m31; m[3][2] = m32; m[3][3] = m33;
    }
    
    // Matrix multiplication
    Matrix4 operator*(const Matrix4& other) const {
        Matrix4 result;
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                result.m[i][j] = 0;
                for (int k = 0; k < 4; k++) {
                    result.m[i][j] += m[i][k] * other.m[k][j];
                }
            }
        }
        return result;
    }
    
    // Matrix-vector multiplication
    Vec4 operator*(const Vec4& v) const {
        return Vec4(
            m[0][0]*v.x + m[0][1]*v.y + m[0][2]*v.z + m[0][3]*v.w,
            m[1][0]*v.x + m[1][1]*v.y + m[1][2]*v.z + m[1][3]*v.w,
            m[2][0]*v.x + m[2][1]*v.y + m[2][2]*v.z + m[2][3]*v.w,
            m[3][0]*v.x + m[3][1]*v.y + m[3][2]*v.z + m[3][3]*v.w
        );
    }
    
    // Create transformation matrices
    static Matrix4 translation(double tx, double ty, double tz) {
        return Matrix4(
            1, 0, 0, tx,
            0, 1, 0, ty,
            0, 0, 1, tz,
            0, 0, 0, 1
        );
    }
    
    static Matrix4 scaling(double sx, double sy, double sz) {
        return Matrix4(
            sx, 0,  0,  0,
            0,  sy, 0,  0,
            0,  0,  sz, 0,
            0,  0,  0,  1
        );
    }
    
    static Matrix4 rotation_x(double angle) {
        double c = std::cos(angle);
        double s = std::sin(angle);
        return Matrix4(
            1, 0,  0, 0,
            0, c, -s, 0,
            0, s,  c, 0,
            0, 0,  0, 1
        );
    }
    
    static Matrix4 rotation_y(double angle) {
        double c = std::cos(angle);
        double s = std::sin(angle);
        return Matrix4(
             c, 0, s, 0,
             0, 1, 0, 0,
            -s, 0, c, 0,
             0, 0, 0, 1
        );
    }
    
    static Matrix4 rotation_z(double angle) {
        double c = std::cos(angle);
        double s = std::sin(angle);
        return Matrix4(
            c, -s, 0, 0,
            s,  c, 0, 0,
            0,  0, 1, 0,
            0,  0, 0, 1
        );
    }
    
    // FIXED: Proper analytic inverse for transformation matrices
    Matrix4 inverse() const {
        // For transformation matrices, use analytic inverse
        Matrix4 inv;
        
        // Extract the 3x3 rotation/scale part and translation
        double r11 = m[0][0], r12 = m[0][1], r13 = m[0][2], tx = m[0][3];
        double r21 = m[1][0], r22 = m[1][1], r23 = m[1][2], ty = m[1][3];
        double r31 = m[2][0], r32 = m[2][1], r33 = m[2][2], tz = m[2][3];
        
        // For scaling matrices, this is just 1/scale
        // For rotation matrices, this is transpose  
        // For combined, we need the full 3x3 inverse
        
        // Calculate determinant of 3x3 part
        double det = r11*(r22*r33 - r23*r32) - r12*(r21*r33 - r23*r31) + r13*(r21*r32 - r22*r31);
        
        if (std::abs(det) < 1e-12) {
            // Singular matrix, return identity
            return Matrix4();
        }
        
        double inv_det = 1.0 / det;
        
        // Calculate inverse of 3x3 part
        inv.m[0][0] = (r22*r33 - r23*r32) * inv_det;
        inv.m[0][1] = (r13*r32 - r12*r33) * inv_det;
        inv.m[0][2] = (r12*r23 - r13*r22) * inv_det;
        
        inv.m[1][0] = (r23*r31 - r21*r33) * inv_det;
        inv.m[1][1] = (r11*r33 - r13*r31) * inv_det;
        inv.m[1][2] = (r13*r21 - r11*r23) * inv_det;
        
        inv.m[2][0] = (r21*r32 - r22*r31) * inv_det;
        inv.m[2][1] = (r12*r31 - r11*r32) * inv_det;
        inv.m[2][2] = (r11*r22 - r12*r21) * inv_det;
        
        // Calculate inverse translation: -R^(-1) * t
        inv.m[0][3] = -(inv.m[0][0]*tx + inv.m[0][1]*ty + inv.m[0][2]*tz);
        inv.m[1][3] = -(inv.m[1][0]*tx + inv.m[1][1]*ty + inv.m[1][2]*tz);
        inv.m[2][3] = -(inv.m[2][0]*tx + inv.m[2][1]*ty + inv.m[2][2]*tz);
        
        // Bottom row stays [0,0,0,1]
        inv.m[3][0] = 0; inv.m[3][1] = 0; inv.m[3][2] = 0; inv.m[3][3] = 1;
        
        return inv;
    }
};

//
// Ray type
//
struct Ray {
    Point3 o;  // origin (point)
    Dir3 d;    // direction
    Ray() = default;
    Ray(const Point3& oo, const Dir3& dd): o(oo), d(dd.normalized()) {}
    Point3 at(double t) const { return Point3(o.x + d.x*t, o.y + d.y*t, o.z + d.z*t); }
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