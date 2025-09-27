#ifndef CORE_H
#define CORE_H

#include <cmath>
#include <algorithm>
#include <limits>
#include <iostream>

/// Numerical epsilon for float comparisons.
constexpr double kEPS = 1e-6;
/// Positive infinity shortcut.
constexpr double kINF = std::numeric_limits<double>::infinity();

/**
 * @brief 4D homogeneous vector with 3D algebra (dot/cross ignore w).
 * Represents points/directions when paired with a w-tag; provides basic
 * arithmetic, norms, and normalization used across geometry and transforms.
 */
struct Vec4 {
    /// X component.
    double x{}, 
    /// Y component.
           y{}, 
    /// Z component.
           z{}, 
    /// Homogeneous tag: 0=direction, 1=point (other values allowed in math).
           w{};

    /// Default-initialized (zeros).
    Vec4() = default;
    /// Construct from components.
    Vec4(double xx, double yy, double zz, double ww): x(xx), y(yy), z(zz), w(ww) {}

    /// Vector addition.
    Vec4 operator+(const Vec4& v) const { return {x+v.x, y+v.y, z+v.z, w+v.w}; }
    /// Vector subtraction.
    Vec4 operator-(const Vec4& v) const { return {x-v.x, y-v.y, z-v.z, w-v.w}; }
    /// Scalar multiply.
    Vec4 operator*(double s)     const { return {x*s, y*s, z*s, w*s}; }
    /// Scalar divide.
    Vec4 operator/(double s)     const { return {x/s, y/s, z/s, w/s}; }
    /// In-place addition.
    Vec4& operator+=(const Vec4& v){ x+=v.x; y+=v.y; z+=v.z; w+=v.w; return *this; }
    /// In-place scalar multiply.
    Vec4& operator*=(double s){ x*=s; y*=s; z*=s; w*=s; return *this; }
    /// Unary minus.
    Vec4 operator-() const { return {-x, -y, -z, -w}; }

    /// 3D dot product (ignores w).
    double dot(const Vec4& v) const { return x*v.x + y*v.y + z*v.z; }
    /// 3D cross product; result has w=0.
    Vec4 cross(const Vec4& v) const {
        return { y*v.z - z*v.y, z*v.x - x*v.z, x*v.y - y*v.x, 0.0 };
    }
    /// Euclidean length of (x,y,z).
    double length() const { return std::sqrt(dot(*this)); }
    /// Unit-length version of (x,y,z); preserves w.
    Vec4 normalized() const {
        double L = length();
        if (L > kEPS) {
            return {x/L, y/L, z/L, w};
        }
        return {0, 1, 0, w};
    }
};

/// Scalar–vector multiplication.
inline Vec4 operator*(double s, const Vec4& v){ return v*s; }

/**
 * @brief 3D point wrapper (w=1).
 * Convenience type for positions; inherits Vec4 storage/ops with w fixed to 1.
 */
struct Point3 : public Vec4 {
    /// Origin point.
    Point3() : Vec4(0, 0, 0, 1) {}
    /// From xyz.
    Point3(double xx, double yy, double zz) : Vec4(xx, yy, zz, 1) {}
    /// From Vec4 (forces w=1).
    Point3(const Vec4& v) : Vec4(v.x, v.y, v.z, 1) {}
};

/**
 * @brief 3D direction wrapper (w=0).
 * Convenience type for directions; provides safe normalization.
 */
struct Dir3 : public Vec4 {
    /// Default unit Y direction.
    Dir3() : Vec4(0, 1, 0, 0) {}
    /// From xyz.
    Dir3(double xx, double yy, double zz) : Vec4(xx, yy, zz, 0) {}
    /// From Vec4 (forces w=0).
    Dir3(const Vec4& v) : Vec4(v.x, v.y, v.z, 0) {}
    /// Unit-length version (fallback if too small).
    Dir3 normalized() const { 
        double L = length();
        if (L > kEPS) {
            return Dir3(x/L, y/L, z/L);
        }
        return Dir3(0, 1, 0);
    }
};

/**
 * @brief Plain 3D vector (x,y,z) with implicit cast to homogeneous direction.
 * Lightweight carrier for 3D data used in loaders and utilities.
 */
struct Vec3 {
    /// X component.
    double x{}, 
    /// Y component.
           y{}, 
    /// Z component.
           z{};

    /// Default-initialized (zeros).
    Vec3() = default;
    /// From components.
    Vec3(double xx, double yy, double zz): x(xx), y(yy), z(zz) {}
    /// From Vec4 (drops w).
    Vec3(const Vec4& v) : x(v.x), y(v.y), z(v.z) {}
    /// Implicit cast to homogeneous direction (w=0).
    operator Vec4() const { return Vec4(x, y, z, 0); }
};

/**
 * @brief Column-major 4×4 transform matrix for affine 3D operations.
 * Supports composition, application to Vec4, common generators (T,S,R), and analytic inverse.
 */
struct Matrix4 {
    /// Elements m[row][col], column-major semantics for multiplication.
    double m[4][4];
    
    /// Identity by default.
    Matrix4() {
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                m[i][j] = (i == j) ? 1.0 : 0.0;
            }
        }
    }
    
    /// Construct from 16 scalars.
    Matrix4(double m00, double m01, double m02, double m03,
            double m10, double m11, double m12, double m13,
            double m20, double m21, double m22, double m23,
            double m30, double m31, double m32, double m33) {
        m[0][0] = m00; m[0][1] = m01; m[0][2] = m02; m[0][3] = m03;
        m[1][0] = m10; m[1][1] = m11; m[1][2] = m12; m[1][3] = m13;
        m[2][0] = m20; m[2][1] = m21; m[2][2] = m22; m[2][3] = m23;
        m[3][0] = m30; m[3][1] = m31; m[3][2] = m32; m[3][3] = m33;
    }
    
    /// Matrix composition (this ∘ other).
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
    
    /// Apply to Vec4.
    Vec4 operator*(const Vec4& v) const {
        return Vec4(
            m[0][0]*v.x + m[0][1]*v.y + m[0][2]*v.z + m[0][3]*v.w,
            m[1][0]*v.x + m[1][1]*v.y + m[1][2]*v.z + m[1][3]*v.w,
            m[2][0]*v.x + m[2][1]*v.y + m[2][2]*v.z + m[2][3]*v.w,
            m[3][0]*v.x + m[3][1]*v.y + m[3][2]*v.z + m[3][3]*v.w
        );
    }
    
    /// Translation matrix.
    static Matrix4 translation(double tx, double ty, double tz) {
        return Matrix4(
            1, 0, 0, tx,
            0, 1, 0, ty,
            0, 0, 1, tz,
            0, 0, 0, 1
        );
    }
    
    /// Non-uniform scaling matrix.
    static Matrix4 scaling(double sx, double sy, double sz) {
        return Matrix4(
            sx, 0,  0,  0,
            0,  sy, 0,  0,
            0,  0,  sz, 0,
            0,  0,  0,  1
        );
    }
    
    /// Rotation about +X by angle (radians).
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
    
    /// Rotation about +Y by angle (radians).
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
    
    /// Rotation about +Z by angle (radians).
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
    
    /**
     * @brief Analytic inverse for affine transforms (R|t; 0 0 0 1).
     * Assumes upper-left 3×3 is invertible; returns identity if near-singular.
     * @return Inverse of this matrix under affine assumption.
     */
    Matrix4 inverse() const {
        Matrix4 inv;
        double r11 = m[0][0], r12 = m[0][1], r13 = m[0][2], tx = m[0][3];
        double r21 = m[1][0], r22 = m[1][1], r23 = m[1][2], ty = m[1][3];
        double r31 = m[2][0], r32 = m[2][1], r33 = m[2][2], tz = m[2][3];
        double det = r11*(r22*r33 - r23*r32) - r12*(r21*r33 - r23*r31) + r13*(r21*r32 - r22*r31);
        if (std::abs(det) < 1e-12) {
            return Matrix4();
        }
        double inv_det = 1.0 / det;
        inv.m[0][0] = (r22*r33 - r23*r32) * inv_det;
        inv.m[0][1] = (r13*r32 - r12*r33) * inv_det;
        inv.m[0][2] = (r12*r23 - r13*r22) * inv_det;
        inv.m[1][0] = (r23*r31 - r21*r33) * inv_det;
        inv.m[1][1] = (r11*r33 - r13*r31) * inv_det;
        inv.m[1][2] = (r13*r21 - r11*r23) * inv_det;
        inv.m[2][0] = (r21*r32 - r22*r31) * inv_det;
        inv.m[2][1] = (r12*r31 - r11*r32) * inv_det;
        inv.m[2][2] = (r11*r22 - r12*r21) * inv_det;
        inv.m[0][3] = -(inv.m[0][0]*tx + inv.m[0][1]*ty + inv.m[0][2]*tz);
        inv.m[1][3] = -(inv.m[1][0]*tx + inv.m[1][1]*ty + inv.m[1][2]*tz);
        inv.m[2][3] = -(inv.m[2][0]*tx + inv.m[2][1]*ty + inv.m[2][2]*tz);
        inv.m[3][0] = 0; inv.m[3][1] = 0; inv.m[3][2] = 0; inv.m[3][3] = 1;
        return inv;
    }
};

/**
 * @brief Geometric ray with origin and unit direction.
 * Provides point evaluation along the parametric t interval.
 */
struct Ray {
    /// Origin point.
    Point3 o;  
    /// Unit direction.
    Dir3 d;    
    /// Default constructor.
    Ray() = default;
    /// Construct with origin and direction (normalized internally).
    Ray(const Point3& oo, const Dir3& dd): o(oo), d(dd.normalized()) {}
    /// Point at parameter t along the ray.
    Point3 at(double t) const { return Point3(o.x + d.x*t, o.y + d.y*t, o.z + d.z*t); }
};

/**
 * @brief Linear RGB color with double channels.
 * Supports addition, scalar multiply, and accumulation for shading.
 */
struct Color {
    /// Red channel.
    double r{}, 
    /// Green channel.
           g{}, 
    /// Blue channel.
           b{};

    /// Default-initialized (black).
    Color() = default;
    /// From components.
    Color(double rr, double gg, double bb): r(rr), g(gg), b(bb) {}
    /// Channel-wise addition.
    Color operator+(const Color& c) const { return {r+c.r, g+c.g, b+c.b}; }
    /// Scalar multiply.
    Color operator*(double s) const { return {r*s, g*s, b*s}; }
    /// In-place add.
    Color& operator+=(const Color& c){ r+=c.r; g+=c.g; b+=c.b; return *this; }
};

/// Channel-wise (Hadamard) color product.
inline Color hadamard(const Color& a, const Color& b){
    return {a.r*b.r, a.g*b.g, a.b*b.b};
}

/// Clamp scalar to [0,1].
inline double clamp01(double v){ return std::max(0.0, std::min(1.0, v)); }

/// Map [0,1] to 8-bit [0,255] with rounding.
inline int toByte(double v){ return (int)std::round(clamp01(v)*255.0); }

#endif
