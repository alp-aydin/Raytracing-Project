#pragma once
#include <memory>
#include "core.h"
#include "geometry.h"

// Transform base wraps a child primitive and applies a 4x4 matrix transformation.
struct Transform : public Primitive {
    std::shared_ptr<Primitive> child;
    Matrix4 transform_matrix;
    Matrix4 inverse_matrix;
    
    explicit Transform(std::shared_ptr<Primitive> subj, const Matrix4& M) 
        : child(std::move(subj)), transform_matrix(M), inverse_matrix(M.inverse()) {}
    
    ~Transform() override = default;

protected:
    // Transform ray from world space to local space
    Ray transform_ray(const Ray& r) const {
        Vec4 origin_4d = inverse_matrix * Vec4(r.o.x, r.o.y, r.o.z, 1.0);
        Vec4 direction_4d = inverse_matrix * Vec4(r.d.x, r.d.y, r.d.z, 0.0);
        
        return Ray(Point3(origin_4d.x, origin_4d.y, origin_4d.z),
                   Dir3(direction_4d.x, direction_4d.y, direction_4d.z));
    }
    
    // Transform point from local space to world space
    Point3 transform_point(const Point3& p) const {
        Vec4 p_4d = transform_matrix * Vec4(p.x, p.y, p.z, 1.0);
        return Point3(p_4d.x, p_4d.y, p_4d.z);
    }
    
    // Transform normal from local space to world space
    Dir3 transform_normal(const Dir3& n) const {
        // For normals, we use the transpose of the inverse of the 3x3 part
        // This is equivalent to using the inverse transpose matrix
        Matrix4 normal_transform = inverse_matrix; // We'll transpose the 3x3 part
        
        // Apply inverse transpose transformation for normals
        Vec4 n_4d = Vec4(
            normal_transform.m[0][0]*n.x + normal_transform.m[1][0]*n.y + normal_transform.m[2][0]*n.z,
            normal_transform.m[0][1]*n.x + normal_transform.m[1][1]*n.y + normal_transform.m[2][1]*n.z,
            normal_transform.m[0][2]*n.x + normal_transform.m[1][2]*n.y + normal_transform.m[2][2]*n.z,
            0.0
        );
        
        return Dir3(n_4d.x, n_4d.y, n_4d.z).normalized();
    }
    
    static inline double project_t_world(const Ray& r, const Point3& Pw) {
        Dir3 v = Dir3(Pw.x - r.o.x, Pw.y - r.o.y, Pw.z - r.o.z);
        const double dd = r.d.x*r.d.x + r.d.y*r.d.y + r.d.z*r.d.z;
        return dd > 0.0 ? (v.x*r.d.x + v.y*r.d.y + v.z*r.d.z) / dd : kINF;
    }
};

struct Translation final : public Transform {
    Translation(std::shared_ptr<Primitive> subj, const Vec3& t) 
        : Transform(std::move(subj), Matrix4::translation(t.x, t.y, t.z)) {}
    
    bool intersect(const Ray& r, double tmin, double tmax, Hit& out) const override;
    bool interval(const Ray& r, double& tEnter, double& tExit, Hit& enterHit, Hit& exitHit) const override;
};

struct Scaling final : public Transform {
    Scaling(std::shared_ptr<Primitive> subj, const Vec3& s) 
        : Transform(std::move(subj), Matrix4::scaling(s.x, s.y, s.z)) {}
    
    bool intersect(const Ray& r, double tmin, double tmax, Hit& out) const override;
    bool interval(const Ray& r, double& tEnter, double& tExit, Hit& enterHit, Hit& exitHit) const override;
};

enum class Axis : int { X=0, Y=1, Z=2 };

struct Rotation final : public Transform {
    Rotation(std::shared_ptr<Primitive> subj, Axis ax, double ang) 
        : Transform(std::move(subj), create_rotation_matrix(ax, ang)) {}
    
    bool intersect(const Ray& r, double tmin, double tmax, Hit& out) const override;
    bool interval(const Ray& r, double& tEnter, double& tExit, Hit& enterHit, Hit& exitHit) const override;

private:
    static Matrix4 create_rotation_matrix(Axis ax, double angle) {
        switch (ax) {
            case Axis::X: return Matrix4::rotation_x(angle);
            case Axis::Y: return Matrix4::rotation_y(angle);
            case Axis::Z: return Matrix4::rotation_z(angle);
        }
        return Matrix4(); // Identity fallback
    }
};