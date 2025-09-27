#pragma once
#include <memory>
#include "core.h"
#include "geometry.h"

/**
 * @brief Transform wrapper applying a 4×4 matrix to a child primitive.
 * Holds forward and inverse matrices; maps rays to local space and hits back to world.
 */
struct Transform : public Primitive {
    /// Wrapped child primitive (owned).
    std::shared_ptr<Primitive> child;
    /// Forward transform from local to world.
    Matrix4 transform_matrix;
    /// Inverse transform from world to local.
    Matrix4 inverse_matrix;
    
    /// Construct from subject and local→world matrix (inverse cached).
    explicit Transform(std::shared_ptr<Primitive> subj, const Matrix4& M) 
        : child(std::move(subj)), transform_matrix(M), inverse_matrix(M.inverse()) {}
    
    /// Virtual destructor.
    ~Transform() override = default;

protected:
    /**
     * @brief Transform a world-space ray into the child's local space.
     * @param r World-space ray.
     * @return Ray expressed in local coordinates.
     */
    Ray transform_ray(const Ray& r) const {
        Vec4 origin_4d = inverse_matrix * Vec4(r.o.x, r.o.y, r.o.z, 1.0);
        Vec4 direction_4d = inverse_matrix * Vec4(r.d.x, r.d.y, r.d.z, 0.0);
        
        return Ray(Point3(origin_4d.x, origin_4d.y, origin_4d.z),
                   Dir3(direction_4d.x, direction_4d.y, direction_4d.z));
    }
    
    /**
     * @brief Transform a local-space point to world space.
     * @param p Point in local coordinates.
     * @return Point in world coordinates.
     */
    Point3 transform_point(const Point3& p) const {
        Vec4 p_4d = transform_matrix * Vec4(p.x, p.y, p.z, 1.0);
        return Point3(p_4d.x, p_4d.y, p_4d.z);
    }
    
    /**
     * @brief Transform a local-space normal to world space (inverse-transpose).
     * @param n Unit normal in local coordinates.
     * @return Unit normal in world coordinates.
     */
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
    
    /**
     * @brief Project a world-space point onto the ray to recover t.
     * @param r World-space ray.
     * @param Pw Point on the ray in world space.
     * @return Parametric t for Pw; +∞ if ray direction is degenerate.
     */
    static inline double project_t_world(const Ray& r, const Point3& Pw) {
        Dir3 v = Dir3(Pw.x - r.o.x, Pw.y - r.o.y, Pw.z - r.o.z);
        const double dd = r.d.x*r.d.x + r.d.y*r.d.y + r.d.z*r.d.z;
        return dd > 0.0 ? (v.x*r.d.x + v.y*r.d.y + v.z*r.d.z) / dd : kINF;
    }
};

/**
 * @brief Translation transform: P' = P + t.
 * Shifts the child without changing orientation or scale.
 */
struct Translation final : public Transform {
    /// Construct with translation vector t = (x,y,z).
    Translation(std::shared_ptr<Primitive> subj, const Vec3& t) 
        : Transform(std::move(subj), Matrix4::translation(t.x, t.y, t.z)) {}
    
    /**
     * @brief Closest-hit test under translation.
     * @param r World-space ray.
     * @param tmin Inclusive lower bound for t.
     * @param tmax Exclusive upper bound for t.
     * @param out Nearest hit mapped back to world.
     * @return true if a hit in (tmin,tmax) exists.
     */
    bool intersect(const Ray& r, double tmin, double tmax, Hit& out) const override;
    /**
     * @brief Entry/exit interval under translation.
     * @param r World-space ray.
     * @param tEnter Entry t.
     * @param tExit Exit t.
     * @param enterHit Boundary hit at entry.
     * @param exitHit Boundary hit at exit.
     * @return true if a valid interval exists.
     */
    bool interval(const Ray& r, double& tEnter, double& tExit, Hit& enterHit, Hit& exitHit) const override;
};

/**
 * @brief Scaling transform: P' = S ∘ P with non-uniform factors.
 * Applies inverse-transpose to normals on return to world space.
 */
struct Scaling final : public Transform {
    /// Construct with scale factors s = (x,y,z).
    Scaling(std::shared_ptr<Primitive> subj, const Vec3& s) 
        : Transform(std::move(subj), Matrix4::scaling(s.x, s.y, s.z)) {}
    
    /**
     * @brief Closest-hit test under scaling.
     * @param r World-space ray.
     * @param tmin Inclusive lower bound for t.
     * @param tmax Exclusive upper bound for t.
     * @param out Nearest hit mapped back with corrected normal.
     * @return true on hit; false on miss/degenerate scale.
     */
    bool intersect(const Ray& r, double tmin, double tmax, Hit& out) const override;
    /**
     * @brief Entry/exit interval under scaling.
     * @param r World-space ray.
     * @param tEnter Entry t.
     * @param tExit Exit t.
     * @param enterHit Boundary hit at entry (normal corrected).
     * @param exitHit Boundary hit at exit (normal corrected).
     * @return true if a valid interval exists.
     */
    bool interval(const Ray& r, double& tEnter, double& tExit, Hit& enterHit, Hit& exitHit) const override;
};

/// Axis selector for rotations.
enum class Axis : int { X=0, Y=1, Z=2 };

/**
 * @brief Rotation transform about principal axis by angle (radians).
 * Uses full 4×4 matrices for ray/point/normal mappings.
 */
struct Rotation final : public Transform {
    /// Construct with axis and angle (radians).
    Rotation(std::shared_ptr<Primitive> subj, Axis ax, double ang) 
        : Transform(std::move(subj), create_rotation_matrix(ax, ang)) {}
    
    /**
     * @brief Closest-hit test under rotation.
     * @param r World-space ray.
     * @param tmin Inclusive lower bound for t.
     * @param tmax Exclusive upper bound for t.
     * @param out Nearest hit mapped back with rotated normal.
     * @return true on hit in (tmin,tmax).
     */
    bool intersect(const Ray& r, double tmin, double tmax, Hit& out) const override;
    /**
     * @brief Entry/exit interval under rotation.
     * @param r World-space ray.
     * @param tEnter Entry t.
     * @param tExit Exit t.
     * @param enterHit Boundary hit at entry (normal rotated).
     * @param exitHit Boundary hit at exit (normal rotated).
     * @return true if a valid interval exists.
     */
    bool interval(const Ray& r, double& tEnter, double& tExit, Hit& enterHit, Hit& exitHit) const override;

private:
    /// Create axis-aligned rotation matrix for (ax, angle).
    static Matrix4 create_rotation_matrix(Axis ax, double angle) {
        switch (ax) {
            case Axis::X: return Matrix4::rotation_x(angle);
            case Axis::Y: return Matrix4::rotation_y(angle);
            case Axis::Z: return Matrix4::rotation_z(angle);
        }
        return Matrix4(); // Identity fallback
    }
};
