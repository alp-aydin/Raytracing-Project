#pragma once
#include <memory>
#include "core.h"
#include "geometry.h"

// Transform base wraps a child primitive and applies an affine transform.
struct Transform : public Primitive {
    std::shared_ptr<Primitive> child;
    explicit Transform(std::shared_ptr<Primitive> subj) : child(std::move(subj)) {}
    ~Transform() override = default;
protected:
    static inline double project_t_world(const Ray& r, const Point3& Pw) {
        Dir3 v{Pw.x - r.o.x, Pw.y - r.o.y, Pw.z - r.o.z};
        const double dd = r.d.x*r.d.x + r.d.y*r.d.y + r.d.z*r.d.z;
        return dd > 0.0 ? (v.x*r.d.x + v.y*r.d.y + v.z*r.d.z) / dd : kINF;
    }
};

struct Translation final : public Transform {
    Vec3 t;
    Translation(std::shared_ptr<Primitive> subj, const Vec3& tt) : Transform(std::move(subj)), t(tt) {}
    bool intersect(const Ray& r, double tmin, double tmax, Hit& out) const override;
    bool interval (const Ray& r, double& tEnter, double& tExit, Hit& enterHit, Hit& exitHit) const override;
};

struct Scaling final : public Transform {
    Vec3 s;
    Scaling(std::shared_ptr<Primitive> subj, const Vec3& ss) : Transform(std::move(subj)), s(ss) {}
    bool intersect(const Ray& r, double tmin, double tmax, Hit& out) const override;
    bool interval (const Ray& r, double& tEnter, double& tExit, Hit& enterHit, Hit& exitHit) const override;
};

enum class Axis : int { X=0, Y=1, Z=2 };

struct Rotation final : public Transform {
    Axis axis;
    double angle; // radians
    Rotation(std::shared_ptr<Primitive> subj, Axis ax, double ang) : Transform(std::move(subj)), axis(ax), angle(ang) {}
    bool intersect(const Ray& r, double tmin, double tmax, Hit& out) const override;
    bool interval (const Ray& r, double& tEnter, double& tExit, Hit& enterHit, Hit& exitHit) const override;
};
