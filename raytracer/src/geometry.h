#pragma once
#include "core.h"

// Simple material (enough to start shading later)
struct Material {
    Color  albedo{1,1,1};
    double kd{1.0};          // diffuse weight
    double ks{0.0};          // specular weight
    double shininess{32.0};
};

// Intersection payload
struct Hit {
    double t{kINF};
    Point3 p;                // hit point
    Dir3   n;                // geometric normal (unit)
    const Material* mat{nullptr};
    bool   front_face{true};

    inline void set_face_normal(const Ray& r, const Dir3& outward) {
        front_face = r.d.dot(outward) < 0.0;
        n = front_face ? outward : Dir3(-outward.x, -outward.y, -outward.z);
    }
};

// Abstract primitive
struct Primitive {
    const Material* mat{nullptr};
    virtual ~Primitive() = default;
    virtual bool intersect(const Ray& r, double tmin, double tmax, Hit& out) const = 0;
    virtual bool interval(const Ray& ray,
                      double& tEnter, double& tExit,
                      Hit& enterHit, Hit& exitHit) const { return false; }
};

// Sphere
struct Sphere : Primitive {
    Point3 c; double r{1.0};
    Sphere(const Point3& C, double R, const Material* M){ c=C; r=R; mat=M; }
    bool intersect(const Ray& r, double tmin, double tmax, Hit& out) const override;
    bool interval(const Ray& ray,
                  double& tEnter, double& tExit,
                  Hit& enterHit, Hit& exitHit) const override;
};

// Half-space defined by plane point p0 and outward unit normal n (keeps the side n·(X-p0) >= 0)
struct HalfSpace : Primitive {
    Point3 p0; Dir3 n;
    HalfSpace(const Point3& P0, const Dir3& N, const Material* M) {
        p0 = P0;
        const double L2 = N.x*N.x + N.y*N.y + N.z*N.z;
        if (L2 > 0.0) {
            const double invL = 1.0 / std::sqrt(L2);
            n = Dir3{ N.x*invL, N.y*invL, N.z*invL };
        } else {
            // Prevent division by zero if a bad normal sneaks in
            n = Dir3{0,1,0};
        }
        mat = M;
    }
    bool intersect(const Ray& r, double tmin, double tmax, Hit& out) const override;
    bool interval(const Ray&, double&, double&, Hit&, Hit&) const override;
};


