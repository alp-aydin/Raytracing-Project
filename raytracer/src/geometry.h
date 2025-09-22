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
};

// Sphere
struct Sphere : Primitive {
    Point3 c; double r{1.0};
    Sphere(const Point3& C, double R, const Material* M){ c=C; r=R; mat=M; }
    bool intersect(const Ray& r, double tmin, double tmax, Hit& out) const override;
};
