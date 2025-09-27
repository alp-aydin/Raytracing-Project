#pragma once
#include "core.h"

// Enhanced material struct to support full specification
struct Material {
    Color  albedo{1,1,1};        // Primary surface color (from diffuse)
    Color  ambient{0,0,0};       // Ambient reflection coefficient
    double kd{1.0};              // Diffuse weight
    double ks{0.0};              // Specular weight
    double kr{0.0};              // Reflection coefficient (from reflected)
    double kt{0.0};              // Transmission coefficient (from refracted)
    double shininess{32.0};      // Phong exponent
    double refractive_index{1.0}; // Per-object refractive index
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
    virtual bool interval(const Ray& /*ray*/,
                      double& /*tEnter*/, double& /*tExit*/,
                      Hit& /*enterHit*/, Hit& /*exitHit*/) const { return false; }
};

// Sphere
struct Sphere : Primitive {
    Point3 c; double r{1.0};
    Sphere(const Point3& C, double R, const Material* M){ c=C; r=R; mat=M; }
    bool intersect(const Ray& r, double tmin, double tmax, Hit& out) const override;
    bool interval(const Ray& r, double& tEnter, double& tExit,
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
    bool interval(const Ray& r, double& tEnter, double& tExit,
              Hit& enterHit, Hit& exitHit) const override;

};

struct Pokeball : Sphere {
    // Region materials (stored by value; out.mat points to these)
    Material topMat;     // red hemisphere
    Material bottomMat;  // white hemisphere
    Material beltMat;    // dark belt
    Material ringMat;    // dark ring around the button
    Material buttonMat;  // light button fill

    // Controls
    double beltHalf = 0.06;      // |y| <= beltHalf  => belt (on unit sphere)
    double btnOuter = 0.28;      // angular radius (radians) of full button
    double ringWidth = 0.06;     // ring thickness (radians)
    Dir3   btnDir{1,0,0};        // button faces +X by default

    // Constructors
    Pokeball(const Point3& C, double R)
    : Sphere(C, R, nullptr) {}

    Pokeball(const Point3& C, double R,
             const Material& top, const Material& bottom,
             const Material& belt, const Material& ring,
             const Material& button,
             double belt_half = 0.06,
             double button_outer = 0.28,
             double ring_width = 0.06,
             const Dir3& button_dir = Dir3(1,0,0))
    : Sphere(C, R, nullptr),
      topMat(top), bottomMat(bottom), beltMat(belt), ringMat(ring), buttonMat(button),
      beltHalf(belt_half), btnOuter(button_outer), ringWidth(ring_width),
      btnDir(button_dir.normalized()) {}

    bool intersect(const Ray& r, double tmin, double tmax, Hit& out) const override;
    bool interval (const Ray& ray, double& tEnter, double& tExit,
                   Hit& enterHit, Hit& exitHit) const override;

private:
    inline const Material* pick_region_material(const Point3& p) const;
};