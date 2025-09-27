#pragma once
#include "core.h"

/// Material: surface shading coefficients used by primitives.
struct Material {
    /// Base reflectance used as diffuse color.
    Color  albedo{1,1,1};
    /// Ambient term multiplier.
    Color  ambient{0,0,0};
    /// Diffuse weight in [0,1].
    double kd{1.0};
    /// Specular weight in [0,1].
    double ks{0.0};
    /// Mirror reflection coefficient in [0,1].
    double kr{0.0};
    /// Transmission/refraction coefficient in [0,1].
    double kt{0.0};
    /// Phong shininess exponent (>0).
    double shininess{32.0};
    /// Object-local refractive index (η).
    double refractive_index{1.0};
};

/// Hit: intersection payload for a ray–primitive query.
struct Hit {
    /// Parametric distance along ray (∞ if no hit).
    double t{kINF};
    /// World-space hit position.
    Point3 p;
    /// Geometric normal (unit, oriented against incoming ray if front_face).
    Dir3   n;
    /// Material pointer for shading at the hit.
    const Material* mat{nullptr};
    /// True if the ray hits the front side (n opposed to ray.d).
    bool   front_face{true};

    /**
     * @brief Orient the stored normal to face against the incident ray.
     * @param r Incident ray.
     * @param outward Geometric outward normal (unit).
     */
    inline void set_face_normal(const Ray& r, const Dir3& outward) {
        front_face = r.d.dot(outward) < 0.0;
        n = front_face ? outward : Dir3(-outward.x, -outward.y, -outward.z);
    }
};

/// Primitive: abstract renderable that supports ray queries.
struct Primitive {
    /// Bound material used if the primitive has a single surface look.
    const Material* mat{nullptr};
    virtual ~Primitive() = default;

    /**
     * @brief Closest-hit test in [tmin,tmax].
     * @param r Input ray.
     * @param tmin Minimum valid t.
     * @param tmax Maximum valid t.
     * @param out Filled with hit data if found.
     * @return true if a valid hit exists.
     */
    virtual bool intersect(const Ray& r, double tmin, double tmax, Hit& out) const = 0;

    /**
     * @brief Compute entry/exit t values for CSG-style interval queries.
     * @param ray Input ray.
     * @param tEnter Entry parameter.
     * @param tExit Exit parameter.
     * @param enterHit Hit data at entry boundary.
     * @param exitHit Hit data at exit boundary.
     * @return true if an interval exists.
     */
    virtual bool interval(const Ray& /*ray*/,
                      double& /*tEnter*/, double& /*tExit*/,
                      Hit& /*enterHit*/, Hit& /*exitHit*/) const { return false; }
};

/// Sphere: centered ball primitive with radius r.
struct Sphere : Primitive {
    /// Center in world space.
    Point3 c;
    /// Radius (>0).
    double r{1.0};

    /// Construct with center, radius, and bound material.
    Sphere(const Point3& C, double R, const Material* M){ c=C; r=R; mat=M; }

    /**
     * @brief Ray–sphere closest-hit in [tmin,tmax].
     * @param r Input ray (direction normalized).
     * @param tmin Minimum valid t.
     * @param tmax Maximum valid t.
     * @param out Hit with oriented normal and material.
     * @return true on hit.
     */
    bool intersect(const Ray& r, double tmin, double tmax, Hit& out) const override;

    /**
     * @brief Compute entry/exit parameters and boundary hits.
     * @param r Input ray.
     * @param tEnter Entry t (<= tExit).
     * @param tExit Exit t.
     * @param enterHit Hit at entry boundary.
     * @param exitHit Hit at exit boundary.
     * @return true if the ray intersects (tangent allowed).
     */
    bool interval(const Ray& r, double& tEnter, double& tExit,
              Hit& enterHit, Hit& exitHit) const override;
};

/// HalfSpace: infinite solid { X | n·(X - p0) >= 0 } with planar boundary.
struct HalfSpace : Primitive {
    /// Point on the boundary plane.
    Point3 p0;
    /// Outward unit normal of the kept side.
    Dir3 n;

    /**
     * @brief Construct a half-space; normal is normalized, falls back to +Y if degenerate.
     * @param P0 Point on plane.
     * @param N Outward normal (need not be unit).
     * @param M Bound material.
     */
    HalfSpace(const Point3& P0, const Dir3& N, const Material* M) {
        p0 = P0;
        const double L2 = N.x*N.x + N.y*N.y + N.z*N.z;
        if (L2 > 0.0) {
            const double invL = 1.0 / std::sqrt(L2);
            n = Dir3{ N.x*invL, N.y*invL, N.z*invL };
        } else {
            n = Dir3{0,1,0};
        }
        mat = M;
    }

    /**
     * @brief Intersect the boundary plane within [tmin,tmax].
     * @param r Input ray.
     * @param tmin Minimum valid t.
     * @param tmax Maximum valid t.
     * @param out Hit on the plane if any.
     * @return true if finite surface hit occurs.
     */
    bool intersect(const Ray& r, double tmin, double tmax, Hit& out) const override;

    /**
     * @brief Inside-interval along the ray (can be unbounded).
     * @param r Input ray.
     * @param tEnter Entry t (may be -∞).
     * @param tExit Exit t (may be +∞).
     * @param enterHit Hit data at entry (synthesized if unbounded).
     * @param exitHit Hit data at exit  (synthesized if unbounded).
     * @return true if the ray intersects the kept side.
     */
    bool interval(const Ray& r, double& tEnter, double& tExit,
              Hit& enterHit, Hit& exitHit) const override;

};

/// Pokeball: sphere with region-based materials (top/bottom hemispheres, belt, button/ring).
struct Pokeball : Sphere {
    /// Region material for red top hemisphere.
    Material topMat;
    /// Region material for white bottom hemisphere.
    Material bottomMat;
    /// Region material for equatorial belt.
    Material beltMat;
    /// Ring material around the button annulus.
    Material ringMat;
    /// Button fill material.
    Material buttonMat;

    /// Half-thickness of belt on unit sphere (|y| <= beltHalf).
    double beltHalf = 0.06;
    /// Angular button radius (radians) from button center direction.
    double btnOuter = 0.28;
    /// Ring thickness (radians) around the button.
    double ringWidth = 0.06;
    /// Button center direction in local sphere frame (unit).
    Dir3   btnDir{1,0,0};

    /// Construct Pokeball with default materials; inherits Sphere center/radius.
    Pokeball(const Point3& C, double R)
    : Sphere(C, R, nullptr) {}

    /**
     * @brief Construct Pokeball with explicit region materials and controls.
     * @param C Center.
     * @param R Radius.
     * @param top Material for top hemisphere.
     * @param bottom Material for bottom hemisphere.
     * @param belt Material for belt band.
     * @param ring Material for button ring.
     * @param button Material for button fill.
     * @param belt_half |y| ≤ belt_half defines belt (unit sphere).
     * @param button_outer Angular button radius (radians).
     * @param ring_width Button ring thickness (radians).
     * @param button_dir Direction the button faces (normalized internally).
     */
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

    /**
     * @brief Intersect as a sphere and map the hit to a region material.
     * @param r Input ray.
     * @param tmin Minimum valid t.
     * @param tmax Maximum valid t.
     * @param out Hit with region material assigned.
     * @return true on hit.
     */
    bool intersect(const Ray& r, double tmin, double tmax, Hit& out) const override;

    /**
     * @brief Entry/exit interval with per-boundary region materials.
     * @param ray Input ray.
     * @param tEnter Entry t.
     * @param tExit Exit t.
     * @param enterHit Hit at entry boundary with region material.
     * @param exitHit Hit at exit boundary with region material.
     * @return true if the base sphere interval exists.
     */
    bool interval (const Ray& ray, double& tEnter, double& tExit,
                   Hit& enterHit, Hit& exitHit) const override;

private:
    /**
     * @brief Select region material for a surface point on the sphere.
     * @param p World-space point on the Pokeball surface.
     * @return Region material pointer.
     */
    inline const Material* pick_region_material(const Point3& p) const;
};
