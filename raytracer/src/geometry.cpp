#include "geometry.h"
#include <cmath>

/**
 * @brief Ray–sphere intersection; stores the closest hit in [tmin, tmax].
 * @param ray Input ray (direction assumed normalized).
 * @param tmin Minimum valid ray parameter.
 * @param tmax Maximum valid ray parameter.
 * @param out Filled with t, position, geometric normal (with face orientation), and material.
 * @return true if a valid root lies in [tmin, tmax].
 */
bool Sphere::intersect(const Ray& ray, double tmin, double tmax, Hit& out) const {
    // Ray-sphere: ||o + t d - c||^2 = r^2
    // Using homogeneous coordinates but computing as 3D vectors
    Dir3 oc = Dir3(ray.o.x - c.x, ray.o.y - c.y, ray.o.z - c.z);
    double a       = 1.0;                      // d is normalized in Ray
    double half_b  = oc.dot(ray.d);            // b = 2*half_b
    double cterm   = oc.dot(oc) - this->r * this->r;
    double disc    = half_b*half_b - a*cterm;
    if (disc < 0.0) return false;

    double sqrtD = std::sqrt(disc);

    // try nearer root first
    double t = (-half_b - sqrtD) / a;
    if (t < tmin || t > tmax) {
        t = (-half_b + sqrtD) / a;
        if (t < tmin || t > tmax) return false;
    }

    out.t = t;
    out.p = ray.at(t);
    Dir3 outward = Dir3((out.p.x - c.x) / this->r, (out.p.y - c.y) / this->r, (out.p.z - c.z) / this->r);   // normal = (p - c) / r
    out.set_face_normal(ray, outward);
    out.mat = mat;
    return true;
}

/**
 * @brief Compute entry/exit parameters of a sphere along a ray.
 * @param ray Input ray (direction assumed normalized).
 * @param t0 Output entry t (<= t1).
 * @param t1 Output exit t.
 * @param h0 Entry hit record (position, oriented normal, material).
 * @param h1 Exit hit record (position, oriented normal, material).
 * @return true if the ray intersects (tangent allowed with t0==t1).
 */
bool Sphere::interval(const Ray& ray,
                      double& t0, double& t1,
                      Hit& h0, Hit& h1) const
{
    // Same quadratic as Sphere::intersect, but keep both roots (t0 <= t1)
    Dir3 oc = Dir3(ray.o.x - c.x, ray.o.y - c.y, ray.o.z - c.z);
    double a = 1.0;
    double half_b = oc.dot(ray.d);
    double cterm = oc.dot(oc) - r*r;
    double disc = half_b*half_b - a*cterm;
    if (disc < 0.0) return false;

    double s = std::sqrt(disc);
    t0 = (-half_b - s) / a;
    t1 = (-half_b + s) / a;
    if (t0 > t1) std::swap(t0, t1);

    // Fill entry hit (outward normal)
    h0.t = t0;
    h0.p = ray.at(t0);
    h0.set_face_normal(ray, Dir3((h0.p.x - c.x) / r, (h0.p.y - c.y) / r, (h0.p.z - c.z) / r));
    h0.mat = mat;

    // Fill exit hit (same outward normal at that boundary)
    h1.t = t1;
    h1.p = ray.at(t1);
    h1.set_face_normal(ray, Dir3((h1.p.x - c.x) / r, (h1.p.y - c.y) / r, (h1.p.z - c.z) / r));
    h1.mat = mat;

    return true;
}

// ---------------- HalfSpace ----------------

/**
 * @brief Intersect the plane boundary of a half-space; returns finite surface hits only.
 * @param r Input ray.
 * @param tmin Minimum valid ray parameter.
 * @param tmax Maximum valid ray parameter.
 * @param out Hit at the plane if within [tmin, tmax] (with oriented normal and material).
 * @return true if the boundary plane is hit; false for parallel rays or out-of-range t.
 */
bool HalfSpace::intersect(const Ray& r, double tmin, double tmax, Hit& out) const {
    const double ndotd = n.dot(r.d);
    if (std::abs(ndotd) < 1e-12) return false; // parallel => no finite *surface* hit

    // Computing n·(p0 - r.o) using homogeneous coordinates
    Dir3 diff = Dir3(p0.x - r.o.x, p0.y - r.o.y, p0.z - r.o.z);
    const double t = n.dot(diff) / ndotd;
    if (t < tmin || t > tmax) return false;

    Hit h;
    h.t = t;
    h.p = r.at(t);
    h.set_face_normal(r, n);
    h.mat = mat;
    out = h;
    return true;
}

/**
 * @brief Compute the inside-interval of a half-space along a ray (may be unbounded).
 * @param r Input ray.
 * @param tEnter Entry t (can be -∞).
 * @param tExit Exit t (can be +∞).
 * @param enterHit Hit representation at entry boundary (synthesized when unbounded).
 * @param exitHit Hit representation at exit boundary (synthesized when unbounded).
 * @return true if the ray intersects the inside set S = {X | n·(X - p0) >= 0}.
 */
bool HalfSpace::interval(const Ray& r, double& tEnter, double& tExit,
                         Hit& enterHit, Hit& exitHit) const
{
    const double ndotd = n.dot(r.d);
    Dir3 diff = Dir3(r.o.x - p0.x, r.o.y - p0.y, r.o.z - p0.z);
    const double f0    = n.dot(diff);

    if (std::abs(ndotd) < 1e-12) {
        if (f0 >= 0.0) {
            tEnter = -kINF; tExit =  kINF;
            enterHit.t = tEnter; enterHit.p = r.o; enterHit.set_face_normal(r, n); enterHit.mat = mat;
            exitHit .t = tExit ; exitHit .p = r.o; exitHit .set_face_normal(r, n); exitHit .mat = mat;
            return true;
        }
        return false; // entirely outside
    }

    const double tPlane = -f0 / ndotd;

    if (ndotd > 0.0) {
        tEnter = tPlane; tExit = kINF;
        enterHit.t = tEnter; enterHit.p = r.at(tEnter); enterHit.set_face_normal(r, n); enterHit.mat = mat;
        exitHit .t = tExit ; exitHit .p = r.o;           exitHit .set_face_normal(r, n); exitHit .mat = mat;
        return true;
    } else {
        tEnter = -kINF; tExit = tPlane;
        enterHit.t = tEnter; enterHit.p = r.o;           enterHit.set_face_normal(r, n); enterHit.mat = mat;
        exitHit .t = tExit ; exitHit .p = r.at(tExit );  exitHit .set_face_normal(r, n); exitHit .mat = mat;
        return true;
    }
}

// ---------------- Pokeball ----------------

/// Clamp a scalar to [-1, 1] to avoid acos domain errors.
static inline double clamp1(double x){
    if (x < -1.0) return -1.0;
    if (x >  1.0) return  1.0;
    return x;
}

/**
 * @brief Select region material (top, bottom, belt, button/ring) for a surface point.
 * @param p World-space point on the sphere surface.
 * @return Material assigned to that region for shading.
 */
const Material* Pokeball::pick_region_material(const Point3& p) const {
    // Local unit position on the sphere
    Dir3 u = Dir3((p.x - this->c.x) / this->r, (p.y - this->c.y) / this->r, (p.z - this->c.z) / this->r);      // unit normal / local direction

    // 1) Button (on equator, centered at +btnDir). Wins over belt.
    const double ang = std::acos(clamp1(u.dot(btnDir)));   // angle to button center
    const double inner = std::max(0.0, btnOuter - ringWidth);
    if (ang <= btnOuter) {
        if (ang >= inner) return &ringMat;  // annulus
        return &buttonMat;                  // button fill
    }

    // 2) Belt (around the equator in XZ-plane): |y| small on unit sphere
    if (std::abs(u.y) <= beltHalf) return &beltMat;

    // 3) Hemispheres (Y up is red top, Y down is white bottom)
    return (u.y >= 0.0) ? &topMat : &bottomMat;
}

/**
 * @brief Intersect as a sphere and assign material by Poké Ball region.
 * @param ray Input ray.
 * @param tmin Minimum valid ray parameter.
 * @param tmax Maximum valid ray parameter.
 * @param out Filled with hit data; material overridden by region mapping.
 * @return true if the base sphere is hit within [tmin, tmax].
 */
bool Pokeball::intersect(const Ray& ray, double tmin, double tmax, Hit& out) const {
    // Use the base sphere intersection, then recolor by region.
    if (!Sphere::intersect(ray, tmin, tmax, out)) return false;
    out.mat = pick_region_material(out.p);
    // normal is already set by Sphere::intersect; keep as-is
    return true;
}

/**
 * @brief Entry/exit interval for Poké Ball with per-boundary region materials.
 * @param ray Input ray.
 * @param t0 Output entry t (<= t1).
 * @param t1 Output exit t.
 * @param h0 Entry hit with region material.
 * @param h1 Exit hit with region material.
 * @return true if the base sphere interval exists.
 */
bool Pokeball::interval(const Ray& ray, double& t0, double& t1,
                        Hit& h0, Hit& h1) const
{
    if (!Sphere::interval(ray, t0, t1, h0, h1)) return false;

    // Assign region materials for the entry and exit boundary hits.
    // (This makes it play nicely with CSG.)
    h0.mat = pick_region_material(h0.p);
    h1.mat = pick_region_material(h1.p);
    return true;
}
