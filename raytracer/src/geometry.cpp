#include "geometry.h"
#include <cmath>

bool Sphere::intersect(const Ray& ray, double tmin, double tmax, Hit& out) const {
    // Ray-sphere: ||o + t d - c||^2 = r^2
    Vec3   oc      = ray.o - c;
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
    Dir3 outward = Dir3((out.p - c) / this->r);   // normal = (p - c) / r
    out.set_face_normal(ray, outward);
    out.mat = mat;
    return true;
}

bool Sphere::interval(const Ray& ray,
                      double& t0, double& t1,
                      Hit& h0, Hit& h1) const
{
    // Same quadratic as your Sphere::intersect, but keep both roots (t0 <= t1)
    Vec3 oc = ray.o - c;
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
    h0.set_face_normal(ray, Dir3((h0.p - c) / r));
    h0.mat = mat;

    // Fill exit hit (same outward normal at that boundary)
    h1.t = t1;
    h1.p = ray.at(t1);
    h1.set_face_normal(ray, Dir3((h1.p - c) / r));
    h1.mat = mat;

    return true;
}

// ---------------- HalfSpace ----------------
bool HalfSpace::intersect(const Ray& r, double tmin, double tmax, Hit& out) const {
    const double ndotd = n.dot(r.d);
    if (std::abs(ndotd) < 1e-12) return false; // parallel => no finite *surface* hit

    const double t = n.dot(p0 - r.o) / ndotd;
    if (t < tmin || t > tmax) return false;

    Hit h;
    h.t = t;
    h.p = r.at(t);
    h.set_face_normal(r, n);
    h.mat = mat;
    out = h;
    return true;
}

// Inside set is S = { X | n·(X - p0) >= 0 }.
// f(t) = n·(r.o + t r.d - p0) = f0 + t * ndotd.
// ndotd > 0  => inside for t >= tPlane  (enter at tPlane, exit = +∞).
// ndotd < 0  => inside for t <= tPlane  (enter = -∞, exit at tPlane).
// ndotd = 0  => parallel: either everywhere inside (f0>=0) or empty (f0<0).
bool HalfSpace::interval(const Ray& r, double& tEnter, double& tExit,
                         Hit& enterHit, Hit& exitHit) const
{
    const double ndotd = n.dot(r.d);
    const double f0    = n.dot(r.o - p0);

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
