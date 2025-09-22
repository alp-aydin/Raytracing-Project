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
