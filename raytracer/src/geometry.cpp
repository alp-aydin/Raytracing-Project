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

