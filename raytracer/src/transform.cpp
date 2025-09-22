#include "transform.h"
#include <cmath>

static inline void sincosd(double a, double& s, double& c){ s = std::sin(a); c = std::cos(a); }

static inline Dir3 rot_apply_dir(const Dir3& v, Axis ax, double ang) {
    double s, c; sincosd(ang, s, c);
    switch (ax) {
        case Axis::X: return Dir3{ v.x, c*v.y - s*v.z, s*v.y + c*v.z };
        case Axis::Y: return Dir3{  c*v.x + s*v.z, v.y, -s*v.x + c*v.z };
        case Axis::Z: return Dir3{  c*v.x - s*v.y, s*v.x + c*v.y, v.z };
    }
    return v;
}
static inline Dir3  rot_apply_dir_inv(const Dir3& v, Axis ax, double ang) { return rot_apply_dir(v, ax, -ang); }
static inline Point3 rot_apply_pt(const Point3& p, Axis ax, double ang) {
    // rotate as if it were a direction, since rotation doesn’t use translation
    Dir3 v{p.x, p.y, p.z};
    Dir3 r = rot_apply_dir(v, ax, ang);
    return Point3{r.x, r.y, r.z};
}
static inline Point3 rot_apply_pt_inv (const Point3& p, Axis ax, double ang) { return rot_apply_pt(p, ax, -ang); }

// -------- Translation --------
bool Translation::intersect(const Ray& r, double tmin, double tmax, Hit& out) const {
    if (!child) return false;
    Ray rl{ Point3{ r.o.x - t.x, r.o.y - t.y, r.o.z - t.z }, r.d };
    Hit h;
    if (!child->intersect(rl, 0.0, kINF, h)) return false;
    Point3 Pw{ h.p.x + t.x, h.p.y + t.y, h.p.z + t.z };
    Dir3  Nw = h.n;
    double tw = project_t_world(r, Pw);
    if (!(tw > tmin && tw < tmax)) return false;
    out = h; out.p = Pw; out.set_face_normal(r, Nw); out.t = tw;
    return true;
}
bool Translation::interval(const Ray& r, double& tEnter, double& tExit, Hit& enterHit, Hit& exitHit) const {
    if (!child) return false;
    Ray rl{ Point3{ r.o.x - t.x, r.o.y - t.y, r.o.z - t.z }, r.d };
    bool ok = child->interval(rl, tEnter, tExit, enterHit, exitHit);
    if (!ok) return false;
    enterHit.p = Point3{ enterHit.p.x + t.x, enterHit.p.y + t.y, enterHit.p.z + t.z };
    exitHit.p  = Point3{  exitHit.p.x + t.x,  exitHit.p.y + t.y,  exitHit.p.z + t.z };
    enterHit.set_face_normal(r, enterHit.n);
    exitHit .set_face_normal(r, exitHit.n);
    tEnter = project_t_world(r, enterHit.p);
    tExit  = project_t_world(r, exitHit.p);
    return true;
}

// -------- Scaling --------
bool Scaling::intersect(const Ray& r, double tmin, double tmax, Hit& out) const {
    if (!child) return false;
    if (std::abs(s.x) < kEPS || std::abs(s.y) < kEPS || std::abs(s.z) < kEPS) return false;
    Ray rl{
        Point3{ r.o.x / s.x, r.o.y / s.y, r.o.z / s.z },
        Dir3  { r.d.x / s.x, r.d.y / s.y, r.d.z / s.z }
    };
    Hit h;
    if (!child->intersect(rl, 0.0, kINF, h)) return false;
    auto mapP = [&](const Point3& p)->Point3{ return Point3{ p.x * s.x, p.y * s.y, p.z * s.z }; };
    auto mapN = [&](const Dir3& n)->Dir3{
        Dir3 w{ n.x / s.x, n.y / s.y, n.z / s.z };
        double L = std::sqrt(w.x*w.x + w.y*w.y + w.z*w.z);
        if (L > 0.0){ w.x/=L; w.y/=L; w.z/=L; }
        return w;
    };
    Point3 Pw = mapP(h.p);
    Dir3  Nw  = mapN(h.n);
    double tw = project_t_world(r, Pw);
    if (!(tw > tmin && tw < tmax)) return false;
    out = h; out.p = Pw; out.set_face_normal(r, Nw); out.t = tw;
    return true;
}
bool Scaling::interval(const Ray& r, double& tEnter, double& tExit, Hit& enterHit, Hit& exitHit) const {
    if (!child) return false;
    if (std::abs(s.x) < kEPS || std::abs(s.y) < kEPS || std::abs(s.z) < kEPS) return false;
    Ray rl{
        Point3{ r.o.x / s.x, r.o.y / s.y, r.o.z / s.z },
        Dir3  { r.d.x / s.x, r.d.y / s.y, r.d.z / s.z }
    };
    bool ok = child->interval(rl, tEnter, tExit, enterHit, exitHit);
    if (!ok) return false;
    auto mapP = [&](const Point3& p)->Point3{ return Point3{ p.x * s.x, p.y * s.y, p.z * s.z }; };
    auto mapN = [&](const Dir3& n)->Dir3{
        Dir3 w{ n.x / s.x, n.y / s.y, n.z / s.z };
        double L = std::sqrt(w.x*w.x + w.y*w.y + w.z*w.z);
        if (L > 0.0){ w.x/=L; w.y/=L; w.z/=L; }
        return w;
    };
    enterHit.p = mapP(enterHit.p);
    exitHit.p  = mapP(exitHit.p);
    enterHit.set_face_normal(r, mapN(enterHit.n));
    exitHit .set_face_normal(r, mapN(exitHit.n));
    tEnter = project_t_world(r, enterHit.p);
    tExit  = project_t_world(r, exitHit.p);
    return true;
}

// -------- Rotation --------
bool Rotation::intersect(const Ray& r, double tmin, double tmax, Hit& out) const {
    if (!child) return false;
    Ray rl{
        rot_apply_pt_inv(r.o, axis, angle),
        rot_apply_dir_inv(r.d, axis, angle)
    };
    Hit h;
    if (!child->intersect(rl, 0.0, kINF, h)) return false;
    Point3 Pw = rot_apply_pt(h.p, axis, angle);
    Dir3  Nw  = rot_apply_dir(h.n, axis, angle);
    double tw = project_t_world(r, Pw);
    if (!(tw > tmin && tw < tmax)) return false;
    out = h; out.p = Pw; out.set_face_normal(r, Nw); out.t = tw;
    return true;
}
bool Rotation::interval(const Ray& r, double& tEnter, double& tExit, Hit& enterHit, Hit& exitHit) const {
    if (!child) return false;
    Ray rl{
        rot_apply_pt_inv(r.o, axis, angle),
        rot_apply_dir_inv(r.d, axis, angle)
    };
    bool ok = child->interval(rl, tEnter, tExit, enterHit, exitHit);
    if (!ok) return false;
    enterHit.p = rot_apply_pt(enterHit.p, axis, angle);
    exitHit.p  = rot_apply_pt(exitHit.p,  axis, angle);
    enterHit.set_face_normal(r, rot_apply_dir(enterHit.n, axis, angle));
    exitHit .set_face_normal(r, rot_apply_dir(exitHit.n,  axis, angle));
    tEnter = project_t_world(r, enterHit.p);
    tExit  = project_t_world(r, exitHit.p);
    return true;
}
