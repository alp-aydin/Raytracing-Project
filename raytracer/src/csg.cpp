#include "csg.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

/**
 * @brief CSG boolean operations along a ray using a boundary-event sweep.
 * Defines local helpers: tolerances, event types/structs, ordering,
 * origin-inside test, boolean combiner, and normal flip.
 */
namespace {
    /// Tolerance for comparisons.
    constexpr double EPS = 1e-6;
    /// Positive infinity shortcut.
    constexpr double INF = std::numeric_limits<double>::infinity();

    /// Enter/Exit tags for boundary events.
    enum class EdgeType { Enter, Exit };

    /// Boundary event along the ray with owning child and hit payload.
    struct Event {
        double t{};
        EdgeType type{};
        int who{};   // 0 = A, 1 = B
        Hit hit{};
    };

    /// Sort by t, then Enter before Exit, then A before B.
    inline bool event_less(const Event& a, const Event& b) {
        if (a.t < b.t - EPS) return true;
        if (a.t > b.t + EPS) return false;
        if (a.type != b.type) return a.type == EdgeType::Enter;
        return a.who < b.who;
    }

    /// Inside if enter before 0 and exit after 0.
    inline bool inside_at_origin(double t0, double t1) {
        return (t0 < EPS) && (t1 > EPS);
    }

    /// Boolean combination for A and B.
    inline bool combine(bool inA, bool inB, CSGOp op) {
        switch (op) {
            case CSGOp::Union:        return inA || inB;
            case CSGOp::Intersection: return inA && inB;
            case CSGOp::Difference:   return inA && !inB;
        }
        return false;
    }

    /// Flip a normal direction.
    inline void flip_dir3(Dir3& n) { n = Dir3{-n.x, -n.y, -n.z}; }
}

/**
 * @brief Compute the first inside interval [tEnter, tExit] for this CSG node.
 * Returns false if empty; on success fills enter/exit times and hits.
 */
bool CSG::interval(const Ray& ray,
                   double& tEnter, double& tExit,
                   Hit& enterHit, Hit& exitHit) const
{
    if (!A_ || !B_) return false;

    // Child intervals
    double ta0{}, ta1{}, tb0{}, tb1{};
    Hit ha0{}, ha1{}, hb0{}, hb1{};
    const bool hitA = A_->interval(ray, ta0, ta1, ha0, ha1);
    const bool hitB = B_->interval(ray, tb0, tb1, hb0, hb1);
    if (!hitA && !hitB) return false;

    // Build boundary events (finite only)
    struct Ev { double t; EdgeType type; int who; Hit h; };
    auto add = [](std::vector<Ev>& v, double t, EdgeType type, int who, const Hit& h){
        if (std::isfinite(t)) v.push_back(Ev{t, type, who, h});
    };
    std::vector<Ev> ev; ev.reserve(4);
    if (hitA) { add(ev, ta0, EdgeType::Enter, 0, ha0); add(ev, ta1, EdgeType::Exit, 0, ha1); }
    if (hitB) { add(ev, tb0, EdgeType::Enter, 1, hb0); add(ev, tb1, EdgeType::Exit, 1, hb1); }

    auto event_less = [](const Ev& a, const Ev& b){
        if (std::abs(a.t - b.t) > 1e-6) return a.t < b.t;
        // On ties: process ENTERS before EXITS; if still tied, A before B
        if (a.type != b.type) return a.type == EdgeType::Enter;
        return a.who < b.who;
    };
    std::sort(ev.begin(), ev.end(), event_less);

    // State at t=0 (before first boundary)
    auto inside_at_origin = [](double t0, double t1){
        return (t0 < 1e-6) && (t1 > 1e-6);
    };
    bool inA = hitA && inside_at_origin(ta0, ta1);
    bool inB = hitB && inside_at_origin(tb0, tb1);

    auto combine = [&](bool a, bool b){
        switch (op_) {
            case CSGOp::Union:        return a || b;
            case CSGOp::Intersection: return a && b;
            case CSGOp::Difference:   return a && !b;  // A \ B
        }
        return false;
    };

    bool inR = combine(inA, inB);

    bool haveEnter = false;
    Hit  hEnter{}, hExit{};

    // If we already start inside, mark an entry at t=0 (renderer tmin should skip it)
    double tEnt = 0.0, tExt = INF;
    if (inR) {
        haveEnter = true;
        tEnt = 0.0;
        hEnter.t = 0.0;
        hEnter.p = ray.o;
        hEnter.n = Dir3{0,0,0};
        hEnter.mat = (inA && ha0.mat) ? ha0.mat : (inB ? hb0.mat : nullptr);
        hEnter.front_face = true;
    }

    // Sweep over boundaries
    for (const auto& e : ev) {
        const bool before = inR;

        // flip the child state that this event belongs to
        if (e.who == 0) inA = (e.type == EdgeType::Enter) ? true : false;
        else            inB = (e.type == EdgeType::Enter) ? true : false;

        const bool after = combine(inA, inB);

        if (!before && after) {
            // We are ENTERING the result at this boundary
            haveEnter = true;
            tEnt = e.t;
            hEnter = e.h;
            // For A\B entering via B's EXIT (who=1, Exit) flip normal
            if (op_ == CSGOp::Difference && e.who == 1 && e.type == EdgeType::Exit) {
                hEnter.set_face_normal(ray, Dir3{-e.h.n.x, -e.h.n.y, -e.h.n.z});
            }
        } else if (before && !after) {
            // We are EXITING the result at this boundary
            tExt = e.t;
            hExit = e.h;
            // For A\B exiting via B's ENTER (who=1, Enter) flip normal
            if (op_ == CSGOp::Difference && e.who == 1 && e.type == EdgeType::Enter) {
                hExit.set_face_normal(ray, Dir3{-e.h.n.x, -e.h.n.y, -e.h.n.z});
            }
            // First full interval is enough
            break;
        }

        inR = after;
    }

    if (!haveEnter || !std::isfinite(tExt)) return false;

    tEnter = tEnt; tExit = tExt;
    enterHit = hEnter; exitHit = hExit;
    return true;
}

/**
 * @brief First-hit query on the CSG solid within [tmin, tmax].
 * On success writes hit data at the earliest valid t and returns true.
 */
bool CSG::intersect(const Ray& r, double tmin, double tmax, Hit& out) const {
    double tEnter, tExit;
    Hit hEnter, hExit;
    if (!interval(r, tEnter, tExit, hEnter, hExit)) return false;

    // choose first time inside the solid
    const double t = std::max(tEnter, tmin);
    if (!(t < tExit && t < tmax)) return false;

    // use the enter hit's material/normal; clamp t and recompute point
    out = hEnter;
    out.t = t;
    out.p = Point3{ r.o.x + r.d.x * t,
                    r.o.y + r.d.y * t,
                    r.o.z + r.d.z * t };
    return true;
}
