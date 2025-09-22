#include "csg.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace {
    constexpr double EPS = 1e-6;
    constexpr double INF = std::numeric_limits<double>::infinity();

    enum class EdgeType { Enter, Exit };

    struct Event {
        double t{};
        EdgeType type{};   // Enter or Exit
        int who{};         // 0 = A, 1 = B
        Hit hit{};         // carry full hit (p, n, mat)
        bool operator<(const Event& o) const { return t < o.t; }
    };

    inline bool inside_at_origin(double t0, double t1) {
        // Are we “already inside” at t=0?  (enter < 0 < exit)
        return (t0 < EPS) && (t1 > EPS);
    }

    inline bool combine(bool inA, bool inB, CSGOp op) {
        switch (op) {
            case CSGOp::Union:        return inA || inB;
            case CSGOp::Intersection: return inA && inB;
            case CSGOp::Difference:   return inA && !inB; // A \ B
        }
        return false;
    }

    inline void flip_dir3(Dir3& n) { n = Dir3{-n.x, -n.y, -n.z}; }
}

bool CSG::interval(const Ray& ray,
                   double& tEnter, double& tExit,
                   Hit& enterHit, Hit& exitHit) const
{
    if (!A_ || !B_) return false;

    // Query child intervals
    double ta0{}, ta1{}, tb0{}, tb1{};
    Hit ha0{}, ha1{}, hb0{}, hb1{};
    const bool hitA = A_->interval(ray, ta0, ta1, ha0, ha1);
    const bool hitB = B_->interval(ray, tb0, tb1, hb0, hb1);

    if (!hitA && !hitB) return false;

    // Build boundary events
    std::vector<Event> ev;
    ev.reserve(4);
    if (hitA) {
        ev.push_back(Event{ta0, EdgeType::Enter, 0, ha0});
        ev.push_back(Event{ta1, EdgeType::Exit,  0, ha1});
    }
    if (hitB) {
        ev.push_back(Event{tb0, EdgeType::Enter, 1, hb0});
        ev.push_back(Event{tb1, EdgeType::Exit,  1, hb1});
    }
    std::sort(ev.begin(), ev.end());

    bool inA = hitA && inside_at_origin(ta0, ta1);
    bool inB = hitB && inside_at_origin(tb0, tb1);
    bool inR = combine(inA, inB, op_);

    // Keep a pointer to A's material (used on faces cut by B in A\B)
    const Material* matA = hitA ? ha0.mat : nullptr;

    bool haveEnter = false;
    Hit ent{}, ext{};

    for (const auto& e : ev) {
        const bool wasIn = inR;

        // toggle which child we are inside based on the event
        if (e.who == 0) inA = (e.type == EdgeType::Enter);
        else            inB = (e.type == EdgeType::Enter);

        inR = combine(inA, inB, op_);

        // Outside -> Inside: record entry
        if (!wasIn && inR && e.t >= EPS) {
            haveEnter = true;
            tEnter = e.t;
            ent = e.hit;

            if (op_ == CSGOp::Difference && e.who == 1) {
                // For A\B, boundaries contributed by B must flip normals
                // and use A’s material (exposed cut surface).
                flip_dir3(ent.n);
                if (matA) ent.mat = matA;
            }
        }

        // Inside -> Outside: record exit and finish
        if (wasIn && !inR) {
            tExit = e.t;
            ext = e.hit;

            if (op_ == CSGOp::Difference && e.who == 1) {
                flip_dir3(ext.n);
                if (matA) ext.mat = matA;
            }
            break;
        }
    }

    if (!haveEnter) return false;

    // If we never exited, the interval is open-ended
    if (!(tExit > tEnter + EPS)) {
        tExit = INF;
        ext = ent;
    }

    enterHit = ent;
    exitHit  = ext;
    return true;
}

bool CSG::intersect(const Ray& ray, double tmin, double tmax, Hit& out) const
{
    double t0{}, t1{};
    Hit h0{}, h1{};
    if (!interval(ray, t0, t1, h0, h1)) return false;

    if (t0 >= tmin && t0 <= tmax) { out = h0; return true; }
    if (t1 >= tmin && t1 <= tmax) { out = h1; return true; }
    return false;
}
