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
    };

    // Sort by t; for ties process Enter before Exit; then A before B (deterministic).
    inline bool event_less(const Event& a, const Event& b) {
        if (a.t < b.t - EPS) return true;
        if (a.t > b.t + EPS) return false;
        if (a.type != b.type) return a.type == EdgeType::Enter;
        return a.who < b.who;
    }

    inline bool inside_at_origin(double t0, double t1) {
        // "inside at t=0" if enter < 0 < exit. Works with +/-INF.
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

    // Build boundary events (skip infinite sentinels; they only indicate openness)
    std::vector<Event> ev;
    ev.reserve(4);
    if (hitA) {
        if (std::isfinite(ta0)) ev.push_back(Event{ta0, EdgeType::Enter, 0, ha0});
        if (std::isfinite(ta1)) ev.push_back(Event{ta1, EdgeType::Exit,  0, ha1});
    }
    if (hitB) {
        if (std::isfinite(tb0)) ev.push_back(Event{tb0, EdgeType::Enter, 1, hb0});
        if (std::isfinite(tb1)) ev.push_back(Event{tb1, EdgeType::Exit,  1, hb1});
    }
    std::sort(ev.begin(), ev.end(), event_less);

    bool inA = hitA && inside_at_origin(ta0, ta1);
    bool inB = hitB && inside_at_origin(tb0, tb1);
    bool inR = combine(inA, inB, op_);

    // Keep a pointer to A's material (used on faces cut by B in A\B)
    const Material* matA = hitA ? ha0.mat : nullptr;

    bool haveEnter = false;
    Hit ent{}, ext{};

    // If we already start inside the result at t=0, the interval begins at 0.
    // This ensures callers (like intersect) can still pick the first EXIT as the visible hit.
    if (inR) {
        haveEnter = true;
        tEnter = 0.0;
        ent.t = 0.0;
        ent.p = ray.o;              // sentinel; renderer will usually skip t=0 with a tmin epsilon
        ent.n = Dir3{0,0,0};
        ent.mat = matA ? matA : (hitA ? ha0.mat : (hitB ? hb0.mat : nullptr));
    }

    for (const auto& e : ev) {
        const bool wasIn = inR;

        // Update inA/inB from this boundary
        if (e.who == 0) inA = (e.type == EdgeType::Enter);
        else            inB = (e.type == EdgeType::Enter);

        const bool nowIn = combine(inA, inB, op_);

        // Outside -> Inside: record entry (ignore very negative crossings)
        if (!wasIn && nowIn && e.t >= -EPS) {
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
        if (wasIn && !nowIn) {
            tExit = e.t;
            ext = e.hit;

            if (op_ == CSGOp::Difference && e.who == 1) {
                flip_dir3(ext.n);
                if (matA) ext.mat = matA;
            }
            inR = nowIn;
            break;
        }

        inR = nowIn;
    }

    if (!haveEnter) return false;

    // If we started inside and never saw a boundary, or if the last boundary didn't close it, it's open-ended.
    if (!(tExit > tEnter + EPS)) {
        tExit = INF;
        // ext is not meaningful here; leave as-is (callers normally only care about tExit)
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
