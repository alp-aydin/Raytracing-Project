#pragma once
#include <vector>
#include "core.h"
#include "geometry.h"  // defines Primitive and Hit

struct Scene {
    std::vector<const Primitive*> objects;  // Scene does NOT own these

    // Find nearest hit in [tmin, tmax); returns true if anything was hit.
    bool intersect(const Ray& r, double tmin, double tmax, Hit& out) const;
};
