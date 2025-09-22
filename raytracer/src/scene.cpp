#include "scene.h"

bool Scene::intersect(const Ray& r, double tmin, double tmax, Hit& out) const {
    Hit    temp;
    bool   hit_any   = false;
    double closest_t = tmax;

    for (const Primitive* obj : objects) {
        if (!obj) continue;
        if (obj->intersect(r, tmin, closest_t, temp)) {
            hit_any   = true;
            closest_t = temp.t;  // tighten the search window
            out       = temp;    // keep the closest so far
        }
    }
    return hit_any;
}
