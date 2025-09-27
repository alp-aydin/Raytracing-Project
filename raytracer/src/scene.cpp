#include "scene.h"

/**
 * @brief Closest-hit search over all scene objects.
 * @param r Ray to test.
 * @param tmin Minimum acceptable t.
 * @param tmax Maximum acceptable t.
 * @return true if any object is hit; @p out holds the nearest hit in [tmin,tmax].
 */
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

/**
 * @brief Shadow/visibility test: does any object block the segment?
 * @param r Shadow ray.
 * @param tmin Start of the tested interval.
 * @param tmax End of the tested interval.
 * @return true if any intersection exists in (tmin,tmax]; false otherwise.
 */
bool Scene::occluded(const Ray& r, double tmin, double tmax) const {
    Hit tmp;
    for (const Primitive* obj : objects) {
        if (!obj) continue;
        if (obj->intersect(r, tmin, tmax, tmp)) {
            return true;
        }
    }
    return false;
}
