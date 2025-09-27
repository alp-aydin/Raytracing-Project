#pragma once
#include <vector>
#include "core.h"
#include "geometry.h"  // defines Primitive and Hit

/**
 * @brief Directional light with uniform radiance from a fixed direction.
 * Used for sun/sky approximations where light arrives parallel.
 */
struct DirectionalLight {
    /// Direction from the light toward the scene (incident direction).
    Dir3  dir = Dir3(0,-1,0);
    /// Emitted radiance (RGB) assumed constant over the image.
    Color radiance{1,1,1};
};

/**
 * @brief Point light emitting equally in all directions.
 * Shaded with simple inverse-square (1/r²) attenuation.
 */
struct PointLight {
    /// Light position in world space.
    Point3 pos{0,0,0};
    /// Source strength (RGB); interpreted with 1/r² falloff.
    Color  intensity{10,10,10};
};

/**
 * @brief Renderable world: geometry, lights, and medium parameters.
 * Stores non-owning object pointers, background/ambient terms, global IOR, and recursion depth.
 */
struct Scene {
    /// Non-owning list of scene primitives (lifetime handled elsewhere).
    std::vector<const Primitive*> objects;
    /// Directional light sources.
    std::vector<DirectionalLight> dir_lights;
    /// Point light sources.
    std::vector<PointLight>       point_lights;
    /// Background color returned on miss.
    Color background{0,0,0};

    /// Global ambient term I_a added to shading.
    Color  ambient{0,0,0};
    /// Medium index of refraction (η) for the environment.
    double medium_index{1.0};
    /// Max recursion depth for reflection/refraction.
    int    recursion_limit{5};

    /**
     * @brief Find the nearest hit along a ray within [tmin, tmax).
     * @param r Input ray.
     * @param tmin Inclusive lower bound for t.
     * @param tmax Exclusive upper bound for t.
     * @param out Filled with closest hit when found.
     * @return true if any object is hit in range; false otherwise.
     */
    bool intersect(const Ray& r, double tmin, double tmax, Hit& out) const;

    /**
     * @brief Shadow/visibility query on (tmin, tmax].
     * @param r Shadow ray from point toward a light.
     * @param tmin Start of tested segment.
     * @param tmax End of tested segment.
     * @return true if an occluder exists; false if path is clear.
     */
    bool occluded(const Ray& r, double tmin, double tmax) const;
};
