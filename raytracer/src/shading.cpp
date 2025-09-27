#include "shading.h"
#include <algorithm>
#include <cmath>

/// Component-wise "screen" blend: 1 - (1-a)*(1-b).
Color combine(const Color& a, const Color& b) {
    return { 
        1.0 - (1.0 - a.r) * (1.0 - b.r),
        1.0 - (1.0 - a.g) * (1.0 - b.g), 
        1.0 - (1.0 - a.b) * (1.0 - b.b)
    };
}

/// Hadamard (element-wise) product for colors.
Color mul(const Color& a, const Color& b) {
    return { a.r * b.r, a.g * b.g, a.b * b.b };
}

/// Scalar multiply color by s.
Color scale(const Color& c, double s) {
    return { c.r * s, c.g * s, c.b * s };
}

/**
 * @brief Lambert + Phong shading with ambient, dir/point lights, and hard shadows.
 * @param scene World containing lights and ambient term.
 * @param hit Surface interaction (p, n, material).
 * @param wo Unit view direction pointing toward the camera.
 * @return Shaded color estimate at the hit.
 */
Color shade_lambert_phong(const Scene& scene, const Hit& hit, const Dir3& wo)
{
    if (!hit.mat) return {1,0,1}; // Magenta for missing material
    
    const Material& m = *hit.mat;
    const Dir3 n = hit.n;

    // 1. Ambient component: E_a = K_a * I_a
    Color E_a = mul(m.ambient, scene.ambient);
    Color E_total = E_a;

    // Much more generous shadow epsilon
    const double shadow_epsilon = std::max(1e-3, 1e-4 * hit.t);

    // 2. Process all directional lights (no distance falloff)
    for (const auto& light : scene.dir_lights) {
        Dir3 wi = (-light.dir).normalized(); // Direction TO light
        double ndotl = std::max(0.0, n.dot(wi));
        
        if (ndotl <= 0.0) continue; // Light behind surface
        
        // Check for shadows
        Point3 shadow_origin = Point3(hit.p.x + n.x * shadow_epsilon,
                                     hit.p.y + n.y * shadow_epsilon,
                                     hit.p.z + n.z * shadow_epsilon);
        Ray shadow_ray{ shadow_origin, wi };
        if (scene.occluded(shadow_ray, shadow_epsilon, kINF)) continue;

        // Diffuse component: E_d = K_d * I_L * (L·N)
        Color E_d = scale(mul(m.albedo, light.radiance), m.kd * ndotl);

        // Specular component
        Color E_s{0,0,0};
        if (m.ks > 0.0) {
            Dir3 r = Dir3(2.0 * n.dot(wi) * n.x - wi.x,
                         2.0 * n.dot(wi) * n.y - wi.y,
                         2.0 * n.dot(wi) * n.z - wi.z).normalized();
            double rdotv = std::max(0.0, r.dot(wo));
            double spec_factor = std::pow(rdotv, m.shininess) * m.ks;
            E_s = scale(light.radiance, spec_factor);
        }

        // Combine diffuse and specular for this light
        Color E_light = combine(E_d, E_s);
        E_total = combine(E_total, E_light);
    }

    // 3. Process all point lights with MUCH more generous lighting
    for (const auto& light : scene.point_lights) {
        Dir3 to_light = Dir3(light.pos.x - hit.p.x, 
                            light.pos.y - hit.p.y, 
                            light.pos.z - hit.p.z);
        double dist_squared = to_light.dot(to_light);
        
        // MUCH more generous minimum distance - prevents weak lighting
        if (dist_squared <= 0.01) {
            dist_squared = 0.01; // Minimum distance for very bright lighting
        }
        
        double distance = std::sqrt(dist_squared);
        Dir3 wi = Dir3(to_light.x / distance, to_light.y / distance, to_light.z / distance);
        double ndotl = std::max(0.0, n.dot(wi));
        
        if (ndotl <= 0.0) continue;

        // More generous shadow checking
        double max_shadow_t = distance - shadow_epsilon;
        if (max_shadow_t <= shadow_epsilon) continue;
        
        Point3 shadow_origin = Point3(hit.p.x + n.x * shadow_epsilon,
                                     hit.p.y + n.y * shadow_epsilon,
                                     hit.p.z + n.z * shadow_epsilon);
        Ray shadow_ray{ shadow_origin, wi };
        if (scene.occluded(shadow_ray, shadow_epsilon, max_shadow_t)) continue;

        // MUCH more generous point light intensity calculation
        // Use a softer falloff that doesn't kill the lighting so much
        double effective_dist = std::max(0.5, distance); // Much more generous minimum
        double falloff = 1.0 / (effective_dist * effective_dist);
        
        // BOOST the lighting significantly
        Color I_L = scale(light.intensity, falloff * 2.0); // 2x boost factor

        // Diffuse component - also boost this
        Color E_d = scale(mul(m.albedo, I_L), m.kd * ndotl * 1.5); // 1.5x boost

        // Specular component
        Color E_s{0,0,0};
        if (m.ks > 0.0) {
            Dir3 r = Dir3(2.0 * n.dot(wi) * n.x - wi.x,
                         2.0 * n.dot(wi) * n.y - wi.y,
                         2.0 * n.dot(wi) * n.z - wi.z).normalized();
            double rdotv = std::max(0.0, r.dot(wo));
            double spec_factor = std::pow(rdotv, m.shininess) * m.ks;
            E_s = scale(I_L, spec_factor);
        }

        Color E_light = combine(E_d, E_s);
        E_total = combine(E_total, E_light);
    }

    // Don't clamp so aggressively - let it get brighter
    E_total.r = std::min(1.5, E_total.r); // Allow some oversaturation
    E_total.g = std::min(1.5, E_total.g);
    E_total.b = std::min(1.5, E_total.b);

    return E_total;
}
