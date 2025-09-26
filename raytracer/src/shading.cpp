#include "shading.h"
#include <algorithm>

// Fixed: Use proper additive lighting instead of screen blend
static inline Color combine(const Color& a, const Color& b) {
    return { 
        std::min(1.0, a.r + b.r),
        std::min(1.0, a.g + b.g), 
        std::min(1.0, a.b + b.b)
    };
}

static inline Color mul(const Color& a, const Color& b) {
    return { a.r*b.r, a.g*b.g, a.b*b.b };
}

Color shade_lambert_phong(const Scene& scene, const Hit& hit, const Dir3& wo)
{
    if (!hit.mat) return {1,0,1};
    const Material& m = *hit.mat;
    const Dir3 n = hit.n;

    // Ambient: E_a = K_a * I_a
    Color Lo = mul(m.ambient, scene.ambient);

    // Scale-aware shadow epsilon
    const double shadow_epsilon = std::max(1e-4, 1e-6 * hit.t);

    // Directional lights (no falloff)
    for (const auto& L : scene.dir_lights) {
        Dir3 wi = (-L.dir).normalized();
        double ndotl = std::max(0.0, n.dot(wi));
        if (ndotl <= 0.0) continue;

        Ray sray{ Point3(hit.p + n * shadow_epsilon), wi };
        if (scene.occluded(sray, shadow_epsilon, kINF)) continue;

        Color diff = m.albedo * (m.kd * ndotl);
        Dir3 h = (wi + wo).normalized();
        double ndoth = std::max(0.0, n.dot(h));
        double spec = (m.ks > 0.0) ? std::pow(ndoth, m.shininess) * m.ks : 0.0;

        Color Li = L.radiance; // directional light as radiance
        Lo = combine(Lo, mul(diff + Color(spec, spec, spec), Li));
    }

    // Point lights (1/r^2 falloff)
    for (const auto& L : scene.point_lights) {
        Dir3 toL = L.pos - hit.p;
        double dist2 = toL.dot(toL);
        if (dist2 <= 0.0) continue;

        Dir3 wi = toL / std::sqrt(dist2);
        double ndotl = std::max(0.0, n.dot(wi));
        if (ndotl <= 0.0) continue;

        // Shadow ray only up to the light
        double max_t = std::sqrt(dist2) - shadow_epsilon;
        if (max_t <= 0.0) continue;

        Ray sray{ Point3(hit.p + n * shadow_epsilon), wi };
        if (scene.occluded(sray, shadow_epsilon, max_t)) continue;

        Color diff = m.albedo * (m.kd * ndotl);
        Dir3 h = (wi + wo).normalized();
        double ndoth = std::max(0.0, n.dot(h));
        double spec = (m.ks > 0.0) ? std::pow(ndoth, m.shininess) * m.ks : 0.0;

        // Treat intensity as point-light power; convert to irradiance with 1/r^2
        Color Li = L.intensity * (1.0 / std::max(1e-9, dist2));
        Lo = combine(Lo, mul(diff + Color(spec, spec, spec), Li));
    }

    return Lo;
}
