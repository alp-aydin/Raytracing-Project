#include "shading.h"
#include <algorithm>
static inline Color combine(const Color& a, const Color& b){ // ⊕ operator
    return { 1.0 - (1.0 - a.r)*(1.0 - b.r),
             1.0 - (1.0 - a.g)*(1.0 - b.g),
             1.0 - (1.0 - a.b)*(1.0 - b.b) };
}
static inline Color mul(const Color& a, const Color& b){ // Hadamard
    return { a.r*b.r, a.g*b.g, a.b*b.b };
}
Color shade_lambert_phong(const Scene& scene, const Hit& hit, const Dir3& wo)
{
    if (!hit.mat) return {1,0,1};
    const Material& m = *hit.mat;
    const Dir3 n = hit.n;

    // Ambient: approximate Ka by albedo (spec asks E_a = K_a I_a)
    Color Lo = mul(m.albedo, scene.ambient);  // E_a  (part 4) :contentReference[oaicite:9]{index=9}

    // Directional lights
    for (const auto& L : scene.dir_lights) {
        Dir3 wi = (-L.dir).normalized();
        double ndotl = std::max(0.0, n.dot(wi));
        if (ndotl <= 0.0) continue;

        // hard shadow: ray from hit point toward light
        Ray sray{ Point3(hit.p + n * 1e-4), wi };
        if (scene.occluded(sray, 1e-4, kINF)) continue; // blocked

        Color diff = m.albedo * (m.kd * ndotl);
        Dir3 h = (wi + wo).normalized();
        double ndoth = std::max(0.0, n.dot(h));
        double spec = (m.ks>0.0) ? std::pow(ndoth, m.shininess) * m.ks : 0.0;
        Color Li = L.radiance; // dir light as radiance, no falloff
        Lo = combine(Lo, mul(diff + Color(spec,spec,spec), Li)); // ⊕ combine :contentReference[oaicite:10]{index=10}
    }

    // Point lights
    for (const auto& L : scene.point_lights) {
        Dir3 wi = (L.pos - hit.p).normalized();
        double ndotl = std::max(0.0, n.dot(wi));
        if (ndotl <= 0.0) continue;

        // distance & shadow
        double dist2 = (L.pos - hit.p).dot(L.pos - hit.p);
        double dist  = std::sqrt(std::max(0.0, dist2));
        Ray sray{ Point3(hit.p + n * 1e-4), wi };
        if (scene.occluded(sray, 1e-4, dist - 1e-4)) continue;

        Color diff = m.albedo * (m.kd * ndotl);
        Dir3 h = (wi + wo).normalized();
        double ndoth = std::max(0.0, n.dot(h));
        double spec = (m.ks>0.0) ? std::pow(ndoth, m.shininess) * m.ks : 0.0;
        Color Li = L.intensity * (1.0 / std::max(1e-6, dist2));
        Lo = combine(Lo, mul(diff + Color(spec,spec,spec), Li)); // ⊕ combine
    }

    return Lo;
}