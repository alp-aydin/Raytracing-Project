#include "shading.h"

Color shade_lambert_phong(const Scene& scene, const Hit& hit, const Dir3& wo)
{
    if (!hit.mat) return {1,0,1}; // magenta error color

    const Material& m = *hit.mat;
    const Dir3 n = hit.n; // unit
    Color Lo{0,0,0};

    // Directional lights
    for (const auto& L : scene.dir_lights) {
        // Incoming light direction at the point
        Dir3 wi = (-L.dir).normalized();
        double ndotl = std::max(0.0, n.dot(wi));

        // Shadow ray (offset to avoid self-intersection)
        Ray shadow_ray(hit.p + n * 1e-4, wi);
        if (scene.occluded(shadow_ray, 1e-5, kINF)) continue;

        // Diffuse
        Color diff = m.albedo * (m.kd * ndotl);

        // Specular (Blinn-Phong)
        Dir3 h = (wi + wo).normalized();
        double ndoth = std::max(0.0, n.dot(h));
        double spec = (m.ks > 0.0) ? std::pow(ndoth, m.shininess) * m.ks : 0.0;

        Lo += hadamard(diff + Color(spec, spec, spec), L.radiance);
    }

    // Point lights with 1/r^2 falloff
    for (const auto& L : scene.point_lights) {
        Vec3 Lvec = L.pos - hit.p;
        double dist2 = Lvec.dot(Lvec);
        double dist = std::sqrt(dist2);
        Dir3 wi = Dir3(Lvec / dist);
        double ndotl = std::max(0.0, n.dot(wi));

        // Shadow ray up to the light
        Ray shadow_ray(hit.p + n * 1e-4, wi);
        if (scene.occluded(shadow_ray, 1e-5, dist - 1e-4)) continue;

        // Diffuse
        Color diff = m.albedo * (m.kd * ndotl);

        // Specular
        Dir3 h = (wi + wo).normalized();
        double ndoth = std::max(0.0, n.dot(h));
        double spec = (m.ks > 0.0) ? std::pow(ndoth, m.shininess) * m.ks : 0.0;

        Color Li = L.intensity * (1.0 / std::max(1e-6, dist2));
        Lo += hadamard(diff + Color(spec, spec, spec), Li);
    }

    return Lo;
}
