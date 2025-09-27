#pragma once
#include "core.h"
#include "scene.h"

/**
 * @brief Lambert + Phong shader with ambient, directional/point lights, and hard shadows.
 * @param scene World state providing lights, ambient, and medium params.
 * @param hit Surface sample (p, n, material) from primary/secondary rays.
 * @param wo Unit view direction from hit toward the eye.
 * @return Estimated linear RGB radiance at the hit.
 */
Color shade_lambert_phong(const Scene& scene, const Hit& hit, const Dir3& wo);

/// Component-wise combine using screen blend (1 - (1-a)*(1-b)).
Color combine(const Color& a, const Color& b);

/// Element-wise (Hadamard) color product.
Color mul(const Color& a, const Color& b);

/// Scale all channels of a color by s.
Color scale(const Color& c, double s);
