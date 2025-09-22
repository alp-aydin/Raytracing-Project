#pragma once
#include "core.h"
#include "scene.h"

// Classic Lambert + Blinn-Phong shading with hard shadows.
// wo: unit vector from hit point toward the eye (view direction).
Color shade_lambert_phong(const Scene& scene, const Hit& hit, const Dir3& wo);
