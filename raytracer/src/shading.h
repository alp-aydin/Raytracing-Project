#pragma once
#include "core.h"
#include "scene.h"

// Classic Lambert + Blinn-Phong shading with hard shadows.
// wo: unit vector from hit point toward the eye (view direction).
Color shade_lambert_phong(const Scene& scene, const Hit& hit, const Dir3& wo);

// Helper functions for color operations (used by both shading.cpp and tracer.cpp)
Color combine(const Color& a, const Color& b);
Color mul(const Color& a, const Color& b);
Color scale(const Color& c, double s);