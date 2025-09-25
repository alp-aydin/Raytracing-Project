#pragma once
#include <vector>
#include "camera.h"
#include "scene.h"

enum class RenderMode {
    Standard,
    Paper
};

struct Tracer {
    const Scene*  scene{nullptr};
    const Camera* camera{nullptr};

    int  width{640};
    int  height{360};
    RenderMode mode{RenderMode::Standard};

    // Renders into a linear RGB buffer (size = width*height).
    void render(std::vector<Color>& framebuffer) const;

    // Single-ray trace returning background if nothing hit.
    Color trace(const Ray& r) const;
    
    // Paper mode specific functions
    Color trace_paper(const Ray& r) const;
    double get_edge_strength(int x, int y) const;
    Color apply_crosshatch(const Color& baseColor, double luminance, int x, int y) const;
    double get_luminance(const Color& c) const;
};