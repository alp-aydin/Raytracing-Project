#pragma once
#include <vector>
#include "camera.h"
#include "scene.h"

struct Tracer {
    const Scene*  scene{nullptr};
    const Camera* camera{nullptr};

    int  width{640};
    int  height{360};

    // Renders into a linear RGB buffer (size = width*height).
    void render(std::vector<Color>& framebuffer) const;

    // Single-ray trace returning background if nothing hit.
    Color trace(const Ray& r) const;
};
