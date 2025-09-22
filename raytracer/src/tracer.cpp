#include "tracer.h"
#include "shading.h"
#include <cstddef>

Color Tracer::trace(const Ray& r) const {
    if (!scene) return Color(0,0,0);
    Hit h;
    if (scene->intersect(r, 1e-4, kINF, h)) {
        Dir3 wo = (-r.d).normalized(); // toward eye
        return shade_lambert_phong(*scene, h, wo);
    }
    return scene->background;
}

void Tracer::render(std::vector<Color>& framebuffer) const {
    if (!scene || !camera) return;
    framebuffer.assign(static_cast<std::size_t>(width)*height, Color(0,0,0));

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            Ray pr = camera->makeRay(x, y);
            framebuffer[static_cast<std::size_t>(y)*width + x] = trace(pr);
        }
    }
}
