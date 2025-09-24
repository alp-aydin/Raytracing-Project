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
    if (!scene || !camera || width <= 0 || height <= 0) return;
    
    // Fixed: Better bounds checking and overflow prevention
    const size_t total_pixels = static_cast<size_t>(width) * height;
    framebuffer.assign(total_pixels, Color(0,0,0));

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            const size_t index = static_cast<size_t>(y) * width + x;
            if (index >= total_pixels) continue; // Safety check
            
            Ray pr = camera->generate_ray(x, y);
            framebuffer[index] = trace(pr);
        }
    }
}