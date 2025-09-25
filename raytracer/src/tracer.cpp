#include "tracer.h"
#include "shading.h"
#include <cstddef>
#include <random>

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

    const int nx = width;
    const int ny = height;

    framebuffer.assign(static_cast<size_t>(nx) * ny, Color{0,0,0});

    const int spp = 8;  // try 4/8/16
    std::mt19937 rng(12345);
    std::uniform_real_distribution<double> uni(-0.5, 0.5); // jitter around center

    for (int y = 0; y < ny; ++y) {
        for (int x = 0; x < nx; ++x) {
            Color acc{0,0,0};
            for (int s = 0; s < spp; ++s) {
                const double dx = uni(rng);
                const double dy = uni(rng);
                Ray r = camera->generate_ray_subpixel(x, y, dx, dy); // no flips in camera
                acc += trace(r);
            }
            acc = acc * (1.0 / double(spp));

            // Flip exactly once when writing (row 0 at top)
            const int yy = ny - 1 - y;
            framebuffer[yy * nx + x] = acc;
        }
    }
}
