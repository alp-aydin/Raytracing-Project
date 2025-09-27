#include "tracer.h"
#include "shading.h"
#include <cstddef>
#include <random>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <iomanip>
#include <chrono>

Color Tracer::trace(const Ray& r) const {
    return trace_recursive(r, 0);
}

Color Tracer::trace_recursive(const Ray& r, int depth) const {
    if (!scene) return Color(0,0,0);

    // Stop condition
    if (depth >= scene->recursion_limit) return Color(0,0,0);

    Hit h;
    if (scene->intersect(r, 1e-4, kINF, h)) {
        Dir3 wo = (-r.d).normalized();            // toward eye
        Color direct = shade_lambert_phong(*scene, h, wo);  // ambient + diffuse + specular (local)
        if (!h.mat) return direct;

        const Material& mat = *h.mat;
        Color total = direct;

        // ----- Perfect mirror reflection -----
        if (mat.kr > 0.0 && depth < scene->recursion_limit - 1) {
            // incident points into surface; use r.d (already normalized in camera)
            Dir3 incident = r.d.normalized();
            Dir3 refl_dir = reflect(incident, h.n).normalized();

            // offset out of the surface along the normal (outward)
            Point3 refl_origin = h.front_face ? (h.p + h.n * 1e-6) : (h.p - h.n * 1e-6);
            Color Ir = trace_recursive(Ray(refl_origin, refl_dir), depth + 1);

            total = combine(total, scale(Ir, mat.kr));   // Er = Kr * Ir
        }

        // ----- Refraction (Snell) with TIR guard -----
        if (mat.kt > 0.0 && depth < scene->recursion_limit - 1) {
            // eta = n_out / n_in when entering; reverse when exiting
            double eta = h.front_face
                ? (scene->medium_index / mat.refractive_index)
                : (mat.refractive_index / scene->medium_index);

            Dir3 incident = r.d.normalized();

            if (!has_total_internal_reflection(incident, h.n, eta)) {
                Dir3 refr_dir = refract(incident, h.n, eta).normalized();

                // offset inside the medium we’re going into
                Point3 refr_origin = h.front_face ? (h.p - h.n * 1e-6) : (h.p + h.n * 1e-6);
                Color It = trace_recursive(Ray(refr_origin, refr_dir), depth + 1);

                total = combine(total, scale(It, mat.kt)); // Et = Kt * It
            }
        }

        return total;
    }
    return scene->background;
}

// R = I - 2(I·N)N, with I pointing into the surface
Dir3 Tracer::reflect(const Dir3& incident, const Dir3& normal) {
    return incident - 2.0 * incident.dot(normal) * normal;
}

Dir3 Tracer::refract(const Dir3& incident, const Dir3& normal, double eta) {
    // I points into surface, N points outward of hit surface
    double cos_i = -incident.dot(normal);
    double sin_t2 = eta * eta * std::max(0.0, 1.0 - cos_i * cos_i);
    if (sin_t2 >= 1.0) {
        // TIR – caller should guard via has_total_internal_reflection()
        return Dir3(0,0,0);
    }
    double cos_t = std::sqrt(1.0 - sin_t2);
    return eta * incident + (eta * cos_i - cos_t) * normal;
}

bool Tracer::has_total_internal_reflection(const Dir3& incident, const Dir3& normal, double eta) {
    double cos_i = -incident.dot(normal);
    double sin_t2 = eta * eta * std::max(0.0, 1.0 - cos_i * cos_i);
    return sin_t2 >= 1.0;
}

Color Tracer::trace_paper(const Ray& r) const {
    if (!scene) return Color(1,1,1);
    Hit h;
    if (scene->intersect(r, 1e-4, kINF, h)) {
        Dir3 wo = (-r.d).normalized();
        Color shaded = shade_lambert_phong(*scene, h, wo);
        return shaded;
    }
    return Color(1,1,1);
}

double Tracer::get_luminance(const Color& c) const {
    return 0.299 * c.r + 0.587 * c.g + 0.114 * c.b;
}

double Tracer::get_edge_strength(int x, int y) const {
    if (!scene || !camera) return 0.0;

    const double epsilon = 1e-4;

    Ray center = camera->generate_ray(x, y);
    Hit centerHit;
    bool centerHits = scene->intersect(center, epsilon, kINF, centerHit);

    int dx[] = {-1, 1, 0, 0};
    int dy[] = {0, 0, -1, 1};

    double maxEdge = 0.0;
    int validNeighbors = 0;

    for (int i = 0; i < 4; ++i) {
        int nx = x + dx[i];
        int ny = y + dy[i];
        if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue;
        validNeighbors++;

        Ray neighbor = camera->generate_ray(nx, ny);
        Hit neighborHit;
        bool neighborHits = scene->intersect(neighbor, epsilon, kINF, neighborHit);

        if (centerHits != neighborHits) {
            maxEdge = std::max(maxEdge, 0.9);
            continue;
        }
        if (centerHits && neighborHits) {
            double minDepth = std::min(centerHit.t, neighborHit.t);
            double maxDepth = std::max(centerHit.t, neighborHit.t);
            if (minDepth > 1e-4 && maxDepth / minDepth > 3.0) {
                maxEdge = std::max(maxEdge, 0.6);
            }
            double normalDot = centerHit.n.dot(neighborHit.n);
            if (normalDot < 0.2) maxEdge = std::max(maxEdge, 0.5);
            if (centerHit.mat != neighborHit.mat && normalDot < 0.7) {
                maxEdge = std::max(maxEdge, 0.3);
            }
        }
    }

    if (validNeighbors < 4) maxEdge *= 0.5;
    return maxEdge;
}

Color Tracer::apply_crosshatch(const Color&, double luminance, int x, int y) const {
    if (luminance < 0.15) return Color(0,0,0);

    double darkness = 1.0 - luminance;
    bool diag1 = ((x + y) % 4) < 1;
    bool diag2 = ((x - y) % 4) < 1;
    bool horizontal = (y % 4) < 1;

    bool drawLine = false;
    if (darkness > 0.8)       drawLine = (diag1 && diag2) || horizontal;
    else if (darkness > 0.65) drawLine = (diag1 && diag2) || (horizontal && ((x + y) % 3 == 0));
    else if (darkness > 0.5)  drawLine = (diag1 && diag2) || (horizontal && ((x + y) % 4 == 0));
    else if (darkness > 0.35) drawLine = diag1 || (horizontal && ((x + y) % 3 == 0));
    else if (darkness > 0.2)  drawLine = diag1;
    else if (darkness > 0.12) drawLine = diag1 && ((x + y) % 8) < 2;

    return drawLine ? Color(0,0,0) : Color(1,1,1);
}

void Tracer::print_progress(int current, int total,
                            std::chrono::steady_clock::time_point start_time) const {
    const int barWidth = 50;
    float progress = static_cast<float>(current) / total;

    std::cout << "\r[";
    int pos = static_cast<int>(barWidth * progress);
    for (int i = 0; i < barWidth; ++i) {
        if (i < pos) std::cout << "=";
        else if (i == pos) std::cout << ">";
        else std::cout << " ";
    }

    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start_time).count();

    std::cout << "] " << int(progress * 100.0) << "% ";

    if (current > 0 && elapsed > 0) {
        float rate = static_cast<float>(current) / elapsed;
        int remaining = static_cast<int>((total - current) / rate);
        int minutes = remaining / 60;
        int seconds = remaining % 60;
        if (minutes > 0) std::cout << "| ETA: " << minutes << "m " << seconds << "s ";
        else std::cout << "| ETA: " << seconds << "s ";
    }
    std::cout << std::flush;
}

void Tracer::render(std::vector<Color>& framebuffer) const {
    if (!scene || !camera || width <= 0 || height <= 0) return;

    const int nx = width;
    const int ny = height;

    framebuffer.assign(static_cast<size_t>(nx) * ny, Color{0,0,0});

    auto start_time = std::chrono::steady_clock::now();
    int total_pixels = nx * ny;

    if (mode == RenderMode::Paper) {
        std::cout << "Rendering in paper mode (" << nx << "x" << ny << ")\n";
        for (int y = 0; y < ny; ++y) {
            if (y % 10 == 0) print_progress(y * nx, total_pixels, start_time);
            for (int x = 0; x < nx; ++x) {
                Ray r = camera->generate_ray(x, y);
                Color base = trace_paper(r);
                double L = get_luminance(base);
                double edge = get_edge_strength(x, y);
                Color out;
                if (edge > 0.8)      out = Color(0,0,0);
                else if (edge > 0.5) out = Color(0.2,0.2,0.2);
                else {
                    out = apply_crosshatch(base, L, x, y);
                    if (edge > 0.3) {
                        double darken = (edge - 0.3) * 0.4;
                        out.r *= (1.0 - darken);
                        out.g *= (1.0 - darken);
                        out.b *= (1.0 - darken);
                    }
                }
                framebuffer[y * nx + x] = out;
            }
        }
    } else {
        const int spp = 8;
        std::mt19937 rng(12345);
        std::uniform_real_distribution<double> uni(-0.5, 0.5);

        std::cout << "Rendering with " << spp << " spp (" << nx << "x" << ny << ")\n";
        for (int y = 0; y < ny; ++y) {
            if (y % 5 == 0) print_progress(y * nx, total_pixels, start_time);
            for (int x = 0; x < nx; ++x) {
                Color acc{0,0,0};
                for (int s = 0; s < spp; ++s) {
                    double dx = uni(rng), dy = uni(rng);
                    acc += trace(camera->generate_ray_subpixel(x, y, dx, dy));
                }
                acc = acc * (1.0 / double(spp));
                const int yy = ny - 1 - y;  // flip once
                framebuffer[yy * nx + x] = acc;
            }
        }
    }

    print_progress(total_pixels, total_pixels, start_time);
    std::cout << "\nRendering complete!\n";
}
