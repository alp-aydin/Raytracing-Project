#include "tracer.h"
#include "shading.h"
#include <cstddef>
#include <random>
#include <cmath>
#include <algorithm>

Color Tracer::trace(const Ray& r) const {
    if (!scene) return Color(0,0,0);
    Hit h;
    if (scene->intersect(r, 1e-4, kINF, h)) {
        Dir3 wo = (-r.d).normalized(); // toward eye
        return shade_lambert_phong(*scene, h, wo);
    }
    return scene->background;
}

Color Tracer::trace_paper(const Ray& r) const {
    if (!scene) return Color(1,1,1); // white paper background
    
    Hit h;
    if (scene->intersect(r, 1e-4, kINF, h)) {
        Dir3 wo = (-r.d).normalized();
        Color shaded = shade_lambert_phong(*scene, h, wo);
        
        // Convert to grayscale for paper effect
        double luminance = get_luminance(shaded);
        
        // Invert for paper effect (darker areas get darker lines)
        luminance = 1.0 - luminance;
        
        return Color(luminance, luminance, luminance);
    }
    return Color(1,1,1); // white paper background
}

double Tracer::get_luminance(const Color& c) const {
    // Standard luminance formula
    return 0.299 * c.r + 0.587 * c.g + 0.114 * c.b;
}

double Tracer::get_edge_strength(int x, int y) const {
    if (!scene || !camera) return 0.0;
    
    const double epsilon = 1e-4;
    
    // Sample center and neighbors
    Ray center = camera->generate_ray(x, y);
    Hit centerHit;
    bool centerHits = scene->intersect(center, epsilon, kINF, centerHit);
    
    double maxDiff = 0.0;
    
    // Check 4-connected neighbors
    int dx[] = {-1, 1, 0, 0};
    int dy[] = {0, 0, -1, 1};
    
    for (int i = 0; i < 4; ++i) {
        int nx = x + dx[i];
        int ny = y + dy[i];
        
        if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
            Ray neighbor = camera->generate_ray(nx, ny);
            Hit neighborHit;
            bool neighborHits = scene->intersect(neighbor, epsilon, kINF, neighborHit);
            
            if (centerHits != neighborHits) {
                // Silhouette edge (object/background boundary)
                maxDiff = std::max(maxDiff, 1.0);
            } else if (centerHits && neighborHits) {
                // Check depth discontinuity
                double depthDiff = std::abs(centerHit.t - neighborHit.t);
                if (depthDiff > 0.1) {
                    maxDiff = std::max(maxDiff, 0.8);
                }
                
                // Check normal discontinuity  
                double normalDiff = 1.0 - centerHit.n.dot(neighborHit.n);
                if (normalDiff > 0.3) {
                    maxDiff = std::max(maxDiff, 0.6);
                }
                
                // Check material discontinuity
                if (centerHit.mat != neighborHit.mat) {
                    maxDiff = std::max(maxDiff, 0.4);
                }
            }
        }
    }
    
    return maxDiff;
}

Color Tracer::apply_crosshatch(const Color& baseColor, double luminance, int x, int y) const {
    // Multiple crosshatch patterns for different density levels

    // Base patterns
    bool diag1 = ((x + y) % 4) < 1;
    bool diag2 = ((x - y) % 4) < 1;
    bool horizontal = (y % 3) < 1;
    bool vertical   = (x % 3) < 1;

    // Fine patterns
    bool fineDiag1 = ((x + y) % 2) < 1;
    bool fineDiag2 = ((x - y) % 2) < 1;

    bool drawLine = false;

    // NEW: super-dense tier (for black materials -> luminance forced to 1.0)
    if (luminance >= 0.95) {
        // tighten the base grids for more coverage
        bool diag1_tight = ((x + y) % 3) < 1;
        bool diag2_tight = ((x - y) % 3) < 1;
        bool horiz_tight = (y % 2) < 1;
        bool vert_tight  = (x % 2) < 1;

        drawLine = diag1_tight || diag2_tight || horiz_tight || vert_tight || fineDiag1 || fineDiag2;
    }
    else if (luminance > 0.8) {
        drawLine = diag1 || diag2 || horizontal || vertical;
    } else if (luminance > 0.6) {
        drawLine = (diag1 && diag2) || horizontal;
    } else if (luminance > 0.4) {
        drawLine = diag1 && diag2;
    } else if (luminance > 0.2) {
        drawLine = diag1;
    } else if (luminance > 0.1) {
        drawLine = diag1 && ((x + y) % 8) < 2;
    }

    return drawLine ? Color(0,0,0) : Color(1,1,1);
}


void Tracer::render(std::vector<Color>& framebuffer) const {
    if (!scene || !camera || width <= 0 || height <= 0) return;

    const int nx = width;
    const int ny = height;

    framebuffer.assign(static_cast<size_t>(nx) * ny, Color{0,0,0});

    if (mode == RenderMode::Paper) {
    const int spp = 8; // match standard
    std::mt19937 rng(12345);
    std::uniform_real_distribution<double> uni(-0.5, 0.5);

    for (int y = 0; y < ny; ++y) {
        for (int x = 0; x < nx; ++x) {

            double luminance_acc = 0.0;
            double edge_acc = 0.0;

            for (int s = 0; s < spp; ++s) {
                const double dx = uni(rng);
                const double dy = uni(rng);

                // base tone
                Ray r = camera->generate_ray_subpixel(x, y, dx, dy);
                Color baseColor = trace_paper(r);
                luminance_acc += get_luminance(baseColor);

                // edge using same jitter (sample neighbors with same dx,dy)
                auto neighbor_hit = [&](int px, int py, Hit& out, bool& ok){
                    if (px < 0 || px >= nx || py < 0 || py >= ny) { ok = false; return; }
                    Ray rn = camera->generate_ray_subpixel(px, py, dx, dy);
                    ok = scene->intersect(rn, 1e-4, kINF, out);
                };

                Hit cH; bool cOk = false; neighbor_hit(x, y, cH, cOk);
                double e = 0.0;
                if (cOk) {
                    static const int DX[4] = {-1, 1, 0, 0};
                    static const int DY[4] = {0, 0, -1, 1};
                    double maxDiff = 0.0;
                    for (int k = 0; k < 4; ++k) {
                        Hit nH; bool nOk = false;
                        neighbor_hit(x + DX[k], y + DY[k], nH, nOk);
                        if (cOk != nOk) {
                            maxDiff = std::max(maxDiff, 1.0);               // silhouette
                        } else if (cOk && nOk) {
                            double depthDiff  = std::abs(cH.t - nH.t);
                            double normalDiff = 1.0 - cH.n.dot(nH.n);
                            if (depthDiff  > 0.1) maxDiff = std::max(maxDiff, 0.8);
                            if (normalDiff > 0.3) maxDiff = std::max(maxDiff, 0.6);
                            if (cH.mat != nH.mat) maxDiff = std::max(maxDiff, 0.4);
                        }
                    }
                    e = maxDiff;
                }
                edge_acc += e;
            }

            const double luminance = luminance_acc / double(spp);
            const double edgeStrength = edge_acc / double(spp);

            Color finalColor;
            if (edgeStrength > 0.5) {
                finalColor = Color(0,0,0);
            } else if (edgeStrength > 0.2) {
                Color hatch = apply_crosshatch(Color(0,0,0), luminance, x, y);
                finalColor = Color(
                    hatch.r * (1.0 - edgeStrength),
                    hatch.g * (1.0 - edgeStrength),
                    hatch.b * (1.0 - edgeStrength)
                );
            } else {
                finalColor = apply_crosshatch(Color(0,0,0), luminance, x, y);
            }
            const int yy = ny - 1 - y;
            framebuffer[yy * nx + x] = finalColor;
        }
    }
}
 
    else {
        // Standard mode rendering (existing code)
        const int spp = 8;
        std::mt19937 rng(12345);
        std::uniform_real_distribution<double> uni(-0.5, 0.5);

        for (int y = 0; y < ny; ++y) {
            for (int x = 0; x < nx; ++x) {
                Color acc{0,0,0};
                for (int s = 0; s < spp; ++s) {
                    const double dx = uni(rng);
                    const double dy = uni(rng);
                    Ray r = camera->generate_ray_subpixel(x, y, dx, dy);
                    acc += trace(r);
                }
                acc = acc * (1.0 / double(spp));

                const int yy = ny - 1 - y;
                framebuffer[yy * nx + x] = acc;
            }
        }
    }
}