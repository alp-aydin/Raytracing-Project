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
    
    // Pattern 1: Diagonal lines (45 degrees)
    bool diag1 = ((x + y) % 4) < 1;
    
    // Pattern 2: Diagonal lines (-45 degrees)  
    bool diag2 = ((x - y) % 4) < 1;
    
    // Pattern 3: Horizontal lines
    bool horizontal = (y % 3) < 1;
    
    // Pattern 4: Vertical lines
    bool vertical = (x % 3) < 1;
    
    // Pattern 5: Fine diagonal grid
    bool fineDiag1 = ((x + y) % 2) < 1;
    bool fineDiag2 = ((x - y) % 2) < 1;
    
    // Apply patterns based on darkness level
    bool drawLine = false;
    
    if (luminance > 0.8) {
        // Very dark - use all patterns
        drawLine = diag1 || diag2 || horizontal || vertical;
    } else if (luminance > 0.6) {
        // Dark - use diagonal crosshatch + one direction
        drawLine = (diag1 && diag2) || horizontal;
    } else if (luminance > 0.4) {
        // Medium dark - diagonal crosshatch
        drawLine = diag1 && diag2;
    } else if (luminance > 0.2) {
        // Medium - single diagonal
        drawLine = diag1;
    } else if (luminance > 0.1) {
        // Light - sparse diagonal
        drawLine = diag1 && ((x + y) % 8) < 2;
    }
    // Very light areas (luminance <= 0.1) remain white
    
    if (drawLine) {
        return Color(0, 0, 0); // Black lines
    } else {
        return Color(1, 1, 1); // White paper
    }
}

void Tracer::render(std::vector<Color>& framebuffer) const {
    if (!scene || !camera || width <= 0 || height <= 0) return;

    const int nx = width;
    const int ny = height;

    framebuffer.assign(static_cast<size_t>(nx) * ny, Color{0,0,0});

    if (mode == RenderMode::Paper) {
        // Paper mode rendering
        for (int y = 0; y < ny; ++y) {
            for (int x = 0; x < nx; ++x) {
                Ray r = camera->generate_ray(x, y);
                Color baseColor = trace_paper(r);
                
                double luminance = get_luminance(baseColor);
                double edgeStrength = get_edge_strength(x, y);
                
                Color finalColor;
                
                if (edgeStrength > 0.5) {
                    // Strong edge - draw black line
                    finalColor = Color(0, 0, 0);
                } else if (edgeStrength > 0.2) {
                    // Medium edge - blend with crosshatch
                    Color hatch = apply_crosshatch(baseColor, luminance, x, y);
                    finalColor = Color(
                        hatch.r * (1.0 - edgeStrength),
                        hatch.g * (1.0 - edgeStrength), 
                        hatch.b * (1.0 - edgeStrength)
                    );
                } else {
                    // No edge - pure crosshatch
                    finalColor = apply_crosshatch(baseColor, luminance, x, y);
                }

                // Flip exactly once when writing (row 0 at top)
                framebuffer[y * nx + x] = finalColor;
            }
        }
    } else {
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