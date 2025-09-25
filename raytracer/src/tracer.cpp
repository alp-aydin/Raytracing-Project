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
        
        // Keep the original shaded color - don't convert to grayscale here
        // We need the full lighting information for proper paper rendering
        return shaded;
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
    
    // Sample center pixel
    Ray center = camera->generate_ray(x, y);
    Hit centerHit;
    bool centerHits = scene->intersect(center, epsilon, kINF, centerHit);
    
    // Only check immediate 4-connected neighbors
    int dx[] = {-1, 1, 0, 0};
    int dy[] = {0, 0, -1, 1};
    
    double maxEdge = 0.0;
    int validNeighbors = 0;
    
    for (int i = 0; i < 4; ++i) {
        int nx = x + dx[i];
        int ny = y + dy[i];
        
        // Skip out-of-bounds neighbors
        if (nx < 0 || nx >= width || ny < 0 || ny >= height) {
            continue;
        }
        
        validNeighbors++;
        Ray neighbor = camera->generate_ray(nx, ny);
        Hit neighborHit;
        bool neighborHits = scene->intersect(neighbor, epsilon, kINF, neighborHit);
        
        // Silhouette detection - most reliable edge type
        if (centerHits != neighborHits) {
            maxEdge = std::max(maxEdge, 0.9);
            continue;
        }
        
        // Both hit something - only check for major discontinuities
        if (centerHits && neighborHits) {
            // Only very significant depth changes
            double minDepth = std::min(centerHit.t, neighborHit.t);
            double maxDepth = std::max(centerHit.t, neighborHit.t);
            if (minDepth > 1e-4 && maxDepth / minDepth > 3.0) {  // Increased threshold
                maxEdge = std::max(maxEdge, 0.6);
            }
            
            // Only very sharp creases (>80 degrees)
            double normalDot = centerHit.n.dot(neighborHit.n);
            if (normalDot < 0.2) {  // More conservative threshold
                maxEdge = std::max(maxEdge, 0.5);
            }
            
            // Material boundaries only with geometric support
            if (centerHit.mat != neighborHit.mat && normalDot < 0.7) {
                maxEdge = std::max(maxEdge, 0.3);
            }
        }
    }
    
    // Reduce edge strength if we don't have enough valid neighbors
    // This helps eliminate artifacts at image borders
    if (validNeighbors < 4) {
        maxEdge *= 0.5;
    }
    
    return maxEdge;
}

Color Tracer::apply_crosshatch(const Color& baseColor, double luminance, int x, int y) const {
    // Handle very dark materials as solid black (no hatching)
    if (luminance < 0.15) {
        return Color(0, 0, 0); // Pitch black for very dark materials
    }
    
    // Convert to inverted luminance for traditional hatching (darker areas need more lines)
    double darkness = 1.0 - luminance;
    
    // Slightly more aggressive - tightened spacing just a bit
    bool diag1 = ((x + y) % 4) < 1;      // Back to 4 from 5 for more density
    bool diag2 = ((x - y) % 4) < 1;      
    bool horizontal = (y % 4) < 1;       // Keep at 4 
    bool vertical = (x % 4) < 1;         

    // Fine patterns for dense areas
    bool fineDiag1 = ((x + y) % 3) < 1;  // Keep at 3
    bool fineDiag2 = ((x - y) % 3) < 1;  

    bool drawLine = false;

    // Slightly more aggressive thresholds
    if (darkness > 0.8) {
        // Very dark areas - dense crosshatch with additional lines
        drawLine = (diag1 && diag2) || horizontal;
    } else if (darkness > 0.65) {
        // Dark areas - crosshatch with occasional additional lines
        drawLine = (diag1 && diag2) || (horizontal && ((x + y) % 3 == 0));
    } else if (darkness > 0.5) {
        // Medium-dark - crosshatch with some horizontal accents
        drawLine = (diag1 && diag2) || (horizontal && ((x + y) % 4 == 0));
    } else if (darkness > 0.35) {
        // Medium - diagonal with horizontal accents
        drawLine = diag1 || (horizontal && ((x + y) % 3 == 0));
    } else if (darkness > 0.2) {
        // Light shadow - single diagonal
        drawLine = diag1;
    } else if (darkness > 0.12) {
        // Very light shadow - sparse diagonal
        drawLine = diag1 && ((x + y) % 8) < 2;  // Slightly denser than before
    }
    // Very light areas remain white

    return drawLine ? Color(0,0,0) : Color(1,1,1);
}

void Tracer::render(std::vector<Color>& framebuffer) const {
    if (!scene || !camera || width <= 0 || height <= 0) return;

    const int nx = width;
    const int ny = height;

    framebuffer.assign(static_cast<size_t>(nx) * ny, Color{0,0,0});

    if (mode == RenderMode::Paper) {
        // Paper mode rendering - simpler approach without excessive sampling
        for (int y = 0; y < ny; ++y) {
            for (int x = 0; x < nx; ++x) {
                // Single ray per pixel for consistency
                Ray r = camera->generate_ray(x, y);
                Color baseColor = trace_paper(r);
                
                double luminance = get_luminance(baseColor);
                double edgeStrength = get_edge_strength(x, y);
                
                Color finalColor;
                
                // Handle edges with clear thresholds
                if (edgeStrength > 0.8) {
                    // Very strong silhouette edges - pure black
                    finalColor = Color(0, 0, 0);
                } else if (edgeStrength > 0.5) {
                    // Strong crease edges - dark gray
                    finalColor = Color(0.2, 0.2, 0.2);
                } else {
                    // No strong edge - apply hatching
                    finalColor = apply_crosshatch(baseColor, luminance, x, y);
                    
                    // Subtle edge enhancement for medium edges
                    if (edgeStrength > 0.3) {
                        double darken = (edgeStrength - 0.3) * 0.4;
                        finalColor.r *= (1.0 - darken);
                        finalColor.g *= (1.0 - darken);
                        finalColor.b *= (1.0 - darken);
                    }
                }

                // Write to framebuffer with y-flip
                framebuffer[y * nx + x] = finalColor;
            }
        }
    } else {
        // Standard mode rendering
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