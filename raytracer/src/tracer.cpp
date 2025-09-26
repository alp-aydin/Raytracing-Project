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
    if (!scene) return Color(0,0,0);
    Hit h;
    if (scene->intersect(r, 1e-4, kINF, h)) {
        Dir3 wo = (-r.d).normalized();
        return shade_lambert_phong(*scene, h, wo);
    }
    return scene->background;
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
        
        if (nx < 0 || nx >= width || ny < 0 || ny >= height) {
            continue;
        }
        
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
            if (normalDot < 0.2) {
                maxEdge = std::max(maxEdge, 0.5);
            }
            
            if (centerHit.mat != neighborHit.mat && normalDot < 0.7) {
                maxEdge = std::max(maxEdge, 0.3);
            }
        }
    }
    
    if (validNeighbors < 4) {
        maxEdge *= 0.5;
    }
    
    return maxEdge;
}

Color Tracer::apply_crosshatch(const Color& /*baseColor*/, double luminance, int x, int y) const {
    if (luminance < 0.15) {
        return Color(0, 0, 0);
    }
    
    double darkness = 1.0 - luminance;
    
    bool diag1 = ((x + y) % 4) < 1;
    bool diag2 = ((x - y) % 4) < 1;      
    bool horizontal = (y % 4) < 1;
    // Removed unused variables: vertical, fineDiag1, fineDiag2

    bool drawLine = false;

    if (darkness > 0.8) {
        drawLine = (diag1 && diag2) || horizontal;
    } else if (darkness > 0.65) {
        drawLine = (diag1 && diag2) || (horizontal && ((x + y) % 3 == 0));
    } else if (darkness > 0.5) {
        drawLine = (diag1 && diag2) || (horizontal && ((x + y) % 4 == 0));
    } else if (darkness > 0.35) {
        drawLine = diag1 || (horizontal && ((x + y) % 3 == 0));
    } else if (darkness > 0.2) {
        drawLine = diag1;
    } else if (darkness > 0.12) {
        drawLine = diag1 && ((x + y) % 8) < 2;
    }

    return drawLine ? Color(0,0,0) : Color(1,1,1);
}

void Tracer::print_progress(int current, int total, 
                           std::chrono::steady_clock::time_point start_time) const {
    const int barWidth = 50;
    float progress = static_cast<float>(current) / total;
    
    std::cout << "\r[";
    int pos = barWidth * progress;
    for (int i = 0; i < barWidth; ++i) {
        if (i < pos) std::cout << "=";
        else if (i == pos) std::cout << ">";
        else std::cout << " ";
    }
    
    // Calculate time remaining
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(now - start_time).count();
    
    std::cout << "] " << int(progress * 100.0) << "% ";
    
    if (current > 0 && elapsed > 0) {
        float rate = static_cast<float>(current) / elapsed;
        int remaining = (total - current) / rate;
        int minutes = remaining / 60;
        int seconds = remaining % 60;
        
        if (minutes > 0) {
            std::cout << "| ETA: " << minutes << "m " << seconds << "s ";
        } else {
            std::cout << "| ETA: " << seconds << "s ";
        }
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
        std::cout << "Rendering in paper mode (" << nx << "x" << ny << ")" << std::endl;
        
        for (int y = 0; y < ny; ++y) {
            // Update progress every row
            if (y % 10 == 0) {
                print_progress(y * nx, total_pixels, start_time);
            }
            
            for (int x = 0; x < nx; ++x) {
                Ray r = camera->generate_ray(x, y);
                Color baseColor = trace_paper(r);
                
                double luminance = get_luminance(baseColor);
                double edgeStrength = get_edge_strength(x, y);
                
                Color finalColor;
                
                if (edgeStrength > 0.8) {
                    finalColor = Color(0, 0, 0);
                } else if (edgeStrength > 0.5) {
                    finalColor = Color(0.2, 0.2, 0.2);
                } else {
                    finalColor = apply_crosshatch(baseColor, luminance, x, y);
                    
                    if (edgeStrength > 0.3) {
                        double darken = (edgeStrength - 0.3) * 0.4;
                        finalColor.r *= (1.0 - darken);
                        finalColor.g *= (1.0 - darken);
                        finalColor.b *= (1.0 - darken);
                    }
                }

                framebuffer[y * nx + x] = finalColor;
            }
        }
    } else {
        const int spp = 8;
        std::mt19937 rng(12345);
        std::uniform_real_distribution<double> uni(-0.5, 0.5);
        
        std::cout << "Rendering with " << spp << " samples per pixel (" 
                  << nx << "x" << ny << ")" << std::endl;

        for (int y = 0; y < ny; ++y) {
            // Update progress every row
            if (y % 5 == 0) {
                print_progress(y * nx, total_pixels, start_time);
            }
            
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
    
    // Final progress update
    print_progress(total_pixels, total_pixels, start_time);
    std::cout << std::endl << "Rendering complete!" << std::endl;
}