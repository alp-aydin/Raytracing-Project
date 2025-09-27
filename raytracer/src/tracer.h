#pragma once
#include <chrono>
#include <vector>
#include "camera.h"
#include "scene.h"

/// RenderMode: selects the shading style (physically shaded vs paper/hatching).
enum class RenderMode {
    Standard,
    Paper
};

/**
 * @brief CPU ray tracer that renders a Scene from a Camera.
 * Owns no resources; consumes scene/camera, produces a linear RGB framebuffer,
 * and supports standard and paper modes with recursive reflection/refraction.
 */
struct Tracer {
    /// World to trace rays against (non-owning).
    const Scene*  scene{nullptr};
    /// Camera providing ray generation (non-owning).
    const Camera* camera{nullptr};

    /// Output width in pixels.
    int  width{640};
    /// Output height in pixels.
    int  height{360};
    /// Selected rendering mode.
    RenderMode mode{RenderMode::Standard};

    /**
     * @brief Render the image into a linear RGB buffer.
     * @param framebuffer Output buffer, resized to width*height (row-major).
     */
    void render(std::vector<Color>& framebuffer) const;

    /// Trace a single primary/secondary ray; returns background on miss.
    Color trace(const Ray& r) const;
    
    /**
     * @brief Evaluate radiance recursively with depth limit.
     * @param r Input ray.
     * @param depth Current recursion depth (0-based).
     * @return Linear RGB radiance for ray r.
     */
    Color trace_recursive(const Ray& r, int depth) const;
    
    /**
     * @brief Paper-mode primary evaluation (no recursive effects).
     * @param r Camera ray.
     * @return Shaded color for paper mode.
     */
    Color trace_paper(const Ray& r) const;

    /// Edge heuristic in [0,1] using neighbor depth/normal/material cues.
    double get_edge_strength(int x, int y) const;

    /// Crosshatch decision for a pixel given luminance and coordinates.
    Color apply_crosshatch(const Color& baseColor, double luminance, int x, int y) const;

    /// Perceptual luminance estimate from linear RGB.
    double get_luminance(const Color& c) const;

    /**
     * @brief Print a console progress bar with ETA.
     * @param current Completed units (e.g., processed pixels).
     * @param total Total units.
     * @param start_time Start timestamp for ETA calculation.
     */
    void print_progress(int current, int total, 
                   std::chrono::steady_clock::time_point start_time) const;

private:
    /// Perfect-mirror reflection: R = I - 2(I·N)N.
    static Dir3 reflect(const Dir3& incident, const Dir3& normal);
    /// Snell refraction through interface; uses eta = n_out/n_in.
    static Dir3 refract(const Dir3& incident, const Dir3& normal, double eta);
    /// Test if eta produces total internal reflection for (I,N).
    static bool has_total_internal_reflection(const Dir3& incident, const Dir3& normal, double eta);
};
