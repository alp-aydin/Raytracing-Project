#pragma once
#include "core.h"

/**
 * @brief Physical screen defined by bottom-left P, size (Lx,Ly), and dpi.
 * Describes image plane resolution and axes used to place pixel centers.
 */
struct ScreenSpec {
    /// Bottom-left corner of the screen in world space.
    Point3 P{0,0,0};
    /// Horizontal length of the screen in world units.
    double Lx{1.0};
    /// Vertical length of the screen in world units.
    double Ly{1.0};
    /// Dots per inch used to derive pixel resolution.
    int    dpi{72};

    /// Pixel count in x (>=1).
    inline int nx() const { return std::max(1, int(std::round(Lx * dpi))); }
    /// Pixel count in y (>=1).
    inline int ny() const { return std::max(1, int(std::round(Ly * dpi))); }
    /// Screen x-axis vector (rightward in world units).
    inline Dir3 U() const { return Dir3{Lx, 0.0, 0.0}; }
    /// Screen y-axis vector (upward in world units).
    inline Dir3 V() const { return Dir3{0.0, Ly, 0.0}; }
};

/**
 * @brief Pinhole camera with eye and a physical screen.
 * Generates primary rays through pixel/sample positions on ScreenSpec.
 */
struct Camera {
    /// Camera origin (eye) in world space.
    Point3 eye{0,0,1};
    /// Screen specification used to map pixels to world.
    ScreenSpec screen;

    /**
     * @brief Generate a primary ray through pixel (i,j).
     * @param i Pixel x index in [0, nx-1].
     * @param j Pixel y index in [0, ny-1] (origin at bottom; internally flipped).
     * @return Ray from eye through the pixel center; defaults to forward if out of range.
     */
    inline Ray generate_ray(int i, int j) const {
        const int nx = screen.nx();
        const int ny = screen.ny();
        if (i < 0 || i >= nx || j < 0 || j >= ny) {
            return Ray{eye, Dir3{0,0,-1}};
        }
        const double sx = (double(i) + 0.5) / double(nx);
        const int jf = ny - 1 - j;
        const double sy = (double(jf) + 0.5) / double(ny);
        const Point3 S = Point3(screen.P.x + screen.U().x * sx + screen.V().x * sy,
                                screen.P.y + screen.U().y * sx + screen.V().y * sy,
                                screen.P.z + screen.U().z * sx + screen.V().z * sy);
        Dir3 d = Dir3(S.x - eye.x, S.y - eye.y, S.z - eye.z).normalized();
        return Ray{eye, d};
    }

    /**
     * @brief Generate a ray through pixel (i,j) with subpixel jitter (dx,dy).
     * @param i Pixel x index.
     * @param j Pixel y index.
     * @param dx Jitter in x in [-0.5, 0.5] pixels (caller’s convention).
     * @param dy Jitter in y in [-0.5, 0.5] pixels (caller’s convention).
     * @return Ray from eye through the jittered subpixel sample.
     */
    inline Ray generate_ray_subpixel(int i, int j, double dx, double dy) const {
        const int nx = screen.nx();
        const int ny = screen.ny();
        const double sx = (i + 0.5 + dx) / double(nx);
        const double sy = (j + 0.5 + dy) / double(ny);
        const Point3 S = Point3(screen.P.x + screen.U().x * sx + screen.V().x * sy,
                                screen.P.y + screen.U().y * sx + screen.V().y * sy,
                                screen.P.z + screen.U().z * sx + screen.V().z * sy);
        Dir3 d = Dir3(S.x - eye.x, S.y - eye.y, S.z - eye.z).normalized();
        return Ray{eye, d};
    }
};
