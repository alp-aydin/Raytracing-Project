#pragma once
#include "core.h"

// Screen defined by bottom-left P, sizes (Lx,Ly), and dpi.
// The screen lies in the XY-plane by default (U along +X, V along +Y).
struct ScreenSpec {
    Point3 P{0,0,0};   // bottom-left corner of the screen
    double Lx{1.0};    // width  in scene units
    double Ly{1.0};    // height in scene units
    int    dpi{100};   // dots (pixels) per unit length

    inline int nx() const { return std::max(1, int(std::round(Lx * dpi))); }
    inline int ny() const { return std::max(1, int(std::round(Ly * dpi))); }

    // Precomputed basis vectors for convenience (default XY plane)
    inline Dir3 U() const { return Dir3{Lx, 0.0, 0.0}; }
    inline Dir3 V() const { return Dir3{0.0, Ly, 0.0}; }
};

struct Camera {
    Point3 eye{0,0,1};  // observer C
    ScreenSpec screen;  // physical screen definition

    // Generate primary ray through pixel center (i,j), 0 <= i < nx, 0 <= j < ny.
    inline Ray generate_ray(int i, int j) const {
        const int nx = screen.nx();
        const int ny = screen.ny();

        // normalized pixel coords in [0,1] with center sampling
        const double sx = (double(i) + 0.5) / double(nx);
        const double sy = (double(j) + 0.5) / double(ny);

        // point on screen: S = P + sx*U + sy*V
        const Point3 S = screen.P
                       + screen.U() * sx
                       + screen.V() * sy;

        Dir3 d = (S - eye).normalized();
        return Ray{eye, d};
    }
};
