#ifndef CAMERA_H
#define CAMERA_H

#include "core.h"

// Screen defined by:
//  - P : bottom-left point on the image plane
//  - U : full-width edge vector (along +x of screen coords)
//  - V : full-height edge vector (along +y of screen coords)
//  - dpi : pixels per unit along U and V (SAME for both axes, per your spec)
//
// Pixel center (ix, iy), 0-based:
//   Xp = P + ((ix + 0.5) / dpi) * U + ((iy + 0.5) / dpi) * V
//
// Notes:
// - U and V can be arbitrary (supports tilted screens). For axis-aligned in XY plane,
//   set U = {sizeX, 0, 0}, V = {0, sizeY, 0}, and P.z constant.
// - We normalize the primary ray direction here so later code can assume ||d|| = 1.

struct Screen {
    Point3 P;   // bottom-left point on the screen plane
    Dir3   U;   // horizontal edge vector (full width)
    Dir3   V;   // vertical   edge vector (full height)
    int   dpi{1}; // pixels per unit along U and V (must be > 0)

    // Convenience: pixel dimensions implied by geometry
    int width () const { return static_cast<int>(std::round(U.length() * dpi)); }
    int height() const { return static_cast<int>(std::round(V.length() * dpi)); }
};

struct Camera {
    Point3   eye;     // observer position
    Screen screen;  // screen definition

    // World-space point at the CENTER of pixel (ix, iy).
    Point3 pixelCenter(int ix, int iy) const;

    // Primary ray from eye through pixel center; dir is normalized.
    Ray makeRay(int ix, int iy) const;
};

#endif // CAMERA_H
