#include "camera.h"

static inline double px_step(int i, int dpi) {
    // (i + 0.5) / dpi  — step in *units* along the corresponding edge vector
    return (static_cast<double>(i) + 0.5) / static_cast<double>(dpi);
}

Vec3 Camera::pixelCenter(int ix, int iy) const {
    // Guard against bad dpi
    int d = (screen.dpi <= 0) ? 1 : screen.dpi;

    const double du = px_step(ix, d);
    const double dv = px_step(iy, d);
    return screen.P + screen.U * du + screen.V * dv;
}

Ray Camera::makeRay(int ix, int iy) const {
    const Vec3 pc  = pixelCenter(ix, iy);
    const Vec3 dir = (pc - eye).normalized();   // ensure unit direction
    return Ray(eye, dir);
}
