#pragma once
#include <vector>
#include "core.h"
#include "geometry.h"  // defines Primitive and Hit

struct DirectionalLight {
    // Direction *from the light toward the scene*
    Dir3  dir = Dir3(0,-1,0);
    Color radiance{1,1,1};
};

struct PointLight {
    Point3 pos{0,0,0};
    // Simple 1/r^2 falloff used in shader
    Color  intensity{10,10,10};
};

struct Scene {
    std::vector<const Primitive*> objects;  // Scene does NOT own these
    std::vector<DirectionalLight> dir_lights;
    std::vector<PointLight>       point_lights;
    Color background{0,0,0};

    // Find nearest hit in [tmin, tmax); returns true if anything was hit.
    bool intersect(const Ray& r, double tmin, double tmax, Hit& out) const;

    // Visibility query for shadow rays: any hit in (tmin, tmax)?
    bool occluded(const Ray& r, double tmin, double tmax) const;
};
