/**
 * @brief CSG unit tests: validate Union and Difference behaviors with simple spheres.
 * Ensures closest-hit logic and hollow subtraction work under ray queries.
 */
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include "csg.h"
#include "geometry.h"

/// Union of two overlapping spheres should yield a hit from outside.
TEST_CASE("CSG Union operation", "[csg][union]") {
    Material mat;
    auto sphere1 = std::make_shared<Sphere>(Point3(-0.5, 0, 0), 0.7, &mat);
    auto sphere2 = std::make_shared<Sphere>(Point3(0.5, 0, 0), 0.7, &mat);
    
    CSG unionCSG(CSGOp::Union, sphere1, sphere2);
    
    /// Ray from the left should hit the union near the first sphere.
    SECTION("Ray hits union") {
        Ray ray(Point3(-2, 0, 0), Dir3(1, 0, 0));
        Hit hit;
        
        bool intersected = unionCSG.intersect(ray, 0.001, 1000.0, hit);
        REQUIRE(intersected);
        // Should hit the first sphere
        REQUIRE(hit.t < 2.0);
    }
}

/// Difference (big - small) concentric spheres should create a hollow shell.
TEST_CASE("CSG Difference operation", "[csg][difference]") {
    Material mat;
    auto bigSphere = std::make_shared<Sphere>(Point3(0, 0, 0), 1.0, &mat);
    auto smallSphere = std::make_shared<Sphere>(Point3(0, 0, 0), 0.5, &mat);
    
    CSG difference(CSGOp::Difference, bigSphere, smallSphere);
    
    /// Forward ray along +Z should hit the inner surface of the shell.
    SECTION("Ray hits hollow region") {
        Ray ray(Point3(0, 0, -2), Dir3(0, 0, 1));
        Hit hit;
        
        bool intersected = difference.intersect(ray, 0.001, 1000.0, hit);
        // Should hit the inner surface of the hollow
        REQUIRE(intersected);
    }
}
