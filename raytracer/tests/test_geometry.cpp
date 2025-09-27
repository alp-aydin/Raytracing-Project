/**
 * @brief Geometry unit tests for Sphere, HalfSpace, and Pokeball primitives.
 * Validates intersection results, hit attributes, and region/material mapping.
 */
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include "geometry.h"

/// Sphere intersection: verify hit distance, miss case, and normal direction.
TEST_CASE("Sphere intersection", "[geometry][sphere]") {
    Material mat;
    mat.albedo = Color(1, 0, 0);
    
    Sphere sphere(Point3(0, 0, 0), 1.0, &mat);
    
    /// Ray should enter at x=-1 with t=1 from (-2,0,0) toward +X.
    SECTION("Ray hits sphere") {
        Ray ray(Point3(-2, 0, 0), Dir3(1, 0, 0));
        Hit hit;
        
        bool intersected = sphere.intersect(ray, 0.001, 1000.0, hit);
        REQUIRE(intersected);
        REQUIRE(hit.t == Catch::Approx(1.0));
        REQUIRE(hit.p.x == Catch::Approx(-1.0));
    }
    
    /// Offset ray misses unit sphere.
    SECTION("Ray misses sphere") {
        Ray ray(Point3(-2, 2, 0), Dir3(1, 0, 0));
        Hit hit;
        
        bool intersected = sphere.intersect(ray, 0.001, 1000.0, hit);
        REQUIRE_FALSE(intersected);
    }
    
    /// Surface normal should be unit and point outward at hit.
    SECTION("Normal is correct") {
        Ray ray(Point3(-2, 0, 0), Dir3(1, 0, 0));
        Hit hit;
        
        sphere.intersect(ray, 0.001, 1000.0, hit);
        REQUIRE(hit.n.length() == Catch::Approx(1.0));
        REQUIRE(hit.n.x == Catch::Approx(-1.0));
        REQUIRE(hit.n.y == Catch::Approx(0.0));
        REQUIRE(hit.n.z == Catch::Approx(0.0));
    }
}

/// HalfSpace intersection: validate plane hit, parallel miss, and normal normalization.
TEST_CASE("HalfSpace intersection", "[geometry][halfspace]") {
    Material mat;
    mat.albedo = Color(0, 1, 0);
    
    HalfSpace plane(Point3(0, 0, 0), Dir3(1, 0, 0), &mat);
    
    /// Ray along +X from x=-1 hits plane x=0 at t=1.
    SECTION("Ray hits halfspace") {
        Ray ray(Point3(-1, 0, 0), Dir3(1, 0, 0));
        Hit hit;
        
        bool intersected = plane.intersect(ray, 0.001, 1000.0, hit);
        REQUIRE(intersected);
        REQUIRE(hit.t == Catch::Approx(1.0));
        REQUIRE(hit.p.x == Catch::Approx(0.0));
    }
    
    /// Ray parallel to plane yields no finite surface hit.
    SECTION("Ray parallel to plane") {
        Ray ray(Point3(1, 0, 0), Dir3(0, 1, 0));
        Hit hit;
        
        bool intersected = plane.intersect(ray, 0.001, 1000.0, hit);
        REQUIRE_FALSE(intersected);
    }
    
    /// Constructed normal should be unit-length even if input was not.
    SECTION("Normal is normalized") {
        HalfSpace plane2(Point3(0, 0, 0), Dir3(3, 4, 0), &mat);
        Ray ray(Point3(-1, 0, 0), Dir3(1, 0, 0));
        Hit hit;
        
        if (plane2.intersect(ray, 0.001, 1000.0, hit)) {
            REQUIRE(plane2.n.length() == Catch::Approx(1.0));
        }
    }
}

/// Pokeball regions: top vs bottom should map to different materials; hits on unit sphere.
TEST_CASE("Pokeball material regions", "[geometry][pokeball]") {
    Pokeball pokeball(Point3(0, 0, 0), 1.0);
    
    // Test that different regions return different materials
    Ray topRay(Point3(0, 2, 0), Dir3(0, -1, 0));    // Hit top
    Ray bottomRay(Point3(0, -2, 0), Dir3(0, 1, 0)); // Hit bottom
    
    Hit topHit, bottomHit;
    REQUIRE(pokeball.intersect(topRay, 0.001, 1000.0, topHit));
    REQUIRE(pokeball.intersect(bottomRay, 0.001, 1000.0, bottomHit));
    
    /// Top and bottom regions should not share the same material pointer.
    REQUIRE(topHit.mat != bottomHit.mat);
    
    /// Both hit points lie on the unit sphere surface.
    SECTION("Hit points are on sphere surface") {
        REQUIRE(topHit.p.length() == Catch::Approx(1.0));
        REQUIRE(bottomHit.p.length() == Catch::Approx(1.0));
    }
}
