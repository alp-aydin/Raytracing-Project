#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include "camera.h"

/**
 * @brief Unit tests for Camera ray generation against a square screen.
 * Verifies center-direction, bounds fallback, normalization, and default DPI.
 */
TEST_CASE("Camera ray generation", "[camera]") {
    Camera cam;
    cam.eye = Point3(0, 0, 1);
    cam.screen.P = Point3(-1, -1, 0);
    cam.screen.Lx = 2.0;
    cam.screen.Ly = 2.0;
    cam.screen.dpi = 100;
    
    /// Center pixel should produce a ray toward screen center (z-negative).
    SECTION("Center ray") {
        int nx = cam.screen.nx();
        int ny = cam.screen.ny();
        Ray ray = cam.generate_ray(nx/2, ny/2);
        
        REQUIRE(ray.o.x == Catch::Approx(0.0));
        REQUIRE(ray.o.y == Catch::Approx(0.0));
        REQUIRE(ray.o.z == Catch::Approx(1.0));
        REQUIRE(ray.d.z < 0);
    }
    
    /// Out-of-bounds request returns the documented fallback ray (0,0,-1).
    SECTION("Bounds checking") {
        Ray ray = cam.generate_ray(-1, -1); // Out of bounds
        REQUIRE(ray.d.z == Catch::Approx(-1.0));
    }
    
    /// Generated ray directions are unit-length.
    SECTION("Ray direction is normalized") {
        Ray ray = cam.generate_ray(0, 0);
        REQUIRE(ray.d.length() == Catch::Approx(1.0));
    }
    
    /// Default DPI value propagated by ScreenSpec.
    SECTION("Default DPI is 72") {
        Camera default_cam;
        REQUIRE(default_cam.screen.dpi == 72);
    }
}
