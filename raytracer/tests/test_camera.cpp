#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include "camera.h"

TEST_CASE("Camera ray generation", "[camera]") {
    Camera cam;
    cam.eye = Point3(0, 0, 1);
    cam.screen.P = Point3(-1, -1, 0);
    cam.screen.Lx = 2.0;
    cam.screen.Ly = 2.0;
    cam.screen.dpi = 100;
    
    SECTION("Center ray") {
        int nx = cam.screen.nx();
        int ny = cam.screen.ny();
        Ray ray = cam.generate_ray(nx/2, ny/2);
        
        // Should point roughly toward screen center
        REQUIRE(ray.o.x == Catch::Approx(0.0));
        REQUIRE(ray.o.y == Catch::Approx(0.0));
        REQUIRE(ray.o.z == Catch::Approx(1.0));
        REQUIRE(ray.d.z < 0); // Points toward screen
    }
    
    SECTION("Bounds checking") {
        Ray ray = cam.generate_ray(-1, -1); // Out of bounds
        // Should return fallback ray
        REQUIRE(ray.d.z == Catch::Approx(-1.0));
    }
}