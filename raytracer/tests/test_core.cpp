#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include "core.h"

TEST_CASE("Vec3 basic operations", "[core][vec3]") {
    Vec3 a(1, 2, 3);
    Vec3 b(4, 5, 6);
    
    SECTION("Addition") {
        Vec3 c = a + b;
        REQUIRE(c.x == 5);
        REQUIRE(c.y == 7);
        REQUIRE(c.z == 9);
    }
    
    SECTION("Dot product") {
        double dot = a.dot(b);
        REQUIRE(dot == Catch::Approx(32.0));
    }
    
    SECTION("Normalization") {
        Vec3 unit = a.normalized();
        REQUIRE(unit.length() == Catch::Approx(1.0));
    }
}

TEST_CASE("Ray operations", "[core][ray]") {
    Ray r(Point3(0, 0, 0), Dir3(1, 0, 0));
    
    SECTION("Point at parameter") {
        Point3 p = r.at(5.0);
        REQUIRE(p.x == Catch::Approx(5.0));
        REQUIRE(p.y == Catch::Approx(0.0));
        REQUIRE(p.z == Catch::Approx(0.0));
    }
}