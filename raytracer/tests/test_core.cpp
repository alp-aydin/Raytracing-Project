#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include "core.h"

/// Verify Vec4 arithmetic, dot product, and normalization semantics.
TEST_CASE("Vec4 basic operations", "[core][vec4]") {
    Vec4 a(1, 2, 3, 0);  // Direction vector
    Vec4 b(4, 5, 6, 0);  // Direction vector
    
    /// Sum preserves component-wise addition and w-tag sum.
    SECTION("Addition") {
        Vec4 c = a + b;
        REQUIRE(c.x == 5);
        REQUIRE(c.y == 7);
        REQUIRE(c.z == 9);
        REQUIRE(c.w == 0);
    }
    
    /// Dot product uses (x,y,z) only; w is ignored.
    SECTION("Dot product") {
        double dot = a.dot(b);
        REQUIRE(dot == Catch::Approx(32.0));
    }
    
    /// Normalized() returns unit length and preserves w=0 for directions.
    SECTION("Normalization") {
        Vec4 unit = a.normalized();
        REQUIRE(unit.length() == Catch::Approx(1.0));
        REQUIRE(unit.w == 0);  // Direction vectors keep w=0
    }
}

/// Validate Point3/Dir3 construction and point–point difference semantics.
TEST_CASE("Point3 and Dir3 operations", "[core][homogeneous]") {
    Point3 p1(1, 2, 3);
    Point3 p2(4, 5, 6);
    Dir3 d(1, 0, 0);
    
    /// Points should carry w=1.
    SECTION("Point creation") {
        REQUIRE(p1.x == 1);
        REQUIRE(p1.y == 2);
        REQUIRE(p1.z == 3);
        REQUIRE(p1.w == 1);  // Points have w=1
    }
    
    /// Directions should carry w=0.
    SECTION("Direction creation") {
        REQUIRE(d.x == 1);
        REQUIRE(d.y == 0);
        REQUIRE(d.z == 0);
        REQUIRE(d.w == 0);  // Directions have w=0
    }
    
    /// p2 - p1 yields a direction (w=0).
    SECTION("Vector from points") {
        Vec4 diff = p2 - p1;  // This creates a direction
        REQUIRE(diff.x == 3);
        REQUIRE(diff.y == 3);
        REQUIRE(diff.z == 3);
        REQUIRE(diff.w == 0);  // Difference of points is a direction
    }
    
    /// Dir3::normalized() yields unit length with w=0.
    SECTION("Direction normalization") {
        Dir3 unit = Dir3(3, 4, 0).normalized();
        REQUIRE(unit.length() == Catch::Approx(1.0));
        REQUIRE(unit.w == 0);
    }
}

/// Check Matrix4 identity, translation, and scaling behavior on points/dirs.
TEST_CASE("Matrix operations", "[core][matrix]") {
    Matrix4 identity;
    Point3 p(1, 2, 3);
    Dir3 d(0, 1, 0);
    
    /// Identity leaves points and directions unchanged.
    SECTION("Identity transformation") {
        Vec4 result_p = identity * p;
        Vec4 result_d = identity * d;
        
        REQUIRE(result_p.x == Catch::Approx(1.0));
        REQUIRE(result_p.y == Catch::Approx(2.0));
        REQUIRE(result_p.z == Catch::Approx(3.0));
        REQUIRE(result_p.w == Catch::Approx(1.0));
        
        REQUIRE(result_d.x == Catch::Approx(0.0));
        REQUIRE(result_d.y == Catch::Approx(1.0));
        REQUIRE(result_d.z == Catch::Approx(0.0));
        REQUIRE(result_d.w == Catch::Approx(0.0));
    }
    
    /// Translation offsets points; directions remain unaffected.
    SECTION("Translation matrix") {
        Matrix4 trans = Matrix4::translation(1, 2, 3);
        Vec4 result = trans * p;
        
        REQUIRE(result.x == Catch::Approx(2.0));
        REQUIRE(result.y == Catch::Approx(4.0));
        REQUIRE(result.z == Catch::Approx(6.0));
        REQUIRE(result.w == Catch::Approx(1.0));
    }
    
    /// Scaling multiplies point coordinates component-wise.
    SECTION("Scaling matrix") {
        Matrix4 scale = Matrix4::scaling(2, 3, 4);
        Vec4 result = scale * p;
        
        REQUIRE(result.x == Catch::Approx(2.0));
        REQUIRE(result.y == Catch::Approx(6.0));
        REQUIRE(result.z == Catch::Approx(12.0));
        REQUIRE(result.w == Catch::Approx(1.0));
    }
}

/// Ensure Ray::at and direction normalization behave as documented.
TEST_CASE("Ray operations", "[core][ray]") {
    Ray r(Point3(0, 0, 0), Dir3(1, 0, 0));
    
    /// at(t) returns o + t*d.
    SECTION("Point at parameter") {
        Point3 p = r.at(5.0);
        REQUIRE(p.x == Catch::Approx(5.0));
        REQUIRE(p.y == Catch::Approx(0.0));
        REQUIRE(p.z == Catch::Approx(0.0));
    }
    
    /// Constructor normalizes the direction.
    SECTION("Ray direction is normalized") {
        Ray r2(Point3(0, 0, 0), Dir3(3, 4, 0));
        REQUIRE(r2.d.length() == Catch::Approx(1.0));
    }
}
