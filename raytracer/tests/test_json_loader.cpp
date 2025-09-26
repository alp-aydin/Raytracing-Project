#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include "json_loader.h"
#include "scene.h"
#include "camera.h"

TEST_CASE("JSON scene loading", "[json][loader]") {
    const std::string testJson = R"({
        "screen": {
            "position": [0, 0, 0],
            "dimensions": [2, 2],
            "dpi": 100,
            "observer": [0, 0, 5]
        },
        "objects": [
            {
                "sphere": {
                    "position": [0, 0, 0],
                    "radius": 1.0,
                    "color": {
                        "diffuse": [1.0, 0.0, 0.0]
                    }
                }
            }
        ]
    })";
    
    Scene scene;
    Camera camera;
    
    SECTION("Valid JSON loads successfully") {
        REQUIRE(jsonio::load_scene_from_json_text(testJson, scene, camera));
        
        REQUIRE(scene.objects.size() == 1);
        REQUIRE(camera.screen.Lx == Catch::Approx(2.0));
        REQUIRE(camera.screen.Ly == Catch::Approx(2.0));
        REQUIRE(camera.screen.dpi == 100);
    }
    
    SECTION("Invalid JSON throws exception") {
        const std::string badJson = "{ invalid json }";
        REQUIRE_THROWS(jsonio::load_scene_from_json_text(badJson, scene, camera));
    }
}