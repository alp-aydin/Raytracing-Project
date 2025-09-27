/**
 * @brief JSON loader unit tests for screen, objects, transforms, and validation.
 * Verifies successful parsing, defaults, strict color schema, and rotation constraints.
 */
#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>
#include "json_loader.h"
#include "scene.h"
#include "camera.h"

TEST_CASE("JSON scene loading", "[json][loader]") {
    /// Valid scene containing one red sphere and explicit material fields.
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
                        "diffuse": [1.0, 0.0, 0.0],
                        "ambient": [0.1, 0.0, 0.0],
                        "specular": [0.5, 0.5, 0.5],
                        "reflected": [0.2, 0.2, 0.2],
                        "refracted": [0.0, 0.0, 0.0],
                        "shininess": 32
                    },
                    "index": 1.0
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
    
    /// Minimal scene tests default dpi propagation (72).
    SECTION("Camera has correct default DPI") {
        const std::string minimalJson = R"({
            "screen": {
                "position": [0, 0, 0],
                "dimensions": [2, 2],
                "observer": [0, 0, 5]
            },
            "objects": []
        })";
        
        Scene scene2;
        Camera camera2;
        REQUIRE(jsonio::load_scene_from_json_text(minimalJson, scene2, camera2));
        REQUIRE(camera2.screen.dpi == 72);  // Should use spec default
    }
    
    /// Malformed JSON must throw a parse/processing exception.
    SECTION("Invalid JSON throws exception") {
        const std::string badJson = "{ invalid json }";
        Scene scene3;
        Camera camera3;
        REQUIRE_THROWS(jsonio::load_scene_from_json_text(badJson, scene3, camera3));
    }
    
    /// Color block must be an object, not a raw array.
    SECTION("Strict color format required") {
        const std::string badColorJson = R"({
            "screen": {
                "position": [0, 0, 0],
                "dimensions": [2, 2],
                "observer": [0, 0, 5]
            },
            "objects": [
                {
                    "sphere": {
                        "position": [0, 0, 0],
                        "radius": 1.0,
                        "color": [1.0, 0.0, 0.0]
                    }
                }
            ]
        })";
        
        Scene scene4;
        Camera camera4;
        REQUIRE_THROWS(jsonio::load_scene_from_json_text(badColorJson, scene4, camera4));
    }
}

TEST_CASE("JSON halfSpace loading", "[json][halfspace]") {
    /// Scene with a single halfSpace primitive using a valid color block.
    const std::string testJson = R"({
        "screen": {
            "position": [0, 0, 0],
            "dimensions": [2, 2],
            "observer": [0, 0, 5]
        },
        "objects": [
            {
                "halfSpace": {
                    "position": [0, 0, 0],
                    "normal": [0, 1, 0],
                    "color": {
                        "diffuse": [0.0, 1.0, 0.0],
                        "ambient": [0.0, 0.1, 0.0],
                        "specular": [0.5, 0.5, 0.5],
                        "reflected": [0.0, 0.0, 0.0],
                        "refracted": [0.0, 0.0, 0.0],
                        "shininess": 1
                    }
                }
            }
        ]
    })";
    
    Scene scene;
    Camera camera;
    
    SECTION("halfSpace loads correctly") {
        REQUIRE(jsonio::load_scene_from_json_text(testJson, scene, camera));
        REQUIRE(scene.objects.size() == 1);
    }
}

TEST_CASE("JSON rotation loading", "[json][rotation]") {
    /// Scene with a rotation wrapper (direction=2 ⇒ Z-axis) around a sphere.
    const std::string testJson = R"({
        "screen": {
            "position": [0, 0, 0],
            "dimensions": [2, 2],
            "observer": [0, 0, 5]
        },
        "objects": [
            {
                "rotation": {
                    "angle": 90,
                    "direction": 2,
                    "subject": {
                        "sphere": {
                            "position": [0, 0, 0],
                            "radius": 1.0,
                            "color": {
                                "diffuse": [1.0, 0.0, 0.0],
                                "ambient": [0.1, 0.0, 0.0],
                                "specular": [0.0, 0.0, 0.0],
                                "reflected": [0.0, 0.0, 0.0],
                                "refracted": [0.0, 0.0, 0.0],
                                "shininess": 1
                            }
                        }
                    }
                }
            }
        ]
    })";
    
    Scene scene;
    Camera camera;
    
    SECTION("Rotation with direction=2 (Z-axis) works") {
        REQUIRE(jsonio::load_scene_from_json_text(testJson, scene, camera));
        REQUIRE(scene.objects.size() == 1);
    }
    
    /// Invalid axis index must throw (only {0,1,2} valid).
    SECTION("Invalid rotation direction throws") {
        const std::string badRotationJson = R"({
            "screen": {
                "position": [0, 0, 0],
                "dimensions": [2, 2],
                "observer": [0, 0, 5]
            },
            "objects": [
                {
                    "rotation": {
                        "angle": 90,
                        "direction": 5,
                        "subject": {
                            "sphere": {
                                "position": [0, 0, 0],
                                "radius": 1.0,
                                "color": {
                                    "diffuse": [1.0, 0.0, 0.0],
                                    "ambient": [0.1, 0.0, 0.0],
                                    "specular": [0.0, 0.0, 0.0],
                                    "reflected": [0.0, 0.0, 0.0],
                                    "refracted": [0.0, 0.0, 0.0],
                                    "shininess": 1
                                }
                            }
                        }
                    }
                }
            ]
        })";
        
        Scene scene2;
        Camera camera2;
        REQUIRE_THROWS(jsonio::load_scene_from_json_text(badRotationJson, scene2, camera2));
    }
}
