#pragma once

#include <string>
#include <stdexcept>

/// Camera: pinhole camera and screen specification (forward-declared).
struct Camera;
/// Scene: renderable world with objects, lights, and medium (forward-declared).
struct Scene;
/// Material: shading coefficients parsed from color blocks (forward-declared).
struct Material;

/**
 * @brief JSON scene I/O utilities.
 * Provides loaders that fill a Scene and Camera from disk or in-memory JSON,
 * and a helper to build a Material from a color block.
 */
namespace jsonio {

/**
 * @brief Load a scene from a JSON file.
 * @param filename Path to scene description (UTF-8 JSON).
 * @param scene Output scene (background, objects, lights, medium).
 * @param cam Output camera and screen spec.
 * @return true on success.
 */
bool load_scene_from_json(const std::string& filename, Scene& scene, Camera& cam);

/**
 * @brief Load a scene from JSON text in memory.
 * @param json_text Scene JSON as a single string.
 * @param scene Output scene (background, objects, lights, medium).
 * @param cam Output camera and screen spec.
 * @return true on success.
 */
bool load_scene_from_json_text(const std::string& json_text, Scene& scene, Camera& cam);

/**
 * @brief Build a Material from a "color" JSON block.
 * @param json_color_any Pointer to a nlohmann::json object holding the block.
 * @return Parsed material (albedo, ambient, ks/kr/kt, shininess, ior).
 */
Material make_material_from_color_block(const void* json_color_any);

} // namespace jsonio
