#pragma once

#include <string>
#include <stdexcept>

// Forward declarations — your actual headers define these.
struct Camera;
struct Scene;
struct Material;

namespace jsonio {

// Load from file path. Fills Scene and Camera. Throws on hard parse errors.
bool load_scene_from_json(const std::string& filename, Scene& scene, Camera& cam);

// Load from in-memory JSON text (same behavior as above).
bool load_scene_from_json_text(const std::string& json_text, Scene& scene, Camera& cam);

// Helper: build a Material from a "color" JSON block.
// The pointer must point to a nlohmann::json object (kept void* to keep the header light).
Material make_material_from_color_block(const void* json_color_any);

} // namespace jsonio
