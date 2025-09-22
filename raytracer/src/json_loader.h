#pragma once
#include <string>

struct Scene;
struct Camera;

namespace json_loader {

// Load from a JSON file on disk.
bool load_all_from_file(const std::string& path, Scene& scene, Camera& camera);

// Load from a JSON text string already in memory.
bool load_all_from_string(const std::string& json_text, Scene& scene, Camera& camera);

} // namespace json_loader
