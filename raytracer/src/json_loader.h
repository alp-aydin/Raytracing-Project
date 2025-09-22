#pragma once
#include <memory>
#include <functional>
#include <string>
#include <vector>
#include <unordered_map>
#include "core.h"
#include "geometry.h"
#include "transform.h"
#include "camera.h"              // <-- ADD THIS

#include <nlohmann/json.hpp>

struct SceneFactories {
    // ... (unchanged)
};

// Parses a single object (supports transform wrappers and CSG)
std::shared_ptr<Primitive> parse_object_tree(
    const nlohmann::json& node,
    const SceneFactories& factories,
    const std::unordered_map<std::string, Material>* materials = nullptr);

// Parse a complete scene file that contains an array under "objects".
std::vector<std::shared_ptr<Primitive>> load_scene_objects_from_file(
    const std::string& filepath,
    const SceneFactories& factories,
    const std::unordered_map<std::string, Material>* materials = nullptr);

// Materials helper
std::unordered_map<std::string, Material> load_materials(const nlohmann::json& root);

// ---------- ADD THIS: camera loader ----------
namespace json_loader {
    // Reads:
    //   screen.position   : [Px,Py,Pz]        (bottom-left in world coords)
    //   screen.dimensions : [Lx,Ly]           (width/height in world units)
    //   screen.dpi        : int               (pixels per unit; same on both axes)
    //   screen.observer   : [Cx,Cy,Cz]        (eye position)
    //
    // Optional overrides (kept off by default to match spec’s XY-plane):
    //   screen.U, screen.V: [ux,uy,uz], [vx,vy,vz] (full-edge vectors; if present, they override Lx/Ly XY defaults)
    Camera load_camera_from_json(const nlohmann::json& root);
}
