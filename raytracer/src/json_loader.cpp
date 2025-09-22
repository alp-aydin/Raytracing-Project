#include "json_loader.h"
#include <fstream>
#include <stdexcept>



// ------------------ camera loader ------------------
namespace json_loader {

static inline void ensure_pos(double v, const char* name) {
    if (!(v > 0.0)) throw std::runtime_error(std::string(name) + " must be > 0");
}
static inline void ensure_pos_int(int v, const char* name) {
    if (v <= 0) throw std::runtime_error(std::string(name) + " must be > 0");
}
static inline bool has_vec3(const nlohmann::json& j) {
    return j.is_array() && j.size()==3 && j[0].is_number() && j[1].is_number() && j[2].is_number();
}
static inline bool has_vec2(const nlohmann::json& j) {
    return j.is_array() && j.size()==2 && j[0].is_number() && j[1].is_number();
}

Camera load_camera_from_json(const nlohmann::json& root) {
    if (!root.contains("screen"))
        throw std::runtime_error("JSON missing required 'screen' block.");

    const auto& js = root.at("screen");

    if (!js.contains("position") || !js.contains("dimensions") || !js.contains("dpi") || !js.contains("observer"))
        throw std::runtime_error("screen requires 'position', 'dimensions', 'dpi', and 'observer'.");

    const auto& pos = js.at("position");
    const auto& dim = js.at("dimensions");
    const auto& obs = js.at("observer");

    if (!has_vec3(pos)) throw std::runtime_error("screen.position must be [Px,Py,Pz]");
    if (!has_vec2(dim)) throw std::runtime_error("screen.dimensions must be [Lx,Ly]");
    if (!has_vec3(obs)) throw std::runtime_error("screen.observer must be [Cx,Cy,Cz]");

    Camera cam;
    cam.screen.P  = Point3{ pos[0].get<double>(), pos[1].get<double>(), pos[2].get<double>() };
    cam.screen.Lx = dim[0].get<double>();
    cam.screen.Ly = dim[1].get<double>();
    cam.screen.dpi= js.at("dpi").get<int>();
    cam.eye       = Point3{ obs[0].get<double>(), obs[1].get<double>(), obs[2].get<double>() };

    ensure_pos(cam.screen.Lx, "screen.dimensions[0]");
    ensure_pos(cam.screen.Ly, "screen.dimensions[1]");
    ensure_pos_int(cam.screen.dpi, "screen.dpi");

    // Optional: allow explicit U/V overrides (full-edge vectors)
    if (js.contains("U") && js.contains("V")) {
        const auto& Uj = js["U"];
        const auto& Vj = js["V"];
        if (!has_vec3(Uj) || !has_vec3(Vj))
            throw std::runtime_error("If provided, screen.U and screen.V must be 3-vectors.");
        cam.screen.U = Dir3{ Uj[0].get<double>(), Uj[1].get<double>(), Uj[2].get<double>() };
        cam.screen.V = Dir3{ Vj[0].get<double>(), Vj[1].get<double>(), Vj[2].get<double>() };
    } else {
        // Default XY-plane edges per spec
        cam.screen.U = Dir3{ cam.screen.Lx, 0.0, 0.0 };
        cam.screen.V = Dir3{ 0.0, cam.screen.Ly, 0.0 };
    }

    return cam;
}

} 
