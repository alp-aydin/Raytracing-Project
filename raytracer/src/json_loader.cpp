#include "json_loader.h"

// STL
#include <vector>
#include <fstream>
#include <sstream>
#include <memory>
#include <utility>
#include <cmath>
#include <algorithm>

// Project headers
#include "core.h"
#include "camera.h"
#include "scene.h"
#include "geometry.h"
#include "transform.h"
#include "csg.h"

// JSON
#include <nlohmann/json.hpp>
using nlohmann::json;

namespace {

// ---------- lifetime pools (Scene stores raw pointers) ----------
static std::vector<std::shared_ptr<Primitive>> g_nodes;    // own primitives
static std::vector<std::shared_ptr<Material>>  g_mats;     // own materials

inline void reset_pools() {
    g_nodes.clear();
    g_mats.clear();
}

// ---------- small helpers ----------
template <typename T>
T get_or(const json& j, const char* key, const T& fallback) {
    if (!j.contains(key)) return fallback;
    return j.at(key).get<T>();
}

inline Vec3 as_vec3(const json& arr) {
    if (!arr.is_array() || arr.size() != 3) throw std::runtime_error("Expected array[3]");
    return Vec3(arr[0].get<double>(), arr[1].get<double>(), arr[2].get<double>());
}

// Add the missing function
inline Vec3 as_vec3_from_array(const json& arr) {
    return as_vec3(arr);
}

inline Color as_rgb(const json& arr) {
    if (!arr.is_array() || arr.size() != 3) throw std::runtime_error("Expected color array[3]");
    return Color(arr[0].get<double>(), arr[1].get<double>(), arr[2].get<double>());
}

inline void ensure_object_1key(const json& j) {
    if (!j.is_object() || j.size() != 1) throw std::runtime_error("Each object node must be a one-entry object");
}

static inline double deg2rad(double d){ return d * M_PI / 180.0; }

// ---------- materials ----------
Material parse_color_block(const json& jcolor) {
    // Default material as specified in the documentation
    Material m;
    m.albedo    = Color(0,0,0);  // default black
    m.ks        = 0.0;
    m.kd        = 1.0;
    m.shininess = 1.0;

    // Parse all color fields according to spec
    if (jcolor.contains("diffuse")) {
        m.albedo = as_rgb(jcolor.at("diffuse"));
    } else if (jcolor.contains("albedo")) {
        m.albedo = as_rgb(jcolor.at("albedo"));
    }

    if (jcolor.contains("specular")) {
        const Color s = as_rgb(jcolor.at("specular"));
        m.ks = (s.r + s.g + s.b) / 3.0; // scalar ks from RGB average
    }

    if (jcolor.contains("shininess")) {
        m.shininess = jcolor.at("shininess").get<double>();
    }

    if (jcolor.contains("kd")) {
        m.kd = jcolor.at("kd").get<double>(); // optional override
    }

    // Handle additional fields mentioned in spec (even if not used by your Material struct)
    // These are stored but may not be directly used depending on your Material definition
    if (jcolor.contains("ambient")) {
        // ambient lighting is typically handled at scene level, but we can store it
        Color ambient = as_rgb(jcolor.at("ambient"));
        // If your Material has an ambient field, set it here
        // m.ambient = ambient;
    }

    if (jcolor.contains("reflected")) {
        // Reflection coefficient - if your Material supports it
        Color reflected = as_rgb(jcolor.at("reflected"));
        // m.kr = (reflected.r + reflected.g + reflected.b) / 3.0;
    }

    if (jcolor.contains("refracted")) {
        // Refraction coefficient - if your Material supports it
        Color refracted = as_rgb(jcolor.at("refracted"));
        // m.kt = (refracted.r + refracted.g + refracted.b) / 3.0;
    }

    return m;
}

// ---------- forward decls ----------
std::shared_ptr<Primitive> parse_object_node(const json& jnode);

// ---------- screen / medium / lights ----------
void parse_screen(const json& root, Camera& cam) {
    if (!root.contains("screen")) return;
    const json& j = root.at("screen");

    // dpi
    const int dpi = get_or<int>(j, "dpi", 72);

    // dimensions: [Lx, Ly]
    double Lx = 1.0, Ly = 1.0;
    if (j.contains("dimensions")) {
        const json& d = j.at("dimensions");
        if (!d.is_array() || (d.size() != 2 && d.size() != 3)) {
            throw std::runtime_error("screen.dimensions must be [Lx, Ly] (or [Lx, Ly, _]).");
        }
        Lx = d[0].get<double>();
        Ly = d[1].get<double>();
    }

    // position -> bottom-left ScreenSpec::P
    if (!j.contains("position")) {
        throw std::runtime_error("screen.position (bottom-left) is required.");
    }
    const Vec3 P = as_vec3_from_array(j.at("position"));

    // observer -> Camera::eye
    if (!j.contains("observer")) {
        throw std::runtime_error("screen.observer is required.");
    }
    const Vec3 eye = as_vec3_from_array(j.at("observer"));

    // Set up camera with screen spec
    cam.screen.P  = Point3{P.x, P.y, P.z};
    cam.screen.Lx = Lx;
    cam.screen.Ly = Ly;
    cam.screen.dpi = dpi;
    cam.eye = Point3{eye.x, eye.y, eye.z};
}

void parse_medium(const json& root, Scene& scene) {
    if (!root.contains("medium")) return;
    const json& jm = root.at("medium");

    if (jm.contains("ambient")) {
        scene.ambient = as_rgb(jm.at("ambient")); // I_a
    }
    if (jm.contains("index")) {
        scene.medium_index = jm.at("index").get<double>();
    }
    if (jm.contains("recursion")) {
        scene.recursion_limit = jm.at("recursion").get<int>();
    }
}

void parse_sources(const json& root, Scene& scene) {
    if (!root.contains("sources")) return;
    const json& arr = root.at("sources");
    if (!arr.is_array()) throw std::runtime_error("'sources' must be an array");

    for (const auto& js : arr) {
        if (!js.contains("position") || !js.contains("intensity")) {
            throw std::runtime_error("each source needs 'position' and 'intensity'");
        }
        const Vec3 p = as_vec3(js.at("position"));
        const Color I = as_rgb(js.at("intensity"));
        scene.point_lights.push_back({ Point3{p.x, p.y, p.z}, I });
    }
}

// ---------- primitives ----------
std::shared_ptr<Primitive> make_sphere(const json& j) {
    if (!j.contains("position") || !j.contains("radius") || !j.contains("color"))
        throw std::runtime_error("sphere requires 'position', 'radius', 'color'");

    const Vec3 c   = as_vec3(j.at("position"));
    const double r = j.at("radius").get<double>();

    const json& jc = j.at("color");
    Material m = parse_color_block(jc);
    auto mptr  = std::make_shared<Material>(m);
    g_mats.push_back(mptr);

    // Handle index field if present (for refractive index per object)
    if (j.contains("index")) {
        double obj_index = j.at("index").get<double>();
        // Store this in material or primitive as needed by your implementation
        // m.refractive_index = obj_index;
    }

    auto s = std::make_shared<Sphere>(Point3{c.x, c.y, c.z}, r, mptr.get());
    return s;
}

std::shared_ptr<Primitive> make_halfspace(const json& j) {
    if (!j.contains("position") || !j.contains("normal") || !j.contains("color"))
        throw std::runtime_error("halfSpace requires 'position', 'normal', 'color'");

    const Vec3 p0 = as_vec3(j.at("position"));
    Vec3 n        = as_vec3(j.at("normal")).normalized(); // normalize on read

    const json& jc = j.at("color");
    Material m = parse_color_block(jc);
    auto mptr  = std::make_shared<Material>(m);
    g_mats.push_back(mptr);

    // Handle index field if present
    if (j.contains("index")) {
        double obj_index = j.at("index").get<double>();
        // Store this in material or primitive as needed
    }

    auto h = std::make_shared<HalfSpace>(Point3{p0.x, p0.y, p0.z}, Dir3{n.x, n.y, n.z}, mptr.get());
    return h;
}

// ---------- transforms ----------
std::shared_ptr<Primitive> make_scaling(const json& j) {
    if (!j.contains("factors") || !j.contains("subject"))
        throw std::runtime_error("scaling requires 'factors' and 'subject'");
    const Vec3 s = as_vec3(j.at("factors"));
    auto child   = parse_object_node(j.at("subject"));
    return std::make_shared<Scaling>(child, Vec3{s.x, s.y, s.z});
}

std::shared_ptr<Primitive> make_translation(const json& j) {
    if (!j.contains("factors") || !j.contains("subject"))
        throw std::runtime_error("translation requires 'factors' and 'subject'");
    const Vec3 t = as_vec3(j.at("factors"));
    auto child   = parse_object_node(j.at("subject"));
    return std::make_shared<Translation>(child, Vec3{t.x, t.y, t.z});
}

std::shared_ptr<Primitive> make_rotation(const json& j) {
    if (!j.contains("angle") || !j.contains("direction") || !j.contains("subject"))
        throw std::runtime_error("rotation requires 'angle', 'direction', and 'subject'");
    const double angle_deg = j.at("angle").get<double>();
    const int axis_i       = j.at("direction").get<int>(); // 0=x, 1=y, 2|3=z
    Axis ax = Axis::Z;
    if (axis_i == 0) ax = Axis::X;
    else if (axis_i == 1) ax = Axis::Y;
    else ax = Axis::Z; // 2 or 3 both map to Z as per spec
    auto child = parse_object_node(j.at("subject"));
    return std::make_shared<Rotation>(child, ax, deg2rad(angle_deg));
}

// ---------- CSG arrays (fold-left) ----------
std::shared_ptr<Primitive> fold_csg_array(const json& arr, CSGOp op) {
    if (!arr.is_array() || arr.empty()) throw std::runtime_error("CSG array must be a non-empty array");
    std::shared_ptr<Primitive> acc = parse_object_node(arr.at(0));
    for (size_t i = 1; i < arr.size(); ++i) {
        auto rhs = parse_object_node(arr.at(i));
        acc = std::make_shared<CSG>(op, acc, rhs);
    }
    return acc;
}

// ---------- dispatcher ----------
std::shared_ptr<Primitive> parse_object_node(const json& jnode) {
    ensure_object_1key(jnode);
    const auto it = jnode.begin();
    const std::string kind = it.key();
    const json& val = it.value();

    if (kind == "sphere")       return make_sphere(val);
    if (kind == "halfspace" || kind == "halfSpace") return make_halfspace(val);

    // transforms
    if (kind == "scaling")      return make_scaling(val);
    if (kind == "translation")  return make_translation(val);
    if (kind == "rotation")     return make_rotation(val);

    // CSG arrays
    if (kind == "union")        return fold_csg_array(val, CSGOp::Union);
    if (kind == "intersection") return fold_csg_array(val, CSGOp::Intersection);
    if (kind == "difference")   return fold_csg_array(val, CSGOp::Difference);

    throw std::runtime_error("unknown object kind: " + kind);
}

} // anon namespace

// ---------- public API ----------
namespace jsonio {

Material make_material_from_color_block(const void* json_color_any) {
    const json& j = *reinterpret_cast<const json*>(json_color_any);
    return parse_color_block(j);
}

bool load_scene_from_json_text(const std::string& json_text, Scene& scene, Camera& cam) {
    reset_pools();

    json root = json::parse(json_text);

    // screen, medium, sources
    parse_screen(root, cam);
    parse_medium(root, scene);
    parse_sources(root, scene);

    if (root.contains("background")) {
    const auto& jb = root.at("background");
    if (!jb.is_array() || jb.size() != 3)
        throw std::runtime_error("background must be array[3]");
    scene.background = as_rgb(jb);
    }

    // objects
    scene.objects.clear();
    if (root.contains("objects")) {
        const json& arr = root.at("objects");
        if (!arr.is_array()) throw std::runtime_error("'objects' must be an array");
        for (const auto& node : arr) {
            auto prim = parse_object_node(node);
            if (prim) {
                g_nodes.push_back(prim);                 // own it
                scene.objects.push_back(prim.get());     // scene keeps raw ptr
            }
        }
    }

    return true;
}

bool load_scene_from_json(const std::string& filename, Scene& scene, Camera& cam) {
    std::ifstream ifs(filename);
    if (!ifs) throw std::runtime_error("Cannot open JSON file: " + filename);
    std::ostringstream ss; ss << ifs.rdbuf();
    return load_scene_from_json_text(ss.str(), scene, cam);
}

} // namespace jsonio