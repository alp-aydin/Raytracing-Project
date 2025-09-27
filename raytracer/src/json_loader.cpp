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

/// Object/material ownership pools (Scene stores raw pointers).
static std::vector<std::shared_ptr<Primitive>> g_nodes;    // own primitives
/// Object/material ownership pools (Scene stores raw pointers).
static std::vector<std::shared_ptr<Material>>  g_mats;     // own materials

/// Clear ownership pools at the start of a new load.
inline void reset_pools() {
    g_nodes.clear();
    g_mats.clear();
}

/**
 * @brief Read key or fallback from a JSON object.
 * @param j Source object.
 * @param key Property to read.
 * @param fallback Value returned if key is absent.
 * @return Parsed value of T or fallback.
 */
template <typename T>
T get_or(const json& j, const char* key, const T& fallback) {
    if (!j.contains(key)) return fallback;
    return j.at(key).get<T>();
}

/**
 * @brief Parse a JSON array[3] into Vec3.
 * @param arr JSON array with 3 doubles.
 * @return Vec3 filled from arr.
 */
inline Vec3 as_vec3(const json& arr) {
    if (!arr.is_array() || arr.size() != 3) throw std::runtime_error("Expected array[3]");
    return Vec3(arr[0].get<double>(), arr[1].get<double>(), arr[2].get<double>());
}

/**
 * @brief Parse a JSON array[3] into linear RGB Color.
 * @param arr JSON array with 3 doubles.
 * @return Color filled from arr.
 */
inline Color as_rgb(const json& arr) {
    if (!arr.is_array() || arr.size() != 3) throw std::runtime_error("Expected color array[3]");
    return Color(arr[0].get<double>(), arr[1].get<double>(), arr[2].get<double>());
}

/// Ensure node is a one-entry object (tagged union form).
inline void ensure_object_1key(const json& j) {
    if (!j.is_object() || j.size() != 1) throw std::runtime_error("Each object node must be a one-entry object");
}

/// Degrees-to-radians conversion.
static inline double deg2rad(double d){ return d * M_PI / 180.0; }

/**
 * @brief Build a Material from a "color" block.
 * @param jcolor JSON object with fields: diffuse, ambient, specular, reflected, refracted, shininess.
 * @return Material ready for shading (ks/kr/kt derived from input colors).
 */
Material parse_color_block(const json& jcolor) {
    if (!jcolor.is_object()) {
        throw std::runtime_error("color must be an object");
    }

    // Defaults = "black" material per spec
    Material m;
    m.albedo           = Color(0,0,0);   // use diffuse as base color
    m.ambient          = Color(0,0,0);
    m.kd               = 1.0;
    m.ks               = 0.0;
    m.kr               = 0.0;
    m.kt               = 0.0;
    m.shininess        = 1.0;
    m.refractive_index = 1.0;

    // NOTE: We currently keep ks/kr/kt as scalars to match your shader.
    // We average RGB inputs so scenes authored to spec still behave sensibly.
    if (jcolor.contains("diffuse")) {
        m.albedo = as_rgb(jcolor.at("diffuse"));
    }
    if (jcolor.contains("ambient")) {
        m.ambient = as_rgb(jcolor.at("ambient"));
    }
    if (jcolor.contains("specular")) {
        const Color s = as_rgb(jcolor.at("specular"));
        m.ks = (s.r + s.g + s.b) / 3.0;
    }
    if (jcolor.contains("reflected")) {
        const Color r = as_rgb(jcolor.at("reflected"));
        m.kr = (r.r + r.g + r.b) / 3.0;
    }
    if (jcolor.contains("refracted")) {
        const Color t = as_rgb(jcolor.at("refracted"));
        m.kt = (t.r + t.g + t.b) / 3.0;
    }
    if (jcolor.contains("shininess")) {
        m.shininess = jcolor.at("shininess").get<double>();
    }
    return m;
}

/// Forward declaration for object node dispatcher.
std::shared_ptr<Primitive> parse_object_node(const json& jnode);

/**
 * @brief Parse "screen" block into Camera.
 * @param root Root JSON object.
 * @param cam Output camera with screen/eye set.
 */
void parse_screen(const json& root, Camera& cam) {
    if (!root.contains("screen")) return;
    const json& j = root.at("screen");

    // dpi (default 72)
    const int dpi = get_or<int>(j, "dpi", 72);

    // dimensions [Lx, Ly]
    double Lx = 1.0, Ly = 1.0;
    if (j.contains("dimensions")) {
        const json& d = j.at("dimensions");
        if (!d.is_array() || d.size() != 2) {
            throw std::runtime_error("screen.dimensions must be [Lx, Ly]");
        }
        Lx = d[0].get<double>();
        Ly = d[1].get<double>();
    }

    // bottom-left P (required)
    if (!j.contains("position")) throw std::runtime_error("screen.position is required");
    const Vec3 P = as_vec3(j.at("position"));

    // observer (eye) required
    if (!j.contains("observer")) throw std::runtime_error("screen.observer is required");
    const Vec3 eye = as_vec3(j.at("observer"));

    cam.screen.P   = Point3{P.x, P.y, P.z};
    cam.screen.Lx  = Lx;
    cam.screen.Ly  = Ly;
    cam.screen.dpi = dpi;
    cam.eye        = Point3{eye.x, eye.y, eye.z};
}

/**
 * @brief Parse "medium" block into Scene globals.
 * @param root Root JSON object.
 * @param scene Output scene (ambient, IOR, recursion limit).
 */
void parse_medium(const json& root, Scene& scene) {
    if (!root.contains("medium")) return;
    const json& jm = root.at("medium");

    if (jm.contains("ambient")) {
        scene.ambient = as_rgb(jm.at("ambient"));
    }
    if (jm.contains("index")) {
        scene.medium_index = jm.at("index").get<double>();
    }
    if (jm.contains("recursion")) {
        scene.recursion_limit = jm.at("recursion").get<int>();
    }
}

/**
 * @brief Parse point lights from "sources" array.
 * @param root Root JSON object.
 * @param scene Output scene with appended lights.
 */
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

/**
 * @brief Construct a Sphere from JSON.
 * @param j Object payload with position, radius, color, optional index.
 * @return Shared primitive; material stored in pool.
 */
std::shared_ptr<Primitive> make_sphere(const json& j) {
    if (!j.contains("position") || !j.contains("radius") || !j.contains("color"))
        throw std::runtime_error("sphere requires 'position', 'radius', 'color'");

    const Vec3 c   = as_vec3(j.at("position"));
    const double r = j.at("radius").get<double>();

    Material m = parse_color_block(j.at("color"));

    // per-object refractive index override
    if (j.contains("index")) m.refractive_index = j.at("index").get<double>();

    auto mptr = std::make_shared<Material>(m);
    g_mats.push_back(mptr);

    return std::make_shared<Sphere>(Point3{c.x, c.y, c.z}, r, mptr.get());
}

/**
 * @brief Construct a HalfSpace from JSON.
 * @param j Object payload with position, normal, color, optional index.
 * @return Shared primitive; material stored in pool.
 */
std::shared_ptr<Primitive> make_halfspace(const json& j) {
    if (!j.contains("position") || !j.contains("normal") || !j.contains("color"))
        throw std::runtime_error("halfSpace requires 'position', 'normal', 'color'");

    const Vec3 p0 = as_vec3(j.at("position"));
    Vec3 n        = as_vec3(j.at("normal"));

    Material m = parse_color_block(j.at("color"));
    if (j.contains("index")) m.refractive_index = j.at("index").get<double>();

    auto mptr = std::make_shared<Material>(m);
    g_mats.push_back(mptr);

    return std::make_shared<HalfSpace>(Point3{p0.x, p0.y, p0.z}, Dir3{n.x, n.y, n.z}, mptr.get());
}

/**
 * @brief Construct a Pokeball (extension) from JSON.
 * @param jnode Payload with position, radius, optional colors and button controls.
 * @return Shared primitive with region materials configured.
 */
// optional extension kept
std::shared_ptr<Primitive> make_pokeball(const json& jnode) {
    if (!jnode.contains("position") || !jnode.contains("radius")) {
        throw std::runtime_error("pokeball requires 'position' and 'radius'.");
    }
    const Vec3 c = as_vec3(jnode.at("position"));
    const double r = jnode.at("radius").get<double>();

    // sensible defaults
    Material top, bottom, belt, ring, button;
    top.albedo    = Color(0.88,0.12,0.20); top.kd=1.0; top.ks=0.15; top.shininess=64;
    bottom.albedo = Color(0.95,0.95,0.98); bottom.kd=1.0; bottom.ks=0.08; bottom.shininess=32;
    belt.albedo   = Color(0.12,0.12,0.15); belt.kd=1.0;
    ring.albedo   = Color(0.35,0.35,0.40); ring.kd=1.0;
    button.albedo = Color(0.96,0.96,0.99); button.kd=1.0; button.ks=0.25; button.shininess=64;

    double belt_half   = 0.06;
    double btn_outer   = 0.28;
    double ring_width  = 0.06;
    Dir3   btn_dir     = Dir3{1,0,0};

    if (jnode.contains("colors")) {
        const json& jc = jnode.at("colors");
        if (jc.contains("top"))     top    = parse_color_block(jc.at("top"));
        if (jc.contains("bottom"))  bottom = parse_color_block(jc.at("bottom"));
        if (jc.contains("belt"))    belt   = parse_color_block(jc.at("belt"));
        if (jc.contains("ring"))    ring   = parse_color_block(jc.at("ring"));
        if (jc.contains("button"))  button = parse_color_block(jc.at("button"));
    }
    if (jnode.contains("belt_half"))    belt_half  = jnode.at("belt_half").get<double>();
    if (jnode.contains("button_outer")) btn_outer  = jnode.at("button_outer").get<double>();
    if (jnode.contains("ring_width"))   ring_width = jnode.at("ring_width").get<double>();
    if (jnode.contains("button_dir")) {
        const Vec3 d = as_vec3(jnode.at("button_dir"));
        btn_dir = Dir3{d.x, d.y, d.z};
    }

    return std::make_shared<Pokeball>(
        Point3{c.x, c.y, c.z}, r,
        top, bottom, belt, ring, button,
        belt_half, btn_outer, ring_width, btn_dir
    );
}

/**
 * @brief Wrap a child in a Scaling transform.
 * @param j Payload with factors [sx,sy,sz] and subject node.
 * @return Transformed primitive.
 */
std::shared_ptr<Primitive> make_scaling(const json& j) {
    if (!j.contains("factors") || !j.contains("subject"))
        throw std::runtime_error("scaling requires 'factors' and 'subject'");
    const Vec3 s = as_vec3(j.at("factors"));
    auto child   = parse_object_node(j.at("subject"));
    return std::make_shared<Scaling>(child, Vec3{s.x, s.y, s.z});
}

/**
 * @brief Wrap a child in a Translation transform.
 * @param j Payload with factors [tx,ty,tz] and subject node.
 * @return Transformed primitive.
 */
std::shared_ptr<Primitive> make_translation(const json& j) {
    if (!j.contains("factors") || !j.contains("subject"))
        throw std::runtime_error("translation requires 'factors' and 'subject'");
    const Vec3 t = as_vec3(j.at("factors"));
    auto child   = parse_object_node(j.at("subject"));
    return std::make_shared<Translation>(child, Vec3{t.x, t.y, t.z});
}

/**
 * @brief Wrap a child in a Rotation transform.
 * @param j Payload with angle (deg), direction axis index {0,1,2}, and subject node.
 * @return Transformed primitive.
 */
std::shared_ptr<Primitive> make_rotation(const json& j) {
    if (!j.contains("angle") || !j.contains("direction") || !j.contains("subject"))
        throw std::runtime_error("rotation requires 'angle', 'direction', and 'subject'");

    const double angle_deg = j.at("angle").get<double>();
    const int axis_i       = j.at("direction").get<int>();

    Axis ax;
    if (axis_i == 0)      ax = Axis::X;
    else if (axis_i == 1) ax = Axis::Y;
    else if (axis_i == 2) ax = Axis::Z;
    else throw std::runtime_error("rotation direction must be 0 (X), 1 (Y), or 2 (Z)");

    auto child = parse_object_node(j.at("subject"));
    return std::make_shared<Rotation>(child, ax, deg2rad(angle_deg));
}

/**
 * @brief Construct a binary CSG node from JSON.
 * @param j Payload with operator ("union","intersection","difference"), left, right.
 * @return Composed CSG primitive.
 */
// ---------- CSG operations ----------
// binary "csg" shape
std::shared_ptr<Primitive> make_csg_binary(const json& j) {
    if (!j.contains("operator") || !j.contains("left") || !j.contains("right"))
        throw std::runtime_error("csg requires 'operator', 'left', 'right'");
    const std::string op = j.at("operator").get<std::string>();
    CSGOp cop;
    if      (op == "union")        cop = CSGOp::Union;
    else if (op == "intersection") cop = CSGOp::Intersection;
    else if (op == "difference")   cop = CSGOp::Difference;
    else throw std::runtime_error("csg.operator must be union/intersection/difference");
    auto lhs = parse_object_node(j.at("left"));
    auto rhs = parse_object_node(j.at("right"));
    return std::make_shared<CSG>(cop, lhs, rhs);
}

/**
 * @brief Fold an array of nodes with a CSG op (union/intersection).
 * @param arr Non-empty array of nodes.
 * @param op Operation applied left-to-right.
 * @return Composed CSG primitive.
 */
// left-fold over array for union/intersection
std::shared_ptr<Primitive> fold_csg_array(const json& arr, CSGOp op) {
    if (!arr.is_array() || arr.empty())
        throw std::runtime_error("CSG array must be a non-empty array");
    std::shared_ptr<Primitive> acc = parse_object_node(arr.at(0));
    for (size_t i = 1; i < arr.size(); ++i) {
        auto rhs = parse_object_node(arr.at(i));
        acc = std::make_shared<CSG>(op, acc, rhs);
    }
    return acc;
}

/**
 * @brief Left-fold a difference list: O1 - O2 - ... - ON.
 * @param arr Array with at least two elements.
 * @return Composed CSG primitive.
 */
// O1 - O2 - ... - ON
std::shared_ptr<Primitive> fold_difference_array(const json& arr) {
    if (!arr.is_array() || arr.size() < 2)
        throw std::runtime_error("difference array must have at least 2 elements");
    std::shared_ptr<Primitive> acc = parse_object_node(arr.at(0));
    for (size_t i = 1; i < arr.size(); ++i) {
        auto rhs = parse_object_node(arr.at(i));
        acc = std::make_shared<CSG>(CSGOp::Difference, acc, rhs);
    }
    return acc;
}

/**
 * @brief Dispatch an object node to its concrete builder.
 * @param jnode One-entry object: {kind: payload}.
 * @return Constructed primitive (possibly transformed/CSG).
 */
// ---------- dispatcher ----------
std::shared_ptr<Primitive> parse_object_node(const json& jnode) {
    ensure_object_1key(jnode);
    const auto it = jnode.begin();
    const std::string kind = it.key();
    const json& val = it.value();

    // Primitives (strict spellings)
    if (kind == "sphere")       return make_sphere(val);
    if (kind == "halfSpace")    return make_halfspace(val);
    if (kind == "pokeball")     return make_pokeball(val); // extension

    // Transforms
    if (kind == "scaling")      return make_scaling(val);
    if (kind == "translation")  return make_translation(val);
    if (kind == "rotation")     return make_rotation(val);

    // Binary CSG
    if (kind == "csg")          return make_csg_binary(val);

    // Variadic CSG
    if (kind == "union")        return fold_csg_array(val, CSGOp::Union);
    if (kind == "intersection") return fold_csg_array(val, CSGOp::Intersection);
    if (kind == "difference")   return fold_difference_array(val);

    throw std::runtime_error("unknown object kind: " + kind);
}

} // anon

// ---------- public API ----------
namespace jsonio {

/**
 * @brief Convert a color-block JSON object (opaque pointer) to Material.
 * @param json_color_any Pointer to nlohmann::json color object.
 * @return Material parsed as in scene files.
 */
Material make_material_from_color_block(const void* json_color_any) {
    const json& j = *reinterpret_cast<const json*>(json_color_any);
    return parse_color_block(j);
}

/**
 * @brief Parse a scene from JSON text.
 * @param json_text UTF-8 JSON string.
 * @param scene Output scene (background, lights, objects).
 * @param cam Output camera.
 * @return true on success; throws on parse/validation errors.
 */
bool load_scene_from_json_text(const std::string& json_text, Scene& scene, Camera& cam) {
    reset_pools();
    try {
        json root = json::parse(json_text);

        parse_screen(root, cam);
        parse_medium(root, scene);
        parse_sources(root, scene);

        if (root.contains("background")) {
            const auto& jb = root.at("background");
            scene.background = as_rgb(jb);
        }

        scene.objects.clear();
        if (root.contains("objects")) {
            const json& arr = root.at("objects");
            if (!arr.is_array()) throw std::runtime_error("'objects' must be an array");
            for (const auto& node : arr) {
                auto prim = parse_object_node(node);
                if (prim) {
                    g_nodes.push_back(prim);
                    scene.objects.push_back(prim.get());
                }
            }
        }
        return true;
    } catch (const json::parse_error& e) {
        throw std::runtime_error(std::string("JSON parse error: ") + e.what());
    } catch (const std::exception& e) {
        throw std::runtime_error(std::string("JSON processing error: ") + e.what());
    }
}

/**
 * @brief Parse a scene from a JSON file on disk.
 * @param filename Path to scene.json.
 * @param scene Output scene.
 * @param cam Output camera.
 * @return true on success; throws on I/O or parse errors.
 */
bool load_scene_from_json(const std::string& filename, Scene& scene, Camera& cam) {
    std::ifstream ifs(filename);
    if (!ifs) throw std::runtime_error("Cannot open JSON file: " + filename);
    std::ostringstream ss; ss << ifs.rdbuf();
    return load_scene_from_json_text(ss.str(), scene, cam);
}

} // namespace jsonio
