#include "json_loader.h"

// STL
#include <vector>
#include <fstream>
#include <sstream>
#include <memory>
#include <type_traits>
#include <utility>

// Your project headers (adjust paths if needed)
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

// ------------------------ small JSON helpers ------------------------
template <typename T>
T get_or(const json& j, const char* key, const T& fallback) {
    if (!j.contains(key)) return fallback;
    return j.at(key).get<T>();
}

inline Vec3 as_vec3_from_array(const json& arr) {
    if (!arr.is_array() || arr.size() != 3) {
        throw std::runtime_error("Expected array[3].");
    }
    return Vec3(arr[0].get<double>(), arr[1].get<double>(), arr[2].get<double>());
}

inline Vec3 as_vecN_from_array_len2_or3(const json& arr) {
    if (!arr.is_array() || (arr.size() < 2 || arr.size() > 3)) {
        throw std::runtime_error("Expected array[2] or array[3] for dimensions.");
    }
    const double x = arr[0].get<double>();
    const double y = arr[1].get<double>();
    const double z = (arr.size() == 3 ? arr[2].get<double>() : 0.0);
    return Vec3(x, y, z);
}

inline Color as_color3_array(const json& arr) {
    if (!arr.is_array() || arr.size() != 3) {
        throw std::runtime_error("Expected color array[3].");
    }
    return Color(arr[0].get<double>(), arr[1].get<double>(), arr[2].get<double>());
}

inline double as_number(const json& j) {
    if (!j.is_number()) throw std::runtime_error("Expected number.");
    return j.get<double>();
}

// ------------------------ SFINAE setters (safe if fields don't exist) ------------------------
template <typename M, typename = void>
struct has_member_ka : std::false_type {};
template <typename M>
struct has_member_ka<M, std::void_t<decltype(std::declval<M&>().ka)> > : std::true_type {};

template <typename M, typename = void>
struct has_member_kr : std::false_type {};
template <typename M>
struct has_member_kr<M, std::void_t<decltype(std::declval<M&>().kr)> > : std::true_type {};

template <typename M, typename = void>
struct has_member_kt : std::false_type {};
template <typename M>
struct has_member_kt<M, std::void_t<decltype(std::declval<M&>().kt)> > : std::true_type {};

template <typename M, typename = void>
struct has_member_kd : std::false_type {};
template <typename M>
struct has_member_kd<M, std::void_t<decltype(std::declval<M&>().kd)> > : std::true_type {};

template <typename P, typename = void>
struct has_member_ior : std::false_type {};
template <typename P>
struct has_member_ior<P, std::void_t<decltype(std::declval<P&>().ior)> > : std::true_type {};

template <typename M>
inline void set_ka_if_exists(M& m, const Color& ka) {
    if constexpr (has_member_ka<M>::value) m.ka = ka;
}
template <typename M>
inline void set_kr_if_exists(M& m, double kr) {
    if constexpr (has_member_kr<M>::value) m.kr = kr;
}
template <typename M>
inline void set_kt_if_exists(M& m, double kt) {
    if constexpr (has_member_kt<M>::value) m.kt = kt;
}
template <typename M>
inline void set_kd_if_exists(M& m, double kd) {
    if constexpr (has_member_kd<M>::value) m.kd = kd;
}
template <typename P>
inline void set_ior_if_exists(P& p, double ior) {
    if constexpr (has_member_ior<P>::value) p.ior = ior;
}

// ------------------------ Material wiring ------------------------
Material parse_color_block_into_material(const json& jcolor) {
    Material m;

    // diffuse / albedo
    if (jcolor.contains("diffuse")) {
        m.albedo = as_color3_array(jcolor.at("diffuse"));
    } else if (jcolor.contains("albedo")) {
        m.albedo = as_color3_array(jcolor.at("albedo"));
    }

    // specular -> set ks from magnitude (keeps your existing model)
    if (jcolor.contains("specular")) {
        const Color s = as_color3_array(jcolor.at("specular"));
        m.ks = (s.x + s.y + s.z) / 3.0;
    }

    // shininess
    if (jcolor.contains("shininess")) {
        m.shininess = jcolor.at("shininess").get<double>();
    }

    // NEW: ambient/reflected/refracted — set only if Material has these fields
    if (jcolor.contains("ambient")) {
        const Color ka = as_color3_array(jcolor.at("ambient"));
        set_ka_if_exists(m, ka);
    }
    if (jcolor.contains("reflected")) {
        double kr{};
        if (jcolor.at("reflected").is_array()) {
            const Color krC = as_color3_array(jcolor.at("reflected"));
            kr = (krC.x + krC.y + krC.z) / 3.0;
        } else {
            kr = jcolor.at("reflected").get<double>();
        }
        set_kr_if_exists(m, kr);
    }
    if (jcolor.contains("refracted")) {
        double kt{};
        if (jcolor.at("refracted").is_array()) {
            const Color ktC = as_color3_array(jcolor.at("refracted"));
            kt = (ktC.x + ktC.y + ktC.z) / 3.0;
        } else {
            kt = jcolor.at("refracted").get<double>();
        }
        set_kt_if_exists(m, kt);
    }

    // optional scalar kd override if your Material has kd
    if (jcolor.contains("kd")) {
        set_kd_if_exists(m, jcolor.at("kd").get<double>());
    }

    return m;
}

// ------------------------ Screen / Medium / Sources ------------------------
void parse_screen(const json& root, Camera& cam) {
    if (!root.contains("screen")) return;
    const json& j = root.at("screen");

    const int dpi = get_or<int>(j, "dpi", 72);

    // Accept [Lx, Ly] or [Lx, Ly, _]
    Vec3 dims = Vec3(2.0, 2.0, 0.0);
    if (j.contains("dimensions")) {
        dims = as_vecN_from_array_len2_or3(j.at("dimensions"));
    }

    if (!j.contains("eye") || !j.contains("center") || !j.contains("up")) {
        throw std::runtime_error("screen requires 'eye', 'center', and 'up'.");
    }
    const Vec3 eye    = as_vec3_from_array(j.at("eye"));
    const Vec3 center = as_vec3_from_array(j.at("center"));
    const Vec3 up     = as_vec3_from_array(j.at("up"));

    // Adjust this call if your Camera ctor differs.
    cam = Camera(eye, center, up, dims.x, dims.y, dpi);
}

void parse_medium(const json& root, Scene& scene) {
    if (!root.contains("medium")) return;
    const json& jm = root.at("medium");

    if (jm.contains("ambient")) {
        // Scene must expose 'ambient' as Color.
        scene.ambient = as_color3_array(jm.at("ambient"));
    }
    if (jm.contains("index")) {
        // Scene must expose 'medium_index' (double).
        scene.medium_index = jm.at("index").get<double>();
    }
    if (jm.contains("recursion")) {
        // Scene must expose 'recursion_limit' (int).
        scene.recursion_limit = jm.at("recursion").get<int>();
    }
}

void parse_sources(const json& root, Scene& scene) {
    if (!root.contains("sources")) return;
    const json& arr = root.at("sources");
    if (!arr.is_array()) throw std::runtime_error("'sources' must be an array.");

    for (const auto& js : arr) {
        if (!js.contains("position") || !js.contains("intensity")) {
            throw std::runtime_error("each source needs 'position' and 'intensity'.");
        }
        const Vec3 pos = as_vec3_from_array(js.at("position"));
        const Color I  = as_color3_array(js.at("intensity"));

        // Assumes Scene has point_lights with an element type { Point3 pos; Color I; }
        scene.point_lights.push_back({ Point3{pos.x, pos.y, pos.z}, I });
    }
}

// ------------------------ Object parsing ------------------------
std::shared_ptr<Primitive> parse_object_node(const json& jnode);

Material parse_material_from_node(const json& jnode) {
    if (!jnode.contains("color")) {
        Material m;
        m.albedo = Color(0.8, 0.8, 0.8);
        return m;
    }
    return parse_color_block_into_material(jnode.at("color"));
}

std::shared_ptr<Primitive> make_sphere(const json& jnode) {
    if (!jnode.contains("position") || !jnode.contains("radius") || !jnode.contains("index") || !jnode.contains("color")) {
        throw std::runtime_error("sphere requires 'position', 'radius', 'index', and 'color'.");
    }
    const Vec3 c = as_vec3_from_array(jnode.at("position"));
    const double r = jnode.at("radius").get<double>();
    Material mat = parse_material_from_node(jnode);

    auto s = std::make_shared<Sphere>(Point3{c.x, c.y, c.z}, r);
    s->mat = std::make_shared<Material>(mat);

    // set per-primitive ior only if the field exists
    set_ior_if_exists(*s, jnode.at("index").get<double>());

    return s;
}

std::shared_ptr<Primitive> make_halfspace(const json& jnode) {
    if (!jnode.contains("position") || !jnode.contains("normal") || !jnode.contains("index") || !jnode.contains("color")) {
        throw std::runtime_error("halfspace requires 'position', 'normal', 'index', and 'color'.");
    }
    const Vec3 p0 = as_vec3_from_array(jnode.at("position"));
    const Vec3 n  = as_vec3_from_array(jnode.at("normal"));
    Material mat = parse_material_from_node(jnode);

    auto h = std::make_shared<HalfSpace>(Point3{p0.x, p0.y, p0.z}, Dir3{n.x, n.y, n.z}.normalized());
    h->mat = std::make_shared<Material>(mat);

    set_ior_if_exists(*h, jnode.at("index").get<double>());

    return h;
}

std::shared_ptr<Primitive> make_transform(const json& jnode) {
    if (!jnode.contains("operation") || !jnode.contains("object")) {
        throw std::runtime_error("transform requires 'operation' and nested 'object'.");
    }
    auto child = parse_object_node(jnode.at("object"));
    if (!child) return nullptr;

    const std::string op = jnode.at("operation").get<std::string>();
    if (op == "translation") {
        const Vec3 t = as_vec3_from_array(jnode.at("value"));
        return std::make_shared<Translation>(child, Vec3{t.x, t.y, t.z});
    } else if (op == "scaling") {
        const Vec3 s = as_vec3_from_array(jnode.at("value"));
        return std::make_shared<Scaling>(child, Vec3{s.x, s.y, s.z});
    } else if (op == "rotation") {
        const json& v = jnode.at("value");
        if (!v.is_array() || v.size() != 2) {
            throw std::runtime_error("rotation.value must be [axis, angle_deg].");
        }
        const int axis = v[0].get<int>();       // 0=x, 1=y, 2(or 3)=z
        const double angle = v[1].get<double>();
        return std::make_shared<Rotation>(child, axis, angle);
    }
    throw std::runtime_error("unknown transform operation: " + op);
}

std::shared_ptr<Primitive> make_csg(const json& jnode) {
    if (!jnode.contains("operator") || !jnode.contains("left") || !jnode.contains("right")) {
        throw std::runtime_error("csg requires 'operator', 'left', 'right'.");
    }
    auto A = parse_object_node(jnode.at("left"));
    auto B = parse_object_node(jnode.at("right"));
    const std::string op = jnode.at("operator").get<std::string>();

    CSGOp csgop;
    if      (op == "union")        csgop = CSGOp::Union;
    else if (op == "intersection") csgop = CSGOp::Intersection;
    else if (op == "difference")   csgop = CSGOp::Difference;
    else throw std::runtime_error("unknown csg operator: " + op);

    return std::make_shared<CSG>(A, B, csgop);
}

std::shared_ptr<Primitive> parse_object_node(const json& jnode) {
    // Each object is a single-key dict: {"sphere": {...}} etc.
    if (!jnode.is_object() || jnode.size() != 1) {
        throw std::runtime_error("each object node must be a one-entry object");
    }
    const auto it = jnode.begin();
    const std::string kind = it.key();
    const json& value = it.value();

    if (kind == "sphere")     return make_sphere(value);
    if (kind == "halfspace")  return make_halfspace(value);
    if (kind == "transform")  return make_transform(value);
    if (kind == "csg")        return make_csg(value);

    throw std::runtime_error("unknown object kind: " + kind);
}

} // anonymous namespace

// ------------------------ public API ------------------------
namespace jsonio {

Material make_material_from_color_block(const void* json_color_any) {
    const json& j = *reinterpret_cast<const json*>(json_color_any);
    return parse_color_block_into_material(j);
}

bool load_scene_from_json_text(const std::string& json_text, Scene& scene, Camera& cam) {
    json j = json::parse(json_text);

    parse_screen(j, cam);
    parse_medium(j, scene);
    parse_sources(j, scene);

    if (j.contains("objects")) {
        const json& arr = j.at("objects");
        if (!arr.is_array()) throw std::runtime_error("'objects' must be an array.");
        for (const auto& node : arr) {
            auto prim = parse_object_node(node);
            if (prim) scene.objects.push_back(std::move(prim));
        }
    }

    return true;
}

bool load_scene_from_json(const std::string& filename, Scene& scene, Camera& cam) {
    std::ifstream ifs(filename);
    if (!ifs) {
        throw std::runtime_error("Cannot open JSON file: " + filename);
    }
    std::ostringstream ss;
    ss << ifs.rdbuf();
    return load_scene_from_json_text(ss.str(), scene, cam);
}

} // namespace jsonio
