#include "json_loader.h"

#include <stdexcept>
#include <fstream>
#include <sstream>
#include <memory>
#include <vector>
#include <cmath>
#include <algorithm>

// Only your existing headers:
#include "core.h"       // Color, Point3, Dir3, Material, etc.
#include "scene.h"      // Scene (ambient, medium_index, recursion_limit, point_lights, objects)
#include "camera.h"     // Camera (set_dpi, set_dimensions, set_bottom_left, set_observer)
#include "geometry.h"   // Sphere, HalfSpace (both already exist in your project)
#include "transform.h"  // Scaling, Translation, RotationX, RotationY, RotationZ
#include "csg.h"        // CSG, CSGOp

// Forward-declare the base type so we don't include any extra headers:
struct Primitive;

// JSON library
#include <nlohmann/json.hpp>
using nlohmann::json;

namespace {

// ---------------- helpers ----------------

[[noreturn]] void fail(const std::string& msg) {
    throw std::runtime_error("json_loader: " + msg);
}

bool has(const json& j, const char* key) {
    return j.contains(key) && !j.at(key).is_null();
}

double as_number(const json& j, const char* where) {
    if (!j.is_number()) fail(std::string(where) + " must be a number");
    return j.get<double>();
}

int as_int(const json& j, const char* where) {
    if (!j.is_number_integer()) fail(std::string(where) + " must be an integer");
    return j.get<int>();
}

Color as_color3(const json& j, const char* where) {
    if (!j.is_array() || j.size() != 3) fail(std::string(where) + " must be [r,g,b]");
    return Color{ j[0].get<double>(), j[1].get<double>(), j[2].get<double>() };
}

Point3 as_point3(const json& j, const char* where) {
    if (!j.is_array() || j.size() != 3) fail(std::string(where) + " must be [x,y,z]");
    return Point3{ j[0].get<double>(), j[1].get<double>(), j[2].get<double>() };
}

Dir3 as_dir3(const json& j, const char* where) {
    if (!j.is_array() || j.size() != 3) fail(std::string(where) + " must be [x,y,z]");
    return Dir3{ j[0].get<double>(), j[1].get<double>(), j[2].get<double>() };
}

Vec3 as_vec3(const json& j, const char* where) {
    if (!j.is_array() || j.size() != 3) fail(std::string(where) + " must be [x,y,z]");
    return Vec3{ j[0].get<double>(), j[1].get<double>(), j[2].get<double>() };
}

// Materials must outlive primitives; store them here.
std::vector<std::unique_ptr<Material>>& materials_store() {
    static std::vector<std::unique_ptr<Material>> store;
    return store;
}

const Material* make_material_from_color_block(const json& color)
{
    auto m = std::make_unique<Material>();

    if (has(color, "diffuse")) {
        m->albedo = as_color3(color.at("diffuse"), "color.diffuse");
    }
    if (has(color, "specular")) {
        auto kscol = as_color3(color.at("specular"), "color.specular");
        m->ks = (kscol.r + kscol.g + kscol.b) / 3.0;
    }
    if (has(color, "shininess"))
        m->shininess = as_number(color.at("shininess"), "color.shininess");
    if (has(color, "kd"))
        m->kd = as_number(color.at("kd"), "color.kd");

    const Material* ptr = m.get();
    materials_store().push_back(std::move(m));
    return ptr;
}

// forward decl
std::unique_ptr<Primitive> parse_object_node(const json& j);

// ---------------- primitives ----------------

std::unique_ptr<Primitive> parse_sphere(const json& s)
{
    if (!has(s, "position")) fail("sphere.position missing");
    if (!has(s, "radius"))   fail("sphere.radius missing");
    if (!has(s, "color"))    fail("sphere.color missing");
    if (!has(s, "index"))    fail("sphere.index missing");

    Point3 c  = as_point3(s.at("position"), "sphere.position");
    double r  = as_number (s.at("radius"),   "sphere.radius");
    int    ix = as_int    (s.at("index"),    "sphere.index");

    const Material* mat = make_material_from_color_block(s.at("color"));

    auto sp = std::make_unique<Sphere>(c, r, ix);
    sp->mat = mat;
    return sp;
}

std::unique_ptr<Primitive> parse_halfspace(const json& h)
{
    if (!has(h, "position")) fail("halfSpace.position missing");
    if (!has(h, "normal"))   fail("halfSpace.normal missing");
    if (!has(h, "color"))    fail("halfSpace.color missing");
    if (!has(h, "index"))    fail("halfSpace.index missing");

    Point3 p0 = as_point3(h.at("position"), "halfSpace.position");
    Dir3   n  = as_dir3  (h.at("normal"),   "halfSpace.normal");
    int    ix = as_int   (h.at("index"),    "halfSpace.index");

    const Material* mat = make_material_from_color_block(h.at("color"));

    auto hs = std::make_unique<HalfSpace>(p0, n, ix);
    hs->mat = mat;
    return hs;
}

// ---------------- transforms ----------------

std::unique_ptr<Primitive> parse_scaling(const json& node)
{
    if (!has(node, "factors")) fail("scaling.factors missing");
    if (!has(node, "subject")) fail("scaling.subject missing");

    Vec3 s = as_vec3(node.at("factors"), "scaling.factors");
    auto subject = parse_object_node(node.at("subject"));
    return std::make_unique<Scaling>(std::move(subject), s.x, s.y, s.z);
}

std::unique_ptr<Primitive> parse_translation(const json& node)
{
    if (!has(node, "factors")) fail("translation.factors missing");
    if (!has(node, "subject")) fail("translation.subject missing");

    Vec3 t = as_vec3(node.at("factors"), "translation.factors");
    auto subject = parse_object_node(node.at("subject"));
    return std::make_unique<Translation>(std::move(subject), t.x, t.y, t.z);
}

std::unique_ptr<Primitive> parse_rotation(const json& node)
{
    if (!has(node, "angle"))     fail("rotation.angle (degrees) missing");
    if (!has(node, "direction")) fail("rotation.direction missing");
    if (!has(node, "subject"))   fail("rotation.subject missing");

    double deg = as_number(node.at("angle"),     "rotation.angle");
    int    ax  = as_int   (node.at("direction"), "rotation.direction");
    double rad = deg * M_PI / 180.0;

    auto subject = parse_object_node(node.at("subject"));

    switch (ax) {
        case 0:  return std::make_unique<RotationX>(std::move(subject), rad);
        case 1:  return std::make_unique<RotationY>(std::move(subject), rad);
        case 2:  // some docs list 3 for Z; accept both
        case 3:  return std::make_unique<RotationZ>(std::move(subject), rad);
        default: fail("rotation.direction must be 0 (x), 1 (y), 2 or 3 (z)");
    }
    return nullptr;
}

// ---------------- CSG (N-ary fold) ----------------

std::unique_ptr<Primitive> fold_nary_csg(CSGOp op, const json& arr)
{
    if (!arr.is_array() || arr.empty())
        fail("CSG node expects a non-empty array");
    std::unique_ptr<Primitive> acc = parse_object_node(arr[0]);
    for (size_t i = 1; i < arr.size(); ++i) {
        auto rhs = parse_object_node(arr[i]);
        acc = std::make_unique<CSG>(op, std::move(acc), std::move(rhs));
    }
    return acc;
}

std::unique_ptr<Primitive> parse_union(const json& u) {
    if (!u.is_array()) fail("union must be an array");
    return fold_nary_csg(CSGOp::Union, u);
}
std::unique_ptr<Primitive> parse_intersection(const json& u) {
    if (!u.is_array()) fail("intersection must be an array");
    return fold_nary_csg(CSGOp::Intersection, u);
}
std::unique_ptr<Primitive> parse_difference(const json& u) {
    if (!u.is_array()) fail("difference must be an array");
    return fold_nary_csg(CSGOp::Difference, u);
}

// ---------------- dispatcher for a single object node ----------------

std::unique_ptr<Primitive> parse_object_node(const json& j)
{
    if (!j.is_object() || j.size() != 1)
        fail("each object node must be a single-key object");

    const auto key = j.begin().key();
    const json& val = j.begin().value();

    if (key == "sphere")       return parse_sphere(val);
    if (key == "halfSpace")    return parse_halfspace(val);

    if (key == "scaling")      return parse_scaling(val);
    if (key == "translation")  return parse_translation(val);
    if (key == "rotation")     return parse_rotation(val);

    if (key == "union")        return parse_union(val);
    if (key == "intersection") return parse_intersection(val);
    if (key == "difference")   return parse_difference(val);

    fail("unknown object node key: " + key);
    return nullptr;
}

// ---------------- top-level blocks ----------------

void parse_screen(const json& root, Camera& cam)
{
    if (!has(root, "screen")) fail("top-level 'screen' missing");
    const auto& s = root.at("screen");

    if (!has(s, "dpi"))        fail("screen.dpi missing");
    if (!has(s, "dimensions")) fail("screen.dimensions missing");
    if (!has(s, "position"))   fail("screen.position missing");
    if (!has(s, "observer"))   fail("screen.observer missing");

    int dpi = as_int(s.at("dpi"), "screen.dpi");
    Vec3 dims = as_vec3(s.at("dimensions"), "screen.dimensions"); // [Lx, Ly, _]
    double Lx = dims.x, Ly = dims.y;

    Point3 P = as_point3(s.at("position"), "screen.position");   // bottom-left
    Point3 C = as_point3(s.at("observer"), "screen.observer");   // eye

    cam.set_dpi(dpi);
    cam.set_dimensions(Lx, Ly);
    cam.set_bottom_left(P);
    cam.set_observer(C);
}

void parse_medium(const json& root, Scene& scene)
{
    if (!has(root, "medium")) fail("top-level 'medium' missing");
    const auto& m = root.at("medium");

    if (!has(m, "ambient"))   fail("medium.ambient missing");
    if (!has(m, "index"))     fail("medium.index missing");
    if (!has(m, "recursion")) fail("medium.recursion missing");

    scene.ambient         = as_color3(m.at("ambient"), "medium.ambient");
    scene.medium_index    = as_number(m.at("index"), "medium.index");
    scene.recursion_limit = as_int   (m.at("recursion"), "medium.recursion");
}

void parse_sources(const json& root, Scene& scene)
{
    if (!has(root, "sources")) fail("top-level 'sources' missing");
    const auto& arr = root.at("sources");
    if (!arr.is_array()) fail("'sources' must be an array");

    for (const auto& s : arr) {
        if (!s.is_object()) fail("sources[] entry must be an object");
        if (!has(s, "position"))  fail("source.position missing");

        Color I{};
        if (has(s, "intensity")) I = as_color3(s.at("intensity"), "source.intensity");
        else if (has(s, "I"))     I = as_color3(s.at("I"),         "source.I");
        else fail("source.intensity missing (RGB)");

        Point3 pos = as_point3(s.at("position"), "source.position");

        // Scene is expected to define:
        // struct PointLight { Point3 pos; Color intensity; };
        scene.point_lights.push_back({pos, I});
    }
}

void parse_objects(const json& root, Scene& scene)
{
    if (!has(root, "objects")) fail("top-level 'objects' missing");
    const auto& arr = root.at("objects");
    if (!arr.is_array()) fail("'objects' must be an array");

    for (const auto& node : arr) {
        auto obj = parse_object_node(node);
        scene.objects.push_back(std::move(obj));
    }
}

} // namespace

namespace json_loader {

bool load_all_from_string(const std::string& json_text, Scene& scene, Camera& camera)
{
    json root;
    try {
        root = json::parse(json_text);
    } catch (const std::exception& e) {
        fail(std::string("invalid JSON: ") + e.what());
    }

    parse_screen(root, camera);
    parse_medium(root, scene);
    parse_sources(root, scene);
    parse_objects(root, scene);
    return true;
}

bool load_all_from_file(const std::string& path, Scene& scene, Camera& camera)
{
    std::ifstream in(path);
    if (!in) fail("cannot open file: " + path);
    std::ostringstream ss;
    ss << in.rdbuf();
    return load_all_from_string(ss.str(), scene, camera);
}

} // namespace json_loader
