#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <stdexcept>
#include <sstream>

// --- Project headers
#include "core.h"
#include "camera.h"
#include "scene.h"
#include "geometry.h"
#include "transform.h"
#include "csg.h"
#include "tracer.h"
#include "shading.h"

// --- JSON (nlohmann)
#include <nlohmann/json.hpp>
using nlohmann::json;

// --- OpenCV (PNG required)
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

// ----------------- helpers -----------------
static std::string slurp(const std::string& path){
    std::ifstream ifs(path, std::ios::binary);
    if(!ifs) throw std::runtime_error("Cannot open file: " + path);
    std::ostringstream ss; ss << ifs.rdbuf(); return ss.str();
}

static inline unsigned char toByte(double x) {
    x = std::max(0.0, std::min(1.0, x));
    return static_cast<unsigned char>(std::lround(x * 255.0));
}

static void write_png(const std::string& outPath, int W, int H, const std::vector<Color>& fb) {
    // Flip vertically so (0,0) ends up top-left in the PNG
    cv::Mat img(H, W, CV_8UC3);
    for (int y = 0; y < H; ++y) {
        int fy = H - 1 - y;
        for (int x = 0; x < W; ++x) {
            const Color& c = fb[size_t(fy) * W + x];
            img.at<cv::Vec3b>(y, x) = cv::Vec3b(
                toByte(c.b), toByte(c.g), toByte(c.r) // BGR
            );
        }
    }
    if (!cv::imwrite(outPath, img)) {
        throw std::runtime_error("cv::imwrite failed for: " + outPath);
    }
}

// --------------- materials -----------------
static Material parse_material(const json& jcolor) {
    Material m; // defaults in geometry.h / shading model
    auto read_color = [&](const char* key, Color& out, bool& has){
        if (jcolor.contains(key)) {
            const auto& a = jcolor.at(key);
            if (!a.is_array() || a.size()!=3) throw std::runtime_error(std::string("color.")+key+" must be [r,g,b]");
            out = Color(a[0].get<double>(), a[1].get<double>(), a[2].get<double>());
            has = true;
        }
    };
    bool hasDiffuse=false, hasSpec=false;
    Color diffuse, spec;

    read_color("diffuse", diffuse, hasDiffuse);
    read_color("albedo",  diffuse, hasDiffuse); // alias
    read_color("specular", spec,    hasSpec);

    if (hasDiffuse) m.albedo = diffuse;
    if (hasSpec)    m.ks = (spec.r + spec.g + spec.b) / 3.0;
    if (jcolor.contains("shininess")) m.shininess = jcolor.at("shininess").get<double>();
    if (jcolor.contains("kd"))        m.kd        = jcolor.at("kd").get<double>();
    return m;
}

// forward decl
static std::shared_ptr<Primitive> parse_object_node(const json& node);

static std::shared_ptr<Primitive> fold_csg_list(const std::string& op, const json& arr) {
    if (!arr.is_array() || arr.empty()) throw std::runtime_error(op + " must be a non-empty array");
    auto acc = parse_object_node(arr[0]);
    for (size_t i=1; i<arr.size(); ++i) {
        auto rhs = parse_object_node(arr[i]);
        CSGOp cop;
        if (op=="union") cop = CSGOp::Union;
        else if (op=="intersection") cop = CSGOp::Intersection;
        else if (op=="difference") cop = CSGOp::Difference;
        else throw std::runtime_error("unknown CSG op: " + op);
        acc = std::make_shared<CSG>(cop, acc, rhs);
    }
    return acc;
}

static std::shared_ptr<Primitive> parse_transform(const std::string& kind, const json& j) {
    if (!j.contains("subject")) throw std::runtime_error(kind + " requires 'subject'");
    auto child = parse_object_node(j.at("subject"));
    if (!child) return nullptr;

    if (kind == "translation") {
        const auto& f = j.at("factors");
        if (!f.is_array() || f.size()!=3) throw std::runtime_error("translation.factors must be [tx,ty,tz]");
        Vec3 t(f[0].get<double>(), f[1].get<double>(), f[2].get<double>());
        return std::make_shared<Translation>(child, t);
    }
    if (kind == "scaling") {
        const auto& f = j.at("factors");
        if (!f.is_array() || f.size()!=3) throw std::runtime_error("scaling.factors must be [sx,sy,sz]");
        Vec3 s(f[0].get<double>(), f[1].get<double>(), f[2].get<double>());
        return std::make_shared<Scaling>(child, s);
    }
    if (kind == "rotation") {
        if (!j.contains("angle") || !j.contains("direction")) {
            throw std::runtime_error("rotation requires 'angle' (deg) and 'direction' (0=x,1=y,2/3=z)");
        }
        double ang_deg = j.at("angle").get<double>();
        int ax = j.at("direction").get<int>();
        Axis axis = (ax==0)?Axis::X : (ax==1)?Axis::Y : Axis::Z;
        double ang_rad = ang_deg * M_PI / 180.0;
        return std::make_shared<Rotation>(child, axis, ang_rad);
    }
    throw std::runtime_error("unknown transform kind: " + kind);
}

static std::shared_ptr<Primitive> parse_object_node(const json& node) {
    if (!node.is_object() || node.size()!=1) throw std::runtime_error("object node must be a single-key dict");
    const auto it = node.begin();
    const std::string kind = it.key();
    const json& j = it.value();

    // Materials are stored to keep lifetime > primitives
    static std::vector<std::unique_ptr<Material>> MAT;

    if (kind == "sphere") {
        if (!j.contains("position") || !j.contains("radius") || !j.contains("color") || !j.contains("index"))
            throw std::runtime_error("sphere requires position, radius, color, index");
        const auto& p = j.at("position");
        if (!p.is_array() || p.size()!=3) throw std::runtime_error("sphere.position must be [x,y,z]");
        Point3 c(p[0].get<double>(), p[1].get<double>(), p[2].get<double>());
        double r = j.at("radius").get<double>();
        Material m = parse_material(j.at("color"));
        MAT.emplace_back(std::make_unique<Material>(m));
        auto s = std::make_shared<Sphere>(c, r, nullptr);
        s->mat = MAT.back().get();
        (void)j.at("index");
        return s;
    }
    if (kind == "halfSpace" || kind == "halfspace") {
        if (!j.contains("position") || !j.contains("normal") || !j.contains("color") || !j.contains("index"))
            throw std::runtime_error("halfSpace requires position, normal, color, index");
        const auto& p0 = j.at("position");
        const auto& n  = j.at("normal");
        if (!p0.is_array() || p0.size()!=3) throw std::runtime_error("halfSpace.position must be [x,y,z]");
        if (!n.is_array()  || n.size()!=3)  throw std::runtime_error("halfSpace.normal must be [nx,ny,nz]");
        Point3 P0(p0[0].get<double>(), p0[1].get<double>(), p0[2].get<double>());
        Dir3   N (n[0].get<double>(),  n[1].get<double>(),  n[2].get<double>());
        Material m = parse_material(j.at("color"));
        MAT.emplace_back(std::make_unique<Material>(m));
        auto h = std::make_shared<HalfSpace>(P0, N, nullptr);
        h->mat = MAT.back().get();
        (void)j.at("index");
        return h;
    }
    if (kind == "translation" || kind=="scaling" || kind=="rotation") {
        return parse_transform(kind, j);
    }
    if (kind == "union" || kind == "intersection" || kind == "difference") {
        return fold_csg_list(kind, j);
    }
    throw std::runtime_error("unknown object kind: " + kind);
}

// --------------- scene blocks ----------------
static void parse_screen(const json& root, Camera& cam) {
    if (!root.contains("screen")) throw std::runtime_error("missing 'screen'");
    const json& s = root.at("screen");

    cam.screen.dpi = s.at("dpi").get<int>();

    const auto& dims = s.at("dimensions");
    if (!dims.is_array() || dims.size()!=2) throw std::runtime_error("screen.dimensions must be [Lx,Ly]");
    cam.screen.Lx = dims[0].get<double>();
    cam.screen.Ly = dims[1].get<double>();

    const auto& pos = s.at("position");
    if (!pos.is_array() || pos.size()!=3) throw std::runtime_error("screen.position must be [x,y,z]");
    cam.screen.P = Point3(pos[0].get<double>(), pos[1].get<double>(), pos[2].get<double>());

    const auto& obs = s.at("observer");
    if (!obs.is_array() || obs.size()!=3) throw std::runtime_error("screen.observer must be [x,y,z]");
    cam.eye = Point3(obs[0].get<double>(), obs[1].get<double>(), obs[2].get<double>());
}

static void parse_medium(const json& root, Scene& scene) {
    if (!root.contains("medium")) return;
    const auto& m = root.at("medium");
    if (m.contains("ambient")) {
        const auto& a = m.at("ambient");
        if (!a.is_array() || a.size()!=3) throw std::runtime_error("medium.ambient must be [r,g,b]");
        scene.ambient = Color(a[0].get<double>(), a[1].get<double>(), a[2].get<double>());
    }
    if (m.contains("index"))     scene.medium_index   = m.at("index").get<double>();
    if (m.contains("recursion")) scene.recursion_limit = m.at("recursion").get<int>();
}

static void parse_sources(const json& root, Scene& scene) {
    if (!root.contains("sources")) return;
    const auto& arr = root.at("sources");
    if (!arr.is_array()) throw std::runtime_error("'sources' must be an array");
    for (const auto& s : arr) {
        const auto& p = s.at("position");
        const auto& I = s.at("intensity");
        if (!p.is_array() || p.size()!=3) throw std::runtime_error("source.position must be [x,y,z]");
        if (!I.is_array() || I.size()!=3) throw std::runtime_error("source.intensity must be [r,g,b]");
        PointLight L;
        L.pos = Point3(p[0].get<double>(), p[1].get<double>(), p[2].get<double>());
        L.intensity = Color(I[0].get<double>(), I[1].get<double>(), I[2].get<double>());
        scene.point_lights.push_back(L);
    }
}


int main(int argc, char** argv) {
    try {
        if (argc < 3) {
            std::cerr << "Usage: " << argv[0] << " <scene.json> <output.png>\n";
            std::cerr << "Note: OpenCV is required and PNG is the only output format.\n";
            return 1;
        }
        const std::string inJson  = argv[1];
        const std::string outPath = argv[2];

        json root = json::parse(slurp(inJson));

        Camera cam;
        parse_screen(root, cam);

        Scene scene;
        parse_medium(root, scene);
        parse_sources(root, scene);

        if (root.contains("background")) {
            const auto& b = root.at("background");
            if (b.is_array() && b.size()==3) {
                scene.background = Color(b[0].get<double>(), b[1].get<double>(), b[2].get<double>());
            }
        }

        // objects
        std::vector<std::shared_ptr<Primitive>> keep_alive;
        if (root.contains("objects")) {
            const auto& objs = root.at("objects");
            if (!objs.is_array()) throw std::runtime_error("'objects' must be an array");
            for (const auto& node : objs) {
                auto prim = parse_object_node(node);
                if (prim) {
                    keep_alive.push_back(prim);
                    scene.objects.push_back(keep_alive.back().get());
                }
            }
        }

        const int W = cam.screen.nx();
        const int H = cam.screen.ny();

        Tracer tracer;
        tracer.scene  = &scene;
        tracer.camera = &cam;
        tracer.width  = W;
        tracer.height = H;

        std::vector<Color> framebuffer;
        tracer.render(framebuffer);

        write_png(outPath, W, H, framebuffer);
        std::cout << "Wrote PNG: " << outPath << "\n";
        return 0;

    } catch (const std::exception& e) {
        std::cerr << "[error] " << e.what() << "\n";
        return 2;
    }
}
