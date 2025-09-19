#include <opencv2/opencv.hpp>
#include <filesystem>
#include <cmath>
#include <vector>
#include <limits>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <sstream>
#include <memory>
#include <cctype>

constexpr double EPS = 1e-6;

struct Vec3 {
    double x{}, y{}, z{};
    Vec3() = default;
    Vec3(double X, double Y, double Z) : x(X), y(Y), z(Z) {}
    Vec3 operator+(const Vec3& o) const { return {x+o.x, y+o.y, z+o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x-o.x, y-o.y, z-o.z}; }
    Vec3 operator*(double s) const { return {x*s, y*s, z*s}; }
    Vec3 operator/(double s) const { return {x/s, y/s, z/s}; }
    Vec3& operator+=(const Vec3& o){ x+=o.x; y+=o.y; z+=o.z; return *this; }
    static double dot(const Vec3& a, const Vec3& b){ return a.x*b.x + a.y*b.y + a.z*b.z; }
    static Vec3 cross(const Vec3& a, const Vec3& b){
        return { a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x };
    }
    double norm() const { return std::sqrt(x*x+y*y+z*z); }
    Vec3 normalized() const { double n = norm(); return (n>0)?(*this)/n:*this; }
};

struct Color {
    double r{}, g{}, b{}; // 0..1
    Color() = default;
    Color(double R, double G, double B) : r(R), g(G), b(B) {}
    Color operator+(const Color& o) const { return {r+o.r, g+o.g, b+o.b}; }
    Color operator*(double s) const { return {r*s, g*s, b*s}; }
    Color& operator+=(const Color& o){ r+=o.r; g+=o.g; b+=o.b; return *this; }
    static Color hadamard(const Color& a, const Color& b){ return {a.r*b.r, a.g*b.g, a.b*b.b}; }
    void clamp(){ r = std::clamp(r,0.0,1.0); g = std::clamp(g,0.0,1.0); b = std::clamp(b,0.0,1.0); }
};

struct Ray { Vec3 origin; Vec3 dir; }; // dir should be normalized

struct Material {
    Color Ka{0.05, 0.05, 0.05}; // ambient
    Color Kd{0.7, 0.7, 0.7};    // diffuse
    Color Ks{0.3, 0.3, 0.3};    // specular
    double shininess{32.0};
};

struct Light {
    Vec3 pos;
    Color intensity{1.0, 1.0, 1.0}; // per-channel intensity (0..1)
};

struct Hit {
    double t{std::numeric_limits<double>::infinity()};
    Vec3 p, n;
    Material mat;
    bool hit{false};
};

struct Object {
    Material mat;
    virtual ~Object() = default;
    virtual Hit intersect(const Ray& ray) const = 0;
};

struct Sphere : Object {
    Vec3 c; double r;
    Sphere(const Vec3& C, double R, const Material& M){ c=C; r=R; mat=M; }
    Hit intersect(const Ray& ray) const override {
        Vec3 oc = ray.origin - c;
        double a = Vec3::dot(ray.dir, ray.dir);
        double b = 2.0 * Vec3::dot(oc, ray.dir);
        double cQ = Vec3::dot(oc, oc) - r*r;
        double disc = b*b - 4*a*cQ;
        Hit h;
        if (disc < 0) return h;
        double s = std::sqrt(disc);
        double t1 = (-b - s) / (2*a);
        double t2 = (-b + s) / (2*a);
        double t = (t1 > EPS) ? t1 : ((t2 > EPS) ? t2 : std::numeric_limits<double>::infinity());
        if (!std::isfinite(t)) return h;
        h.t = t;
        h.p = ray.origin + ray.dir * t;
        h.n = (h.p - c).normalized();
        h.mat = mat;
        h.hit = true;
        return h;
    }
};

struct Plane : Object {
    // Plane: dot(n, p) + d = 0 (n normalized)
    Vec3 n; double d;
    Plane(const Vec3& N, double D, const Material& M){ n=N.normalized(); d=D; mat=M; }
    Hit intersect(const Ray& ray) const override {
        Hit h;
        double denom = Vec3::dot(n, ray.dir);
        if (std::fabs(denom) < 1e-9) return h; // parallel
        double t = -(Vec3::dot(n, ray.origin) + d) / denom;
        if (t <= EPS) return h;
        h.t = t;
        h.p = ray.origin + ray.dir * t;
        h.n = n;
        h.mat = mat;
        h.hit = true;
        return h;
    }
};

struct CheckeredPlane : Plane {
    Color a{0.9,0.9,0.9}, b{0.2,0.2,0.2};
    double tile{1.0};
    Vec3 u, v;      // tangent basis
    Vec3 p0;        // any fixed point on the plane (anchor)

    CheckeredPlane(const Vec3& N, double D, Color A, Color B, double tileSize)
        : Plane(N, D, /*unused base mat*/ Material{}),
          a(A), b(B), tile(tileSize > 0 ? tileSize : 1.0)
    {
        // Orthonormal basis on plane
        Vec3 any = (std::fabs(n.x) < 0.9) ? Vec3{1,0,0} : Vec3{0,1,0};
        u = Vec3::cross(n, any).normalized();
        v = Vec3::cross(n, u);
        // A consistent origin point on the plane: dot(n, p0) + d = 0  ->  p0 = -d * n
        p0 = n * (-d);
    }

    Hit intersect(const Ray& ray) const override {
        Hit h = Plane::intersect(ray);
        if (!h.hit) return h;

        // Use anchored local coordinates to avoid artifacts
        Vec3 local = h.p - p0;
        double U = Vec3::dot(local, u) / tile;
        double V = Vec3::dot(local, v) / tile;
        bool odd = ((int)std::floor(U) + (int)std::floor(V)) & 1;
        Color c = odd ? b : a;

        h.mat.Kd = c;
        h.mat.Ka = {0.05*c.r, 0.05*c.g, 0.05*c.b};
        h.mat.Ks = {0,0,0};
        h.mat.shininess = 1.0;
        return h;
    }
};


struct Scene {
    std::vector<const Object*> objects;            // raw ptrs into owned_objects
    std::vector<std::unique_ptr<Object>> owned;    // ownership lives here
    std::vector<Light> lights;
    Color ambient{0.1, 0.1, 0.1};

    bool occluded(const Vec3& p, const Vec3& toLight, double maxDist) const {
        Ray shadowRay{ p + toLight * EPS, toLight };
        for (const auto* obj : objects) {
            Hit h = obj->intersect(shadowRay);
            if (h.hit && h.t < maxDist - EPS) return true;
        }
        return false;
    }

    Color shade(const Ray& ray, const Hit& h) const {
        Color out = Color::hadamard(h.mat.Ka, ambient); // ambient
        Vec3 V = (ray.origin - h.p).normalized();
        for (const auto& L : lights) {
            Vec3 Ldir = (L.pos - h.p);
            double dist = Ldir.norm();
            Ldir = Ldir / dist;
            if (occluded(h.p, Ldir, dist)) continue;
            double ndotl = std::max(0.0, Vec3::dot(h.n, Ldir));
            Color diffuse = Color::hadamard(h.mat.Kd, L.intensity) * ndotl;
            Vec3 H = (Ldir + V).normalized();           // Blinn-Phong
            double ndoth = std::max(0.0, Vec3::dot(h.n, H));
            Color spec = Color::hadamard(h.mat.Ks, L.intensity) * std::pow(ndoth, h.mat.shininess);
            out += diffuse + spec;
        }
        out.clamp();
        return out;
    }

    Color trace(const Ray& ray) const {
        Hit best;
        for (const auto* obj : objects) {
            Hit h = obj->intersect(ray);
            if (h.hit && h.t < best.t) best = h;
        }
        if (!best.hit) return {0.0, 0.0, 0.0};
        return shade(ray, best);
    }
};

// -------------------- Parsing --------------------

static std::string trim_leading(const std::string& s){
    size_t i=0; while(i<s.size() && std::isspace(static_cast<unsigned char>(s[i]))) ++i; return s.substr(i);
}

bool load_scene_from_file(const std::string& path,
                          int& width, int& height,
                          double& fov_deg,
                          Vec3& camPos,
                          Scene& scene)
{
    // defaults...
    width = 800; height = 600; fov_deg = 60.0; camPos = {0,1,3};
    scene.ambient = {0.1,0.1,0.12};
    scene.lights.clear(); scene.owned.clear(); scene.objects.clear();

    std::ifstream in(path);
    if(!in) { std::cerr << "Cannot open scene file: " << path << "\n"; return false; }

    std::string line;
    int ln = 0;
    while (std::getline(in, line)) {
        ++ln;
        auto hash = line.find('#');
        if (hash != std::string::npos) line = line.substr(0, hash);
        line = trim_leading(line);
        if (line.empty()) continue;
        std::istringstream ss(line);
        std::string kw; ss >> kw;
        if (kw.empty()) continue;

        if (kw == "image") {
            ss >> width >> height;
            if (!ss) { std::cerr << "Parse error (image) line " << ln << "\n"; return false; }
        } else if (kw == "fov") {
            ss >> fov_deg;
            if (!ss) { std::cerr << "Parse error (fov) line " << ln << "\n"; return false; }
        } else if (kw == "camera") {
            ss >> camPos.x >> camPos.y >> camPos.z;
            if (!ss) { std::cerr << "Parse error (camera) line " << ln << "\n"; return false; }
        } else if (kw == "ambient") {
            ss >> scene.ambient.r >> scene.ambient.g >> scene.ambient.b;
            if (!ss) { std::cerr << "Parse error (ambient) line " << ln << "\n"; return false; }
        } else if (kw == "light") {
            Light L;
            ss >> L.pos.x >> L.pos.y >> L.pos.z >> L.intensity.r >> L.intensity.g >> L.intensity.b;
            if (!ss) { std::cerr << "Parse error (light) line " << ln << "\n"; return false; }
            scene.lights.push_back(L);
        } else if (kw == "sphere") {
            Vec3 c; double r; Color col; double shiny=32.0;
            ss >> c.x >> c.y >> c.z >> r >> col.r >> col.g >> col.b >> shiny;
            if (!ss) { std::cerr << "Parse error (sphere) line " << ln << "\n"; return false; }
            Material m;
            m.Kd = col;
            m.Ka = {0.05*col.r, 0.05*col.g, 0.05*col.b};
            m.Ks = {0.4, 0.4, 0.4};
            m.shininess = shiny;
            auto sp = std::make_unique<Sphere>(c, r, m);
            scene.objects.push_back(sp.get());
            scene.owned.push_back(std::move(sp));
        } else if (kw == "plane") {
            Vec3 n; double d; Color col; double shiny=8.0;
            ss >> n.x >> n.y >> n.z >> d >> col.r >> col.g >> col.b >> shiny;
            if (!ss) { std::cerr << "Parse error (plane) line " << ln << "\n"; return false; }
            Material m;
            m.Kd = col;
            m.Ka = {0.05*col.r, 0.05*col.g, 0.05*col.b};
            m.Ks = {0.2, 0.2, 0.2};
            m.shininess = shiny;
            auto pl = std::make_unique<Plane>(n, d, m);
            scene.objects.push_back(pl.get());
            scene.owned.push_back(std::move(pl));
        } else if (kw == "checkerplane") {
            // <-- new branch
            Vec3 n; double d; Color a, b; double size;
            ss >> n.x >> n.y >> n.z >> d
               >> a.r >> a.g >> a.b
               >> b.r >> b.g >> b.b
               >> size;
            if (!ss) { std::cerr << "Parse error (checkerplane) line " << ln << "\n"; return false; }
            auto cp = std::make_unique<CheckeredPlane>(n, d, a, b, size);
            scene.objects.push_back(cp.get());
            scene.owned.push_back(std::move(cp));
        } else {
            std::cerr << "Unknown keyword '" << kw << "' at line " << ln << "\n";
            return false;
        }
    }
    if (scene.lights.empty()) {
        std::cerr << "Warning: no lights defined; image will be black.\n";
    }
    return true;
}


// -------------------- Main --------------------

int main(int argc, char** argv) {
    const std::string scene_path = (argc > 1) ? argv[1] : "scene.txt";

    int width=800, height=600;
    double fov_deg = 60.0;
    Vec3 camPos{0,1,3};
    Scene scene;

    if (!load_scene_from_file(scene_path, width, height, fov_deg, camPos, scene)) {
        std::cerr << "Falling back to built-in test scene.\n";
        // Minimal fallback
        Material red;   red.Kd={0.8,0.2,0.2}; red.Ks={0.6,0.6,0.6}; red.shininess=64;
        Material green; green.Kd={0.2,0.8,0.2}; green.Ks={0.4,0.4,0.4}; green.shininess=32;
        Material gray;  gray.Kd={0.6,0.6,0.6}; gray.Ks={0.2,0.2,0.2}; gray.shininess=8;
        scene.ambient = {0.1,0.1,0.12};
        auto s1 = std::make_unique<Sphere>(Vec3{-0.8,0.5,-1.5}, 0.5, red);
        auto s2 = std::make_unique<Sphere>(Vec3{ 0.9,0.75,-2.5}, 0.75, green);
        auto pl = std::make_unique<Plane>(Vec3{0,1,0}, 0.0, gray);
        scene.objects = { s1.get(), s2.get(), pl.get() };
        scene.owned.push_back(std::move(s1));
        scene.owned.push_back(std::move(s2));
        scene.owned.push_back(std::move(pl));
        scene.lights  = { {{ 2.5,5.0, 1.0},{1,1,1}}, {{-3.0,6.0,-2.0},{0.7,0.7,0.9}} };
    }

    // ---------- Render ----------
    const double fov = fov_deg * M_PI / 180.0;
    const double aspect = static_cast<double>(width) / height;
    const double scale = std::tan(fov * 0.5);

    cv::Mat img(height, width, CV_8UC3);
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            double px = ( (x + 0.5) / width  * 2.0 - 1.0) * aspect * scale;
            double py = ( 1.0 - (y + 0.5) / height * 2.0) * scale;
            Vec3 dir = Vec3(px, py, -1.0).normalized();
            Ray ray{ camPos, dir };
            Color c = scene.trace(ray); c.clamp();
            cv::Vec3b& BGR = img.at<cv::Vec3b>(y, x);
            BGR[2] = static_cast<unsigned char>(std::round(c.r * 255.0));
            BGR[1] = static_cast<unsigned char>(std::round(c.g * 255.0));
            BGR[0] = static_cast<unsigned char>(std::round(c.b * 255.0));
        }
    }

    if (!cv::imwrite("output.png", img)) {
        std::cerr << "Failed to write output.png\n";
        return 1;
    }
    std::cout << "Wrote output.png\n";
    return 0;
}
