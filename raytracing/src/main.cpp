#include <opencv2/opencv.hpp>
#include <cmath>
#include <vector>
#include <limits>
#include <algorithm>
#include <iostream>

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
    Vec3 normalized() const {
        double n = norm();
        return (n > 0) ? (*this)/n : *this;
    }
};

struct Color {
    double r{}, g{}, b{}; // 0..1
    Color() = default;
    Color(double R, double G, double B) : r(R), g(G), b(B) {}

    Color operator+(const Color& o) const { return {r+o.r, g+o.g, b+o.b}; }
    Color operator*(double s) const { return {r*s, g*s, b*s}; }
    Color& operator+=(const Color& o){ r+=o.r; g+=o.g; b+=o.b; return *this; }

    static Color hadamard(const Color& a, const Color& b){
        return {a.r*b.r, a.g*b.g, a.b*b.b};
    }
    void clamp(){
        r = std::min(1.0, std::max(0.0, r));
        g = std::min(1.0, std::max(0.0, g));
        b = std::min(1.0, std::max(0.0, b));
    }
};

struct Ray { Vec3 origin; Vec3 dir; }; // dir should be normalized

struct Material {
    Color Ka{0.05, 0.05, 0.05}; // ambient
    Color Kd{0.7, 0.7, 0.7};    // diffuse (Lambert)
    Color Ks{0.3, 0.3, 0.3};    // specular
    double shininess{32.0};
    // For minimal example we skip reflection/refraction
};

struct Light {
    Vec3 pos;
    Color intensity{1.0, 1.0, 1.0}; // per-channel intensity (0..1)
};

struct Hit {
    double t{std::numeric_limits<double>::infinity()};
    Vec3 p;
    Vec3 n;
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
        double a = Vec3::dot(ray.dir, ray.dir); // should be 1 if dir normalized
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
    // Plane defined by normal n (normalized) and offset d: dot(n, p) + d = 0
    Vec3 n; double d;
    Plane(const Vec3& N, double D, const Material& M){ n=N.normalized(); d=D; mat=M; }

    Hit intersect(const Ray& ray) const override {
        double denom = Vec3::dot(n, ray.dir);
        Hit h;
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

struct Scene {
    std::vector<const Object*> objects;
    std::vector<Light> lights;
    Color ambient{0.1, 0.1, 0.1};

    bool occluded(const Vec3& p, const Vec3& toLight, double maxDist) const {
        Ray shadowRay{ p + toLight * EPS, toLight }; // small offset to avoid acne
        for (const auto* obj : objects) {
            Hit h = obj->intersect(shadowRay);
            if (h.hit && h.t < maxDist - EPS) return true;
        }
        return false;
    }

    Color shade(const Ray& ray, const Hit& h) const {
        // Phong: ambient + sum_lights( diffuse + specular )
        Color out = Color::hadamard(h.mat.Ka, ambient); // ambient term
        Vec3 V = (ray.origin - h.p).normalized();

        for (const auto& L : lights) {
            Vec3 Ldir = (L.pos - h.p);
            double dist = Ldir.norm();
            Ldir = Ldir / dist;

            if (occluded(h.p, Ldir, dist)) continue; // in shadow

            double ndotl = std::max(0.0, Vec3::dot(h.n, Ldir));
            Color diffuse = Color::hadamard(h.mat.Kd, L.intensity) * ndotl;

            // Blinn-Phong: use half-vector for numerical stability
            Vec3 H = (Ldir + V).normalized();
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
        if (!best.hit) return {0.0, 0.0, 0.0}; // background
        return shade(ray, best);
    }
};

int main() {
    // ---------- Image / camera ----------
    const int width = 800;
    const int height = 600;
    const double fov_deg = 60.0;
    const double fov = fov_deg * M_PI / 180.0;
    const double aspect = static_cast<double>(width) / height;
    const Vec3 camPos(0, 1, 3); // slightly above, looking toward -Z

    // ---------- Scene ----------
    Material red;   red.Kd = {0.8, 0.2, 0.2}; red.Ks = {0.6, 0.6, 0.6}; red.shininess = 64;
    Material green; green.Kd = {0.2, 0.8, 0.2}; green.Ks = {0.4, 0.4, 0.4}; green.shininess = 32;
    Material gray;  gray.Kd = {0.6, 0.6, 0.6}; gray.Ks = {0.2, 0.2, 0.2}; gray.shininess = 8;

    Sphere s1({-0.8, 0.5, -1.5}, 0.5, red);
    Sphere s2({ 0.9, 0.75, -2.5}, 0.75, green);
    Plane  ground({0,1,0}, 0.0, gray); // y=0 plane (n=(0,1,0), d=0)

    Light  l1{{ 2.5, 5.0,  1.0}, {1.0, 1.0, 1.0}};
    Light  l2{{-3.0, 6.0, -2.0}, {0.7, 0.7, 0.9}};

    Scene scene;
    scene.objects = { &s1, &s2, &ground };
    scene.lights  = { l1, l2 };
    scene.ambient = {0.1, 0.1, 0.12};

    // ---------- Render ----------
    cv::Mat img(height, width, CV_8UC3);

    const double scale = std::tan(fov * 0.5);
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            // NDC -> camera space
            double px = ( (x + 0.5) / width  * 2.0 - 1.0) * aspect * scale;
            double py = ( 1.0 - (y + 0.5) / height * 2.0) * scale;
            Vec3 dir = Vec3(px, py, -1.0).normalized();

            Ray ray{ camPos, dir };
            Color c = scene.trace(ray);
            c.clamp();

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
