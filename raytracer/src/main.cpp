#include <iostream>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>

#include "core.h"
#include "camera.h"
#include "scene.h"
#include "tracer.h"
#include "json_loader.h"

/**
 * @brief Convert a linear Color framebuffer to an 8-bit BGR OpenCV image.
 * @param fb Row-major framebuffer of size W*H with linear [0,1] channels.
 * @param W Image width in pixels.
 * @param H Image height in pixels.
 * @return cv::Mat with type CV_8UC3 in BGR channel order.
 */
static cv::Mat framebuffer_to_mat_bgr8(const std::vector<Color>& fb, int W, int H) {
    cv::Mat img(H, W, CV_8UC3);
    for (int y = 0; y < H; ++y) {
        const int row = y * W;
        auto* p = img.ptr<cv::Vec3b>(y);
        for (int x = 0; x < W; ++x) {
            const Color& c = fb[row + x];
            // clamp to [0,1], convert to 8-bit and put in BGR order for OpenCV
            const int R = toByte(c.r);
            const int G = toByte(c.g);
            const int B = toByte(c.b);
            p[x] = cv::Vec3b{ (unsigned char)B, (unsigned char)G, (unsigned char)R };
        }
    }
    return img;
}

/**
 * @brief Program entry: load scene JSON, render with Tracer, and write PNG.
 * @param argc Argument count.
 * @param argv Arguments: <scene.json> <output.png> [--paper].
 * @return Exit code: 0 on success; nonzero on usage, load, or I/O errors.
 */
int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <scene.json> <output.png> [--paper]\n";
        std::cerr << "  --paper: Enable paper rendering mode with crosshatching\n";
        return 1;
    }
    const std::string json_path = argv[1];
    const std::string out_path  = argv[2];
    
    // Check for paper mode flag
    bool paper_mode = false;
    if (argc > 3 && std::string(argv[3]) == "--paper") {
        paper_mode = true;
    }

    Scene scene;
    Camera cam;

    try {
        if (!jsonio::load_scene_from_json(json_path, scene, cam)) {
            std::cerr << "Failed to load scene from " << json_path << "\n";
            return 2;
        }
    } catch (const std::exception& e) {
        std::cerr << "[error] " << e.what() << "\n";
        return 3;
    }

    // Image size from screen spec (Lx, Ly, dpi) as in your camera.h
    const int W = cam.screen.nx();
    const int H = cam.screen.ny();

    Tracer tracer;
    tracer.scene  = &scene;
    tracer.camera = &cam;
    tracer.width  = W;
    tracer.height = H;
    tracer.mode   = paper_mode ? RenderMode::Paper : RenderMode::Standard;

    std::vector<Color> framebuffer;
    tracer.render(framebuffer);

    cv::Mat img = framebuffer_to_mat_bgr8(framebuffer, W, H);

    if (!cv::imwrite(out_path, img)) {
        std::cerr << "Failed to write PNG: " << out_path << "\n";
        return 4;
    }
    
    std::string mode_str = paper_mode ? " (paper mode)" : "";
    std::cout << "Wrote " << out_path << " (" << W << "x" << H << ")" << mode_str << "\n";
    return 0;
}
