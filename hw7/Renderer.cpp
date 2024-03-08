//
// Created by goksu on 2/25/20.
//

#include <fstream>
#include "Scene.hpp"
#include "Renderer.hpp"
#include <thread>
#include <mutex>
#include <algorithm>

using namespace std;

inline double deg2rad(const double& deg) { return deg * M_PI / 180.0; }

const double EPSILON = 0.001;


// The main render function. This where we iterate over all pixels in the image,
// generate primary rays and cast these rays into the scene. The content of the
// framebuffer is saved to a file.
void Renderer::Render(const Scene& scene)
{
    std::vector<Vector3f> framebuffer(scene.width * scene.height);

    double scale = tan(deg2rad(scene.fov * 0.5));
    double imageAspectRatio = scene.width / (double)scene.height;
    Vector3f eye_pos(278, 273, -800);
    //int m = 0;

    // change the spp value to change sample ammount
    int spp = 2800;
    std::cout << "SPP: " << spp << "\n";
    int num_thread = 14;
    int spp_thread = spp / num_thread;
    vector<thread> threads(num_thread);

    mutex mtx;

    double process = 0;
    double unit_process = 1.f / (scene.height * num_thread);

    auto shotRay = [&]() {
        for (uint32_t j = 0; j < scene.height; ++j) {
            for (uint32_t i = 0; i < scene.width; ++i) {
                // generate primary ray direction
                double x = (2 * (i + 0.5) / (double)scene.width - 1) *
                          imageAspectRatio * scale;
                double y = (1 - 2 * (j + 0.5) / (double)scene.height) * scale;

                Vector3f dir = normalize(Vector3f(-x, y, 1));
                for (int k = 0; k < spp_thread; k++){
                    framebuffer[scene.width * j + i] += scene.castRay(Ray(eye_pos, dir));  
                }
            }
            mtx.lock();
            process = process + unit_process;
            UpdateProgress(process);
            mtx.unlock();
        }
    };

    for (int i = 0; i < num_thread; ++i) {
        threads.at(i) = thread(shotRay);
    }

    for (int i = 0; i < num_thread; ++i) {
        threads.at(i).join();
    }
    UpdateProgress(1.0);

    transform(framebuffer.begin(), framebuffer.end(), framebuffer.begin(), [spp](Vector3f & v) {
        return v / spp;
    });

    // save framebuffer to file
    FILE* fp = fopen("binary.ppm", "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
    for (auto i = 0; i < scene.height * scene.width; ++i) {
        static unsigned char color[3];
        color[0] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].x), 0.6f));
        color[1] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].y), 0.6f));
        color[2] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].z), 0.6f));
        fwrite(color, 1, 3, fp);
    }
    fclose(fp);    
}
