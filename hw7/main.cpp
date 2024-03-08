#include "Renderer.hpp"
#include "Scene.hpp"
#include "Triangle.hpp"
#include "Sphere.hpp"
#include "Vector.hpp"
#include "global.hpp"
#include <chrono>

// In the main function of the program, we create the scene (create objects and
// lights) as well as set the options for the render (image width and height,
// maximum recursion depth, field-of-view, etc.). We then call the render
// function().
int main(int argc, char** argv)
{

    // Change the definition here to change resolution
    Scene scene(1080, 720);

    Material* red = new Material(DIFFUSE, Vector3f(0.0f));
    red->Kd = Vector3f(0.63f, 0.065f, 0.05f);
    Material* green = new Material(DIFFUSE, Vector3f(0.0f));
    green->Kd = Vector3f(0.14f, 0.45f, 0.091f);
    Material* white = new Material(DIFFUSE, Vector3f(0.0f));
    white->Kd = Vector3f(0.725f, 0.71f, 0.68f);
    Material* light = new Material(DIFFUSE, (8.0f * Vector3f(0.747f+0.058f, 0.747f+0.258f, 0.747f) + 15.6f * Vector3f(0.740f+0.287f,0.740f+0.160f,0.740f) + 18.4f *Vector3f(0.737f+0.642f,0.737f+0.159f,0.737f)));
    light->Kd = Vector3f(0.65f);

    Material * microfacet1 = new Material(MICROFACET, Vector3f(0.0));
    microfacet1->Ks = Vector3f(0.45);
    microfacet1->Kd = Vector3f(0.3);
    microfacet1->ior = 12.85;
    microfacet1->roughness = 1;

    Material * microfacet2 = new Material(MICROFACET, Vector3f(0.0));
    microfacet2->Ks = Vector3f(0.45);
    microfacet2->Kd = Vector3f(0.3);
    microfacet2->ior = 12.85;
    microfacet2->roughness = 0.25;

    Material * microfacet3 = new Material(MICROFACET, Vector3f(0.0));
    microfacet3->Ks = Vector3f(0.45);
    microfacet3->Kd = Vector3f(0.6);
    microfacet3->ior = 12.85;
    microfacet3->roughness = 0.5;
    
    Vector3f center1(200, 75, 150);
    Vector3f center2(400, 75, 200);
    Vector3f center3(250, 100, 380);
    Sphere sphere1 = Sphere(center1, 75, microfacet1);
    Sphere sphere2 = Sphere(center2, 75, microfacet2);
    Sphere sphere3 = Sphere(center3, 100, microfacet3);


    MeshTriangle floor("../models/cornellbox/floor.obj", white);
    MeshTriangle shortbox("../models/cornellbox/shortbox.obj", white);
    MeshTriangle tallbox("../models/cornellbox/tallbox.obj", white);
    MeshTriangle left("../models/cornellbox/left.obj", red);
    MeshTriangle right("../models/cornellbox/right.obj", green);
    MeshTriangle light_("../models/cornellbox/light.obj", light);

    scene.Add(&floor);
    //scene.Add(&shortbox);
    //scene.Add(&tallbox);
    scene.Add(&left);
    scene.Add(&right);
    scene.Add(&light_);
    scene.Add(&sphere1);
    scene.Add(&sphere2);
    scene.Add(&sphere3);

    scene.buildBVH();

    Renderer r;

    auto start = std::chrono::system_clock::now();
    r.Render(scene);
    auto stop = std::chrono::system_clock::now();

    std::cout << "Render complete: \n";
    std::cout << "Time taken: " << std::chrono::duration_cast<std::chrono::hours>(stop - start).count() << " hours\n";
    std::cout << "          : " << std::chrono::duration_cast<std::chrono::minutes>(stop - start).count() << " minutes\n";
    std::cout << "          : " << std::chrono::duration_cast<std::chrono::seconds>(stop - start).count() << " seconds\n";

    return 0;
}
