#pragma once
#include "Scene.hpp"

struct hit_payload
{
    float tNear;     
    uint32_t index; // the index of triangle
    Vector2f uv;    // barycentic coods
    Object* hit_obj;
};

class Renderer
{
public:
    void Render(const Scene& scene);

private:
};
