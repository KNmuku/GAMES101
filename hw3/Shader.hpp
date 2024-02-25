//
// Created by LEI XU on 4/27/19.
//

// My notes: these two payload structs are used to store arguments, 
// which are passed to fragment_shader function.

#ifndef RASTERIZER_SHADER_H
#define RASTERIZER_SHADER_H
#include <eigen3/Eigen/Eigen>
#include "Texture.hpp"


struct fragment_shader_payload
{
    fragment_shader_payload()
    {
        texture = nullptr;
    }

    fragment_shader_payload(const Eigen::Vector3f& view_pos, const Eigen::Vector3f& col, const Eigen::Vector3f& nor,const Eigen::Vector2f& tc, Texture* tex) :
         view_pos(view_pos), color(col), normal(nor), tex_coords(tc), texture(tex) {}
   
    std::array<Eigen::Vector2f, 3> vertices_tex_coords;
    std::array<Eigen::Vector3f, 3> vertices;
    Eigen::Vector3f view_pos;
    Eigen::Vector3f color;
    Eigen::Vector3f normal;
    Eigen::Vector2f tex_coords;
    Texture* texture;
};

struct vertex_shader_payload
{
    Eigen::Vector3f position;
};

#endif //RASTERIZER_SHADER_H
