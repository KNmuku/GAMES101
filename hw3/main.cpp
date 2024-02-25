#include <iostream>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include "global.hpp"
#include "rasterizer.hpp"
#include "Triangle.hpp"
#include "Shader.hpp"
#include "Texture.hpp"
#include "OBJ_Loader.h"

using namespace std;
using namespace Eigen;

Matrix4f get_view_matrix(Vector3f eye_pos)
{
    Matrix4f view = Matrix4f::Identity();

    Matrix4f translate;
    translate << 1,0,0,-eye_pos[0],
                 0,1,0,-eye_pos[1],
                 0,0,1,-eye_pos[2],
                 0,0,0,1;

    view = translate*view;

    return view;
}

Matrix4f get_model_matrix(float angle)
{
    Matrix4f rotation;
    angle = angle * MY_PI / 180.f;
    rotation << cos(angle), 0, sin(angle), 0,
                0, 1, 0, 0,
                -sin(angle), 0, cos(angle), 0,
                0, 0, 0, 1;

    Matrix4f scale;
    scale << 2.5, 0, 0, 0,
              0, 2.5, 0, 0,
              0, 0, 2.5, 0,
              0, 0, 0, 1;

    Matrix4f translate;
    translate << 1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;

    return translate * rotation * scale;
}

Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio, float zNear, float zFar)
{
    Matrix4f projection = Matrix4f::Identity();
    float radian_fov = eye_fov / 180.f * MY_PI;
    float top   = abs(zNear) * tan(radian_fov / 2);
    float bot   = -top;
    float right = top * aspect_ratio;
    float left  = -right; 
    float n     = -zNear;
    float f     = -zFar;

    Matrix4f persp_to_ortho;
    Matrix4f translate;
    Matrix4f scale;
    Matrix4f ortho;
    Matrix4f reflect;
    persp_to_ortho << n, 0, 0, 0,
                      0, n, 0, 0,
                      0, 0, (n + f), -n * f,
                      0, 0, 1, 0;
    translate   <<    1, 0, 0, -(left + right) / 2.f,
                      0, 1, 0, -(bot + top) / 2.f,
                      0, 0, 1, -(n + f) / 2.f,
                      0, 0, 0, 1;
    scale     <<      2.f/(right-left), 0, 0, 0,
                      0, 2.f/(top-bot), 0, 0,
                      0, 0, 2.f/(n - f), 0,
                      0, 0, 0, 1;

    ortho = scale * translate;
    projection = ortho * persp_to_ortho * projection;
    return projection;
}

Vector3f vertex_shader(const vertex_shader_payload& payload)
{
    return payload.position;
}

Vector3f normal_fragment_shader(const fragment_shader_payload& payload)
{
    Vector3f return_color = (payload.normal.head<3>().normalized() + Vector3f(1.0f, 1.0f, 1.0f)) / 2.f;
    Vector3f result;
    result << return_color.x() * 255, return_color.y() * 255, return_color.z() * 255;
    return result;
}

// In reality, there is Phong reflectance model, because we are using
// reflected light directly rather than bisector!
static Vector3f reflect(const Vector3f& vec, const Vector3f& axis)
{
    auto costheta = vec.dot(axis);
    return 2 * costheta * axis - vec;
}

struct light
{
    Vector3f position;
    Vector3f intensity;
};


Vector3f phong_fragment_shader(const fragment_shader_payload& payload)
{
    Vector3f ka = Vector3f(0.005, 0.005, 0.005);
    // triangle color is normalized (0, 1)
    Vector3f kd = payload.color;
    Vector3f ks = Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    vector<light> lights = {l1, l2};
    Vector3f amb_light_intensity{10, 10, 10};
    //Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Vector3f color = payload.color;
    Vector3f point = payload.view_pos;
    Vector3f normal = payload.normal;

    Vector3f result_color = {0, 0, 0};

    for (auto& light : lights)
    {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
        // components are. Then, accumulate that result on the *result_color* object.
      normal = normal.normalized();
      Vector3f view = (-point).normalized();
      Vector3f input = (light.position - point).normalized();
      Vector3f reflected = reflect(input, normal).normalized();
      //Vector3f bisector = (view + input).normalized();
      float distance = (light.position - point).norm(); 
      Vector3f falloff = light.intensity / pow(distance, 2);

      Vector3f La = ka.cwiseProduct(amb_light_intensity);
      Vector3f Ld = kd.cwiseProduct(falloff) * max(0.f, normal.dot(input));
      Vector3f Ls = ks.cwiseProduct(falloff) * pow(max(0.f, view.dot(reflected)), p);
      result_color += La + Ld + Ls;
        
    }

    return result_color * 255.f;
}

Vector3f texture_fragment_shader(const fragment_shader_payload& payload)
{
    Vector3f return_color = {0, 0, 0};
    if (payload.texture)
    {
        // TODO: Get the texture value at the texture coordinates of the current fragment
      return_color = payload.texture->getColor(payload.tex_coords.x(),
                                                payload.tex_coords.y());
    }
    Vector3f texture_color;
    texture_color << return_color.x(), return_color.y(), return_color.z();

    Vector3f ka = Vector3f(0.005, 0.005, 0.005);
    // texture_color is not normalized (0, 255)
    Vector3f kd = texture_color / 255.f;
    Vector3f ks = Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    vector<light> lights = {l1, l2};
    Vector3f amb_light_intensity{10, 10, 10};
    
    //Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Vector3f color = texture_color;
    
    // current shading point position in view space;
    Vector3f point = payload.view_pos;
    Vector3f normal = payload.normal;

    Vector3f result_color = {0, 0, 0};

    for (auto& light : lights)
    {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
        // components are. Then, accumulate that result on the *result_color* object.
      normal = normal.normalized();
      Vector3f view = (-point).normalized();
      Vector3f input = (light.position - point).normalized();
      Vector3f reflected = reflect(input, normal).normalized();
      //Vector3f bisector = (input + view).normalized();
      float distance = (light.position - point).norm(); 
      Vector3f falloff = light.intensity / pow(distance, 2);

      Vector3f La = ka.cwiseProduct(amb_light_intensity);
      Vector3f Ld = kd.cwiseProduct(falloff) * max(0.f, normal.dot(input));
      Vector3f Ls = ks.cwiseProduct(falloff) * pow(max(0.f, view.dot(reflected)), p);
      result_color += La + Ld + Ls;
    }

    return result_color * 255.f;
}


Vector3f displacement_fragment_shader(const fragment_shader_payload& payload)
{
    
    Vector3f ka = Vector3f(0.005, 0.005, 0.005);
    Vector3f kd = payload.color;
    Vector3f ks = Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    vector<light> lights = {l1, l2};
    Vector3f amb_light_intensity{10, 10, 10};
    Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Vector3f color = payload.color; 
    Vector3f point = payload.view_pos;
    Vector3f normal = payload.normal;

    float kh = 0.2, kn = 0.1;
    
    // TODO: Implement displacement mapping here
    // Let n = normal = (x, y, z)
    // Vector t = (x*y/sqrt(x*x+z*z),sqrt(x*x+z*z),z*y/sqrt(x*x+z*z))
    // Vector b = n cross product t
    // Matrix TBN = [t b n]
    // dU = kh * kn * (h(u+1/w,v)-h(u,v))
    // dV = kh * kn * (h(u,v+1/h)-h(u,v))
    // Vector ln = (-dU, -dV, 1)
    // Position p = p + kn * n * h(u,v)
    // Normal n = normalize(TBN * ln)
    float x = normal.x();
    float y = normal.y();
    float z = normal.z();
    Vector3f t{x*y / sqrt(x*x+z*z), sqrt(x*x+z*z), z*y / sqrt(x*x+z*z)};
    t.normalize();
    Vector3f b = normal.cross(t);
    Matrix3f TBN;
    TBN << t, b, normal;
    /*
    Vector3f E1 = payload.vertices[1] - payload.vertices[0];
    Vector3f E2 = payload.vertices[2] - payload.vertices[1];
    float delU1 = (payload.vertices_tex_coords[1] - payload.vertices_tex_coords[0]).x();
    float delU2 = (payload.vertices_tex_coords[2] - payload.vertices_tex_coords[1]).x(); 
    float delV1 = (payload.vertices_tex_coords[1] - payload.vertices_tex_coords[0]).y();
    float delV2 = (payload.vertices_tex_coords[2] - payload.vertices_tex_coords[1]).y(); 
    float fractional = 1.f / (delV1 * delU2 - delV2 * delU1);

    Vector3f t = fractional * (delV1 * E2 - delV2 * E1);
    Vector3f b = fractional * (delU2 * E1 - delU1 * E2);

    t = (t - (t.dot(normal) * normal)).normalized();
    b = (b - (b.dot(normal) * normal - (b.dot(t) * t))).normalized();

    Matrix3f TBN;
    TBN << t, b, normal;*/

    float w = payload.texture->width;
    float h = payload.texture->height;
    float u = payload.tex_coords.x();
    float v = payload.tex_coords.y();

    float dh_du = kh * kn * (payload.texture->getColor(u + 1.f/w, v).norm() - payload.texture->getColor(u, v).norm());
    float dh_dv = kh * kn * (payload.texture->getColor(u, v + 1.f/h).norm() - payload.texture->getColor(u, v).norm());
    // directly using color as normal
    //Vector3f n_local = (payload.texture->getColor(u, v) / 255.f - Vector3f{.5f, .5f, .5f}) * 2;
    Vector3f n_local{-dh_du, -dh_dv, 1};
    Vector3f n_view = (TBN * n_local).normalized();

    point += kn * normal * payload.texture->getColor(u, v).norm();
    normal = n_view;
  
    Vector3f result_color = {0, 0, 0};

    for (auto& light : lights)
    {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
        // components are. Then, accumulate that result on the *result_color* object.
      normal = normal.normalized();
      Vector3f view = (-point).normalized();
      Vector3f input = (light.position - point).normalized();
      Vector3f reflected = reflect(input, normal).normalized();
      //Vector3f bisector = (input + view).normalized();
      float distance = (light.position - point).norm(); 
      Vector3f falloff = light.intensity / pow(distance, 2);

      Vector3f La = ka.cwiseProduct(amb_light_intensity);
      Vector3f Ld = kd.cwiseProduct(falloff) * max(0.f, normal.dot(input));
      Vector3f Ls = ks.cwiseProduct(falloff) * pow(max(0.f, view.dot(reflected)), p);
      result_color += La + Ld + Ls;
    }

    return result_color * 255.f;
}


Vector3f bump_fragment_shader(const fragment_shader_payload& payload)
{
    
    Vector3f ka = Vector3f(0.005, 0.005, 0.005);
    Vector3f kd = payload.color;
    Vector3f ks = Vector3f(0.7937, 0.7937, 0.7937);

    auto l1 = light{{20, 20, 20}, {500, 500, 500}};
    auto l2 = light{{-20, 20, 0}, {500, 500, 500}};

    vector<light> lights = {l1, l2};
    Vector3f amb_light_intensity{10, 10, 10};
    Vector3f eye_pos{0, 0, 10};

    float p = 150;

    Vector3f color = payload.color; 
    Vector3f point = payload.view_pos;
    Vector3f normal = payload.normal;


    float kh = 0.2, kn = 0.1;

    // TODO: Implement bump mapping here
    // Let n = normal = (x, y, z)
    // Vector t = (x*y/sqrt(x*x+z*z),sqrt(x*x+z*z),z*y/sqrt(x*x+z*z))
    // Vector b = n cross product t
    // Matrix TBN = [t b n]
    // dU = kh * kn * (h(u+1/w,v)-h(u,v))
    // dV = kh * kn * (h(u,v+1/h)-h(u,v))
    // Vector ln = (-dU, -dV, 1)
    // Normal n = normalize(TBN * ln)
    
    float x = normal.x();
    float y = normal.y();
    float z = normal.z();
    Vector3f t{x*y / sqrt(x*x+z*z), sqrt(x*x+z*z), z*y / sqrt(x*x+z*z)};
    t.normalize();
    Vector3f b = normal.cross(t);
    Matrix3f TBN;
    TBN << t, b, normal;
    /*
    Vector3f E1 = payload.vertices[1] - payload.vertices[0];
    Vector3f E2 = payload.vertices[2] - payload.vertices[1];
    float delU1 = (payload.vertices_tex_coords[1] - payload.vertices_tex_coords[0]).x();
    float delU2 = (payload.vertices_tex_coords[2] - payload.vertices_tex_coords[1]).x(); 
    float delV1 = (payload.vertices_tex_coords[1] - payload.vertices_tex_coords[0]).y();
    float delV2 = (payload.vertices_tex_coords[2] - payload.vertices_tex_coords[1]).y(); 
    float fractional = 1.f / (delV1 * delU2 - delV2 * delU1);

    Vector3f t = fractional * (delV1 * E2 - delV2 * E1);
    Vector3f b = fractional * (delU2 * E1 - delU1 * E2);

    t = (t - (t.dot(normal) * normal)).normalized();
    b = (b - (b.dot(normal) * normal - (b.dot(t) * t))).normalized();

    Matrix3f TBN;
    TBN << t, b, normal;*/
    
    float w = payload.texture->width;
    float h = payload.texture->height;
    float u = payload.tex_coords.x();
    float v = payload.tex_coords.y();

    float dh_du = kh * kn * (payload.texture->getColor(u + 1.f/w, v).norm() - payload.texture->getColor(u, v).norm());
    float dh_dv = kh * kn * (payload.texture->getColor(u, v + 1.f/h).norm() - payload.texture->getColor(u, v).norm());
    // directly using color as normal
    //Vector3f n_local = (payload.texture->getColor(u, v) / 255.f - Vector3f{.5f, .5f, .5f}) * 2;
    Vector3f n_local{-dh_du, -dh_dv, 1};
    Vector3f n_view = (TBN * n_local).normalized();
  
    Vector3f result_color = {0, 0, 0};
    result_color = n_view;

    return result_color * 255.f;
}

int main(int argc, const char** argv)
{
    vector<Triangle*> TriangleList;

    float angle = 140.0;
    bool command_line = false;

    string filename = "output.png";
    objl::Loader Loader;
    string obj_path = "../models/spot/";

    // Load .obj File
    bool loadout = Loader.LoadFile("../models/spot/spot_triangulated_good.obj");
    for(auto mesh:Loader.LoadedMeshes)
    {
        for(int i=0;i<mesh.Vertices.size();i+=3)
        {
            Triangle* t = new Triangle();
            for(int j=0;j<3;j++)
            {
                t->setVertex(j,Vector4f(mesh.Vertices[i+j].Position.X,mesh.Vertices[i+j].Position.Y,mesh.Vertices[i+j].Position.Z,1.0));
                t->setNormal(j,Vector3f(mesh.Vertices[i+j].Normal.X,mesh.Vertices[i+j].Normal.Y,mesh.Vertices[i+j].Normal.Z));
                t->setTexCoord(j,Vector2f(mesh.Vertices[i+j].TextureCoordinate.X, mesh.Vertices[i+j].TextureCoordinate.Y));
            }
            TriangleList.push_back(t);
        }
    }

    rst::rasterizer r(700, 700);

    // default texture
    auto texture_path = "hmap.jpg";
    r.set_texture(Texture(obj_path + texture_path));

    function<Vector3f(fragment_shader_payload)> active_shader = phong_fragment_shader;

    if (argc >= 2)
    {
        command_line = true;
        filename = string(argv[1]);

        if (argc == 3 && string(argv[2]) == "texture")
        {
            cout << "Rasterizing using the texture shader\n";
            active_shader = texture_fragment_shader;
            texture_path = "spot_texture.png";
            r.set_texture(Texture(obj_path + texture_path));
        }
        else if (argc == 3 && string(argv[2]) == "normal")
        {
            cout << "Rasterizing using the normal shader\n";
            active_shader = normal_fragment_shader;
        }
        else if (argc == 3 && string(argv[2]) == "phong")
        {
            cout << "Rasterizing using the phong shader\n";
            active_shader = phong_fragment_shader;
        }
        else if (argc == 3 && string(argv[2]) == "bump")
        {
            cout << "Rasterizing using the bump shader\n";
            active_shader = bump_fragment_shader;
        }
        else if (argc == 3 && string(argv[2]) == "displacement")
        {
            cout << "Rasterizing using the displacement shader\n";
            active_shader = displacement_fragment_shader;
        }
    }

    Vector3f eye_pos = {0,0,10};

    r.set_vertex_shader(vertex_shader);
    r.set_fragment_shader(active_shader);

    int key = 0;
    int frame_count = 0;

    if (command_line)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);
        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45.0, 1, 0.1, 50));

        r.draw(TriangleList);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        cv::imwrite(filename, image);

        return 0;
    }

    while(key != 27)
    {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45.0, 1, 0.1, 50));

        //r.draw(pos_id, ind_id, col_id, rst::Primitive::Triangle);
        r.draw(TriangleList);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

        cv::imshow("image", image);
        cv::imwrite(filename, image);
        key = cv::waitKey(10);

        if (key == 'a' )
        {
            angle -= 0.1;
        }
        else if (key == 'd')
        {
            angle += 0.1;
        }

    }
    return 0;
}
