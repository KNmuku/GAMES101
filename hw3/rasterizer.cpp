//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>

using namespace Eigen;
using namespace std;

rst::pos_buf_id rst::rasterizer::load_positions(const vector<Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const vector<Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const vector<Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_normals(const vector<Vector3f>& normals)
{
    auto id = get_next_id();
    nor_buf.emplace(id, normals);

    normal_id = id;

    return {id};
}


// Bresenham's line drawing algorithm
void rst::rasterizer::draw_line(Vector3f begin, Vector3f end)
{
    auto x1 = begin.x();
    auto y1 = begin.y();
    auto x2 = end.x();
    auto y2 = end.y();

    Vector3f line_color = {255, 255, 255};

    int x,y,dx,dy,dx1,dy1,px,py,xe,ye,i;

    dx=x2-x1;
    dy=y2-y1;
    dx1=fabs(dx);
    dy1=fabs(dy);
    px=2*dy1-dx1;
    py=2*dx1-dy1;

    if(dy1<=dx1)
    {
        if(dx>=0)
        {
            x=x1;
            y=y1;
            xe=x2;
        }
        else
        {
            x=x2;
            y=y2;
            xe=x1;
        }
        Vector2i point = Vector2i(x, y);
        set_pixel(point,line_color);
        for(i=0;x<xe;i++)
        {
            x=x+1;
            if(px<0)
            {
                px=px+2*dy1;
            }
            else
            {
                if((dx<0 && dy<0) || (dx>0 && dy>0))
                {
                    y=y+1;
                }
                else
                {
                    y=y-1;
                }
                px=px+2*(dy1-dx1);
            }
//            delay(0);
            Vector2i point = Vector2i(x, y);
            set_pixel(point,line_color);
        }
    }
    else
    {
        if(dy>=0)
        {
            x=x1;
            y=y1;
            ye=y2;
        }
        else
        {
            x=x2;
            y=y2;
            ye=y1;
        }
        Vector2i point = Vector2i(x, y);
        set_pixel(point,line_color);
        for(i=0;y<ye;i++)
        {
            y=y+1;
            if(py<=0)
            {
                py=py+2*dx1;
            }
            else
            {
                if((dx<0 && dy<0) || (dx>0 && dy>0))
                {
                    x=x+1;
                }
                else
                {
                    x=x-1;
                }
                py=py+2*(dx1-dy1);
            }
//            delay(0);
            Vector2i point = Vector2i(x, y);
            set_pixel(point,line_color);
        }
    }
}

auto to_vec4(const Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

static bool insideTriangle(float x, float y, const array<Vector4f, 3> & _v){
  const int NUM = 3;
  Vector3f p;
  vector<Vector3f> vs(NUM);

  p << x, y, 0.f;

  for (int i = 0; i < NUM; ++i) {
    vs[i] = Vector3f{_v[i].x(), _v[i].y(), 0.f}; 
  }

  Vector3f ver_to_ver[] = {vs[1]-vs[0], vs[2]-vs[1], vs[0]-vs[2]};
  Vector3f ver_to_p[]   = {p-vs[0], p-vs[1], p-vs[2]};
   
  vector<float> z_values(NUM);
  for (int i = 0; i < NUM; ++i) {
    z_values[i] = ver_to_ver[i].cross(ver_to_p[i]).z();
  }

  return ((z_values[0] >= 0) && (z_values[1] >= 0) && (z_values[2] >= 0)) ||
         ((z_values[0] <= 0) && (z_values[1] <= 0) && (z_values[2] <= 0)); 
}

static tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector4f* v){
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(vector<Triangle *> &TriangleList) {

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Matrix4f mvp = projection * view * model;
    for (const auto& t:TriangleList)
    {
        Triangle newtri = *t;

        array<Vector4f, 3> mm {
                (view * model * t->v[0]),
                (view * model * t->v[1]),
                (view * model * t->v[2])
        };

        array<Vector3f, 3> viewspace_pos;

        transform(mm.begin(), mm.end(), viewspace_pos.begin(), [](auto& v) {
            return v.template head<3>();
        });

        Vector4f v[] = {
                mvp * t->v[0],
                mvp * t->v[1],
                mvp * t->v[2]
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec.x()/=vec.w();
            vec.y()/=vec.w();
            vec.z()/=vec.w();
        }

        // transform normal to view space, TO BE DEDUCTED!
        Matrix4f inv_trans = (view * model).inverse().transpose();
        Vector4f n[] = {
                (inv_trans * to_vec4(t->normal[0], 0.0f)).normalized(),
                (inv_trans * to_vec4(t->normal[1], 0.0f)).normalized(),
                (inv_trans * to_vec4(t->normal[2], 0.0f)).normalized(),
        };

        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            //screen space coordinates
            newtri.setVertex(i, v[i]);
        }

        for (int i = 0; i < 3; ++i)
        {
            //view space normal
            newtri.setNormal(i, n[i].head<3>());
        }

        // default color: brown (R,G,B) = (148,121,92)
        newtri.setColor(0, 148,121.0,92.0);
        newtri.setColor(1, 148,121.0,92.0);
        newtri.setColor(2, 148,121.0,92.0);

        // Also pass view space vertice position
        rasterize_triangle(newtri, viewspace_pos);
    }
}

static Vector3f interpolate(const Vector4f & ds, float alpha, float beta, float gamma, const Vector3f& vert0, const Vector3f& vert1, const Vector3f& vert2, float weight)
{
  float z0 = ds[0];
  float z1 = ds[1];
  float z2 = ds[2];
  float zt = ds[3];
  return zt * (alpha * vert0 / z0 + beta * vert1 / z1 + gamma * vert2 / z2);
}

static Vector2f interpolate(const Vector4f & ds, float alpha, float beta, float gamma, const Vector2f& vert0, const Vector2f& vert1, const Vector2f& vert2, float weight)
{
  float z0 = ds[0];
  float z1 = ds[1];
  float z2 = ds[2];
  float zt = ds[3];
  return zt * (alpha * vert0 / z0 + beta * vert1 / z1 + gamma * vert2 / z2);
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t, const array<Vector3f, 3>& view_pos) {
    // TODO: From your HW2, get the triangle rasterization code.
    // TODO: Inside your rasterization loop:
    //    * v[i].w() is the vertex view space depth value z.
    //    * Z is interpolated view space depth for the current pixel
    //    * zp is depth between zNear and zFar, used for z-buffer

    // float Z = 1.0 / (alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
    // float zp = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
    // zp *= Z;

    // TODO: Interpolate the attributes:
    // auto interpolated_color
    // auto interpolated_normal
    // auto interpolated_texcoords
    // auto interpolated_shadingcoords

    // Use: fragment_shader_payload payload( interpolated_color, interpolated_normal.normalized(), interpolated_texcoords, texture ? &*texture : nullptr);
    // Use: payload.view_pos = interpolated_shadingcoords;
    // Use: Instead of passing the triangle's color directly to the frame buffer, pass the color to the shaders first to get the final color;
    // Use: auto pixel_color = fragment_shader(payload);
  array<Vector4f, 3> v = t.toVector4();
  // aligned-axis pixel bounding box
  int left   = (int) floor(min(min(v[0].x(), v[1].x()), v[2].x()));
  int bottom = (int) floor(min(min(v[0].y(), v[1].y()), v[2].y()));
  int right  = (int)  ceil(max(max(v[0].x(), v[1].x()), v[2].x()));
  int top    = (int)  ceil(max(max(v[0].y(), v[1].y()), v[2].y()));

  for (int x = left; x < right; ++x) {
    for (int y = bottom; y < top; ++y) {
      if (insideTriangle(x + .5f, y + .5f, v)) {
        auto[alpha, beta, gamma] = computeBarycentric2D(x + .5f, y + .5f, t.v);
        float d0 = -v[0].w();
        float d1 = -v[1].w();
        float d2 = -v[2].w();
        float z_interp = 1.f / (alpha / d0 + beta / d1 + gamma / d2);
        int index = get_index(x, y);
        if (z_interp < depth_buf[get_index(x, y)]) {
          depth_buf[index] = z_interp;
          
          Vector4f ds{d0, d1, d2, z_interp};
          auto color_interp = interpolate(ds, alpha, beta, gamma, t.color[0], t.color[1], t.color[2], 1);
          // ---------------------------------|
          // shading frequency: phong shading |
          /* ---------------------------------|
          auto normal_interp = interpolate(ds, alpha, beta, gamma, t.normal[0], t.normal[1], t.normal[2], 1);
          normal_interp.normalize();
          auto tex_coord_interp = interpolate(ds, alpha, beta, gamma, t.tex_coords[0], t.tex_coords[1], t.tex_coords[2], 1);
          auto view_pos_interp = interpolate(ds, alpha, beta, gamma, view_pos[0], view_pos[1], view_pos[2], 1);
          fragment_shader_payload payload(view_pos_interp, color_interp, normal_interp, tex_coord_interp, texture ? &*texture : nullptr);
          auto pixel_color = fragment_shader(payload);*/
          // -----------------------------------|
          // shading frequency: gouraud shading |
          // -----------------------------------|
          array<Vector3f, 3> vts_colors;
          for (int i = 0; i < 3; ++i) {
            fragment_shader_payload payload(view_pos[i], t.color[i], t.normal[i], t.tex_coords[i], texture ? &*texture : nullptr);
            vts_colors.at(i) = fragment_shader(payload);
          }
          auto pixel_color = interpolate(ds, alpha, beta, gamma, vts_colors.at(0), vts_colors.at(1), vts_colors.at(2), 1);
          //
          // --------------------------------|
          // shading frequency: flat shading |
          /* --------------------------------|
          Vector3f BC = view_pos[2] - view_pos[1];
          Vector3f BA = view_pos[0] - view_pos[1];
          Vector3f normal = (BC).cross(BA).normalized();
          fragment_shader_payload payload(view_pos[0], t.color[0], normal, t.tex_coords[0], texture ? &*texture : nullptr);
          auto pixel_color = fragment_shader(payload);
          */
          set_pixel({(float) x, (float) y}, pixel_color); // set pixel color
        }
      }
    }
  }
}

void rst::rasterizer::set_model(const Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff) {
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        fill(frame_buf.begin(), frame_buf.end(), Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        fill(depth_buf.begin(), depth_buf.end(), numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);

    texture = nullopt;
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-y)*width + x;
}

void rst::rasterizer::set_pixel(const Vector2i &point, const Vector3f &color)
{
    //old index: auto ind = point.y() + point.x() * width;
    int ind = (height-point.y())*width + point.x();
    frame_buf[ind] = color;
}

void rst::rasterizer::set_vertex_shader(function<Vector3f(vertex_shader_payload)> vert_shader)
{
    vertex_shader = vert_shader;
}

void rst::rasterizer::set_fragment_shader(function<Vector3f(fragment_shader_payload)> frag_shader)
{
    fragment_shader = frag_shader;
}


