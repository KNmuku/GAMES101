// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <cmath>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>

using namespace std;

rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

// helper function: inside triangle
// float (x, y) !!!!!
// !!!!
// !!!!
bool rst::rasterizer::insideTriangle(float x, float y, const array<Vector4f, 3> & _v) {   
    // TODO : Implement this function to check if the point (x, y) is inside the triangle represented by _v[0], _v[1], _v[2]
  const int NUM = 3;
  Eigen::Vector3f p;
  vector<Eigen::Vector3f> vs(NUM);

  p << x, y, 0.f;

  for (int i = 0; i < NUM; ++i) {
    vs[i] = Eigen::Vector3f{_v[i].x(), _v[i].y(), 0.f}; 
  }

  Eigen::Vector3f ver_to_ver[] = {vs[1]-vs[0], vs[2]-vs[1], vs[0]-vs[2]};
  Eigen::Vector3f ver_to_p[]   = {p-vs[0], p-vs[1], p-vs[2]};
   
  vector<float> z_values(NUM);
  for (int i = 0; i < NUM; ++i) {
    z_values[i] = ver_to_ver[i].cross(ver_to_p[i]).z();
  }

  return ((z_values[0] >= 0) && (z_values[1] >= 0) && (z_values[2] >= 0)) ||
         ((z_values[0] <= 0) && (z_values[1] <= 0) && (z_values[2] <= 0)); 

}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        std::array<Eigen::Vector3f, 3> viewspace_pos {
            (view * model * to_vec4(buf[i[0]], 1.f)).head<3>(),
            (view * model * to_vec4(buf[i[1]], 1.f)).head<3>(), 
            (view * model * to_vec4(buf[i[2]], 1.f)).head<3>()
        };

        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*vert.x() + 0.5*width;
            vert.y() = 0.5*height*vert.y() + 0.5*width;
            vert.z() = vert.z() * f1 + f2;
        }

        for (int j = 0; j < 3; ++j)
        {
            t.setVertex(j, v[j].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        msaa_rasterize_triangle(t, viewspace_pos);
    }

    msaa_merge();
}

void rst::rasterizer::msaa_merge() {
  for (int x = 0; x < width; ++x) {
    for (int y = 0; y < height; ++y) {
      int index = get_index(x, y);
      array<Vector3f, 4> msaa_samples = msaa_frame_buf.at(index);
      Vector3f average = (msaa_samples.at(0) + msaa_samples.at(1) + msaa_samples.at(2) + msaa_samples.at(3)) / 4.f;
      frame_buf.at(index) = average;
    }
  }
}

struct msaa_sample {
  float x_offset;
  float y_offset;
};

//Screen space rasterization
void rst::rasterizer::msaa_rasterize_triangle(const Triangle& t, const array<Vector3f, 3> & viewspace_pos) {
  std::array<Vector4f, 3> v4array = t.toVector4();
    
  // TODO : Find out the bounding box of current triangle.
  // iterate through the pixel and find if the current pixel is inside the triangle

  // aligned-axis pixel bounding box
  int left   = (int) floor(min(min(v4array[0].x(), v4array[1].x()), v4array[2].x()));
  int bottom = (int) floor(min(min(v4array[0].y(), v4array[1].y()), v4array[2].y()));
  int right  = (int)  ceil(max(max(v4array[0].x(), v4array[1].x()), v4array[2].x()));
  int top    = (int)  ceil(max(max(v4array[0].y(), v4array[1].y()), v4array[2].y()));


  for (int x = left; x < right; ++x) {
    for (int y = bottom; y < top; ++y) {
      // MSAA
      const int SPLIT = 4;
      array<msaa_sample, SPLIT> samples; // one pixel is splited as 
                                  // | ze | st |
                                  // | ——   —— |
                                  // | nd | rd |
      samples.at(0) = {.25f, .25f};
      samples.at(1) = {.75f, .25f};
      samples.at(2) = {.25f, .75f};
      samples.at(3) = {.75f, .75f};

      for (int i = 0; i < SPLIT; ++i) {
        float x_sample = x + samples.at(i).x_offset;
        float y_sample = y + samples.at(i).y_offset;
        if (insideTriangle(x_sample, y_sample, v4array)) {
          auto[alpha, beta, gamma] = computeBarycentric2D(x_sample, y_sample, t.v);

          float z_interpolated = 1.f / (alpha/(-viewspace_pos[0].z()) + beta/(-viewspace_pos[1].z()) + gamma/(-viewspace_pos[2].z()));
         
          int index = get_index(x, y); 
          if (z_interpolated < msaa_depth_buf.at(index).at(i)) {
            msaa_depth_buf.at(index).at(i) = z_interpolated;
            // set msaa_frame_buf pixel color
            msaa_frame_buf.at(index).at(i) = t.getColor();
          }
        }
      }

      /*
      const int SPLIT = 4;
      vector<int> msaa_inside(SPLIT);
      int sum = 0;
      sum += msaa_inside.at(0) = insideTriangle(x + 0.25f, y + .25f, v4array) ? 1 : 0; 
      sum += msaa_inside.at(1) = insideTriangle(x + .75f, y + .25f, v4array) ? 1 : 0; 
      sum += msaa_inside.at(2) = insideTriangle(x + .25f, y + .75f, v4array) ? 1 : 0; 
      sum += msaa_inside.at(3) = insideTriangle(x + .75f, y + .75f, v4array) ? 1 : 0; 

      float ratio = (float)sum / SPLIT;

      if (ratio) {
        
        auto[alpha, beta, gamma] = computeBarycentric2D(x + .5f, y + .5f, t.v);
        float w_reciprocal = 1.0/(alpha / v4array[0].w() + beta / v4array[1].w() + gamma / v4array[2].w());
        float z_interpolated = alpha * v4array[0].z() / v4array[0].w() + beta * v4array[1].z() / v4array[1].w()
                              + gamma * v4array[2].z() / v4array[2].w();
        z_interpolated *= w_reciprocal;

        int index = get_index(x, y);
        if (z_interpolated <= depth_buf.at(index)) {
          if (ratio == 1.0f) {
            depth_buf.at(index) = z_interpolated;
          }
          Eigen::Vector3f origin_color = frame_buf.at(index); 
          set_pixel({(float) x, (float) y, 0.f}, ratio * t.getColor() + (1-ratio) * origin_color);
        }
      }*/

      /*// NO MSAA
      if (insideTriangle(x + .5f, y + .5f, v4array)) {

        // If so, use the following code to get the interpolated z value.
        auto[alpha, beta, gamma] = computeBarycentric2D(x + .5f, y + .5f, t.v);
        float w_reciprocal = 1.0/(alpha / v4array[0].w() + beta / v4array[1].w() + gamma / v4array[2].w());
        float z_interpolated = alpha * v4array[0].z() / v4array[0].w() + beta * v4array[1].z() / v4array[1].w()
                                + gamma * v4array[2].z() / v4array[2].w();
        z_interpolated *= w_reciprocal;
    
        int index = get_index(x, y);

        if (z_interpolated < depth_buf.at(index)) {
            depth_buf.at(index) = z_interpolated;
            // TODO : set the current pixel (use the set_pixel function) to the color of the triangle 
            // (use getColor function) if it should be painted.
            set_pixel({(float) x, (float) y, 0.f}, t.getColor());
        }
      }*/
    } 
  }

}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m)
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        for (array<Vector3f,4> & samples : msaa_frame_buf) {
          for (Vector3f & color : samples) {
            color = Eigen::Vector3f{0, 0, 0}; 
          }
        }
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        for (array<float, 4> & samples : msaa_depth_buf) {
          for (float & depth : samples) {
            depth = std::numeric_limits<float>::infinity();
          }
        }
        //std::fill(depth_buf.begin(), depth_buf.end(), std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
    msaa_frame_buf.resize(w * h);
    msaa_depth_buf.resize(w * h);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = get_index(point.x(), point.y()); 
    frame_buf[ind] = color;

}

// clang-format on
