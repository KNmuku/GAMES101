//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>

class Texture{
private:
    cv::Mat image_data;

public:
    Texture(const std::string& name)
    {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v)
    {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }
    
    // Bilinear interpolation
    Eigen::Vector3f getColorBilinear(float u, float v) {
      float u_img = u * width;
      float v_img = (1 - v) * height;
      //std::cout << "(u,v)" << "(" << u_img << "," << v_img << ")" << std::endl;
      
      int u_center = u_img;
      int v_center = v_img;
      u_center = (u_img - u_center) >= .5 ? u_center+1 : u_center;
      v_center = (v_img - v_center) >= .5 ? v_center+1 : v_center;


      auto color00 = image_data.at<cv::Vec3b>(v_center - .5, u_center - .5);
      auto color10 = image_data.at<cv::Vec3b>(v_center - .5, u_center + .5);
      auto color01 = image_data.at<cv::Vec3b>(v_center + .5, u_center - .5);
      auto color11 = image_data.at<cv::Vec3b>(v_center + .5, u_center + .5);

      /*
      if (!(color00 == color10 && color10 == color01 && color01 == color11)) {

      std::cout << "color00\n" << Eigen::Vector3f(color00[0], color00[1], color00[2]) << std::endl;
      std::cout << "color10\n" << Eigen::Vector3f(color10[0], color10[1], color10[2]) << std::endl;
      std::cout << "color01\n" << Eigen::Vector3f(color01[0], color01[1], color01[2]) << std::endl;
      std::cout << "color11\n" << Eigen::Vector3f(color11[0], color11[1], color11[2]) << std::endl;
      }*/

      float s = (u_img - u_center + .5f) / 1.f;
      float t = (v_img - v_center + .5f) / 1.f;
      // note: floating point number addition has no associative,
      // so this formula "color00 + s * (color10 - color00)" will lose much accuracy!

      auto color0 = (1-s) * color00 + s * color10;
      auto color1 = (1-s) * color01 + s * color11;
      auto color  = (1-t) * color0 + t * color1;
      Eigen::Vector3f returned_color(color[0], color[1], color[2]); 
      
      /*
      if (!(color00 == color10 && color10 == color01 && color01 == color11)) {
      std::cout << "getColor: " << getColor(u, v) << std::endl;
      std::cout << "Bilinear: " << returned_color << std::endl;
      }*/
      return returned_color;
    }

};
#endif //RASTERIZER_TEXTURE_H
