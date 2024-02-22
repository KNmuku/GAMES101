#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <cmath>
#include <opencv2/opencv.hpp>

using namespace std;
constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos)
{
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle)
{
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the model matrix for rotating the triangle around the Z axis.
    // Then return it.
    float radian_angle = rotation_angle / 180.f * MY_PI;
    Eigen::Matrix4f rotate;
    rotate << cos(radian_angle), -sin(radian_angle), 0, 0,
              sin(radian_angle), cos(radian_angle) , 0, 0,
              0, 0, 1, 0,
              0, 0, 0, 1;
    model = rotate * model;
    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar)
{
    // Students will implement this function

    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    // TODO: Implement this function
    // Create the projection matrix for the given parameters.
    // Then return it.
    float radian_fov = eye_fov / 180.f * MY_PI;
    float top   = abs(zNear) * tan(radian_fov / 2);
    float bot   = -top;
    float right = top * aspect_ratio;
    float left  = -right; 
    float n     = -zNear;
    float f     = -zFar;

    Eigen::Matrix4f persp_to_ortho;
    Eigen::Matrix4f translate;
    Eigen::Matrix4f scale;
    Eigen::Matrix4f ortho;
    Eigen::Matrix4f reflect;
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
    reflect   <<      1, 0, 0, 0,
                      0, 1, 0, 0,
                      0, 0, -1, 0,
                      0, 0, 0, 1;

    ortho = scale * translate;
    projection = reflect * ortho * persp_to_ortho * projection;
    return projection;
}

Eigen::Matrix4f get_rotation(Vector3f axis, float angle) {
  float radian_angle = angle / 180.f * MY_PI;
  Eigen::Matrix3f random_rotate;
  Eigen::Matrix4f rotate = Eigen::Matrix4f::Identity();
  Eigen::Matrix3f i = Eigen::Matrix3f::Identity();
  Eigen::Matrix3f proj_to_axis, proj_to_compl, vert_to_plane;
  // Corresponding to projection_to_axis,
  //                  projection_to_complement,
  //                  vertical_to_plane;
  proj_to_axis       = axis * (axis.transpose());
  proj_to_compl = (i - proj_to_axis) * cos(radian_angle);   
  vert_to_plane   <<     0, -axis.z(), axis.y(),
                             axis.z(), 0, -axis.x(),
                             -axis.y(), axis.x(), 0;
  vert_to_plane *= sin(radian_angle);
  random_rotate = proj_to_axis + proj_to_compl + vert_to_plane;

  rotate.block<3,3>(0,0) = random_rotate;
  return rotate;
}

int main(int argc, const char** argv)
{
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        }
        else
            return 0;
    }

    rst::rasterizer r(700, 700);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -2}, {0, 2, -2}, {-2, 0, -2}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        
        // rotate around random axis n.
        float ang = 45.f/180.f * MY_PI;
        Vector3f n;
        n << 0, -sin(ang), cos(ang);
        r.set_model(get_rotation(n, angle));
        
        // r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50)); // change zNear & zFar to be negative

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(700, 700, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        }
        else if (key == 'd') {
            angle -= 10;
        }
    }

    return 0;
}
