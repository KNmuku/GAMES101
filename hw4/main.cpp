#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>


std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) 
{
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4) 
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
        << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }     
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) 
{
    
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001) 
    {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                 3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

cv::Point2f recursive_bezier(const std::vector<cv::Point2f> &control_points, float t) 
{
    // TODO: Implement de Casteljau's algorithm
    if (control_points.size() == 1) {
        return control_points.at(0);
    } 
    std::vector<cv::Point2f> lerped(control_points.size() - 1);
    for (int i = 0; i < control_points.size() - 1; ++i) {
        lerped.at(i) = (1 - t) * control_points.at(i) + t * control_points.at(i + 1);
    }
    return recursive_bezier(lerped, t);
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) 
{
    // TODO: Iterate through all t = 0 to t = 1 with small steps, and call de Casteljau's 
    // recursive Bezier algorithm.
    for (double t = 0.0; t <= 1.0; t += 0.001) {
        auto point = recursive_bezier(control_points, t);
        window.at<cv::Vec3b>(point.y, point.x)[1] = 255;

        cv::Point2f this_pixel{floor(point.x), floor(point.y)};
        
        std::vector<cv::Point2f> points = {
            cv::Point2f(floor(point.x + .5f), floor(point.y + .5f)),
            cv::Point2f(floor(point.x + .5f), floor(point.y - .5f)),
            cv::Point2f(floor(point.x - .5f), floor(point.y + .5f)),
            cv::Point2f(floor(point.x - .5f), floor(point.y - .5f)),
        };

        cv::Point2f dis0 = cv::Point2f(this_pixel.x + .5f, this_pixel.y + .5f) - point;
        float d0 = pow(dis0.x, 2) + pow(dis0.y, 2); 

        for (int i = 0; i < points.size(); ++i) {
            cv::Point2f disi = points.at(i) - point;
            float di = pow(disi.x, 2) + pow(disi.y, 2); 
            float ratio = pow(d0 / di, .5);
            float origin = window.at<cv::Vec3b>(points.at(i).y, points.at(i).x)[1]; 
            window.at<cv::Vec3b>(points.at(i).y, points.at(i).x)[1] = std::max(origin, 255 * ratio);
        }
    }

}

int main(int argc, const char ** argv) 
{
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27) 
    {
        for (auto &point : control_points) 
        {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == 4) 
        {
            //naive_bezier(control_points, window);
            bezier(control_points, window);

            cv::imshow("Bezier Curve", window);

            if (argc == 2) {
                cv::imwrite(argv[1], window); 
            }

            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

    return 0;
}
