#ifndef KINECT2_UNDISTORTION_MAPS_HPP
#define KINECT2_UNDISTORTION_MAPS_HPP

#include "kinect2_interface.h"
#include <opencv2/opencv.hpp>


class Kinect2DepthToColorMap {
public:
    Kinect2DepthToColorMap(Kinect2Interface::CameraParameters &params) :
        map_depth_to_color_(params.height_ir * params.width_ir, 1, CV_32FC2, cv::Scalar()),
        camera_matrix_color_(cv::Mat::eye(cv::Size(3,3), CV_32FC1)),
        camera_parameters_(params)
    {
        cv::Vec2f *map_ptr = map_depth_to_color_.ptr<cv::Vec2f>();

        for(std::size_t i = 0 ; i < params.height_ir ; ++i) {
            for(std::size_t j = 0 ; j < params.width_ir ; ++j) {
                cv::Vec2i d(j,i);
                cv::Vec2f &c = map_ptr[i * params.width_ir + j];
                depth_to_color(d, c);
            }
        }

        camera_matrix_color_.at<float>(0,0) = params.color.fx;
        camera_matrix_color_.at<float>(1,1) = params.color.fy;
        camera_matrix_color_.at<float>(2,0) = params.color.cx;
        camera_matrix_color_.at<float>(2,1) = params.color.cy;
    }

    void getTransformation(const cv::Mat &points3d)
    {
//        cv::Mat camera_matrix;
//        cv::calibrateCamera(points3d,
//                            map_depth_to_color_,
//                            cv::Size(camera_parameters_.width_rgb,
//                                     camera_parameters_.height_rgb,
//                                     camera_matrix,
//                                     dist_co))

        cv::Mat rvec;
        cv::Mat tvec;
        cv::solvePnP(points3d,
                     map_depth_to_color_,
                     camera_matrix_color_,
                     cv::Mat(),
                     rvec,
                     tvec,
                     false,
                     CV_P3P);

        std::cout << rvec << std::endl;
        std::cout << tvec << std::endl;

    }


private:
    const float depth_q_ = 0.01;
    const float color_q_ = 0.002199;

    cv::Mat                            map_depth_to_color_;
    cv::Mat                            camera_matrix_color_;
    Kinect2Interface::CameraParameters camera_parameters_;

    inline void depth_to_color(const cv::Vec2i &d, cv::Vec2f &c) const
    {
        double mx = d[0] - camera_parameters_.ir.cx * depth_q_;
        double my = d[1] - camera_parameters_.ir.cy * depth_q_;

        double wx =
                (mx * mx * mx * camera_parameters_.color.mx_x3y0) +
                (my * my * my * camera_parameters_.color.mx_x0y3) +
                (mx * mx * my * camera_parameters_.color.mx_x2y1) +
                (my * my * mx * camera_parameters_.color.mx_x1y2) +
                (mx * mx * camera_parameters_.color.mx_x2y0) +
                (my * my * camera_parameters_.color.mx_x0y2) +
                (mx * my * camera_parameters_.color.mx_x1y1) +
                (mx * camera_parameters_.color.mx_x1y0) +
                (my * camera_parameters_.color.mx_x0y1) +
                (camera_parameters_.color.mx_x0y0);

        double wy =
                (mx * mx * mx * camera_parameters_.color.my_x3y0) +
                (my * my * my * camera_parameters_.color.my_x0y3) +
                (mx * mx * my * camera_parameters_.color.my_x2y1) +
                (my * my * mx * camera_parameters_.color.my_x1y2) +
                (mx * mx * camera_parameters_.color.my_x2y0) +
                (my * my * camera_parameters_.color.my_x0y2) +
                (mx * my * camera_parameters_.color.my_x1y1) +
                (mx * camera_parameters_.color.my_x1y0) +
                (my * camera_parameters_.color.my_x0y1) +
                (camera_parameters_.color.my_x0y0);

        c[0] = (wx / (camera_parameters_.color.fx * color_q_)) -
                (camera_parameters_.color.shift_m / camera_parameters_.color.shift_d);
        c[1] = (wy / color_q_) +
                camera_parameters_.color.cy;
    }



};




#endif // KINECT2_UNDISTORTION_MAPS_HPP
