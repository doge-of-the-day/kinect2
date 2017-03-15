#ifndef KINECT2_UNDISTORTION_MAPS_HPP
#define KINECT2_UNDISTORTION_MAPS_HPP

#include "kinect2_interface.h"
#include <opencv2/opencv.hpp>



class Kinect2DepthToColorMap {
public:
    Kinect2DepthToColorMap(Kinect2Interface::CameraParameters &params) :
        map_x_(params.height_ir, params.width_ir, CV_32FC1, cv::Scalar()),
        map_y_(params.height_ir, params.width_ir, CV_32FC1, cv::Scalar()),
        camera_matrix_color_(cv::Mat::eye(cv::Size(3,3), CV_32FC1)),
        camera_parameters_(params)
    {
        float * map_x_ptr = map_x_.ptr<float>();
        float * map_y_ptr = map_y_.ptr<float>();

        for(std::size_t i = 0 ; i < params.height_ir ; ++i) {
            for(std::size_t j = 0 ; j < params.width_ir ; ++j) {
                const std::size_t pos = i * params.width_ir +j;
                depth_to_color(i,
                               j,
                               map_x_ptr[pos],
                               map_y_ptr[pos]);
            }
        }

        camera_matrix_color_.at<float>(0,0) = params.color.fx;
        camera_matrix_color_.at<float>(1,1) = params.color.fy;
        camera_matrix_color_.at<float>(2,0) = params.color.cx;
        camera_matrix_color_.at<float>(2,1) = params.color.cy;
    }


    bool getRGBCoordinates(const std::size_t row,
                           const std::size_t col,
                           const float z,
                           cv::Vec2i &pixel)
    {
        assert(row < camera_parameters_.height_ir);
        assert(col < camera_parameters_.width_ir);


        const float *map_x_ptr = map_x_.ptr<float>();
        const float *map_y_ptr = map_y_.ptr<float>();

        const std::size_t pos = row * camera_parameters_.width_ir + col;

        const float mx = map_x_ptr[pos];
        const float my = map_y_ptr[pos];
        const int   cx = std::floor((mx + (camera_parameters_.color.shift_m / z))
                                    * camera_parameters_.color.fx
                                    + camera_parameters_.color.cx
                                    + 0.5f);
        const int   cy = std::floor(my + 0.5f);
        const int   c_off = cx + cy * camera_parameters_.width_rgb;

        if(c_off < 0 ||
                c_off >= camera_parameters_.width_rgb * camera_parameters_.height_rgb) {
            return false;
        }

        pixel[0] = cx;
        pixel[1] = cy;
        return true;
    }



//    void getTransformation(const cv::Mat &points3d)
//    {

//        cv::Mat offsets(camera_parameters_.height_ir,
//                        camera_parameters_.width_ir,
//                        CV_32SC1,
//                        cv::Scalar());

//        int *offsets_ptr = offsets.ptr<int>();
//        for(std::size_t i = 0 ; i < camera_parameters_.height_ir ; ++i) {
//            for(std::size_t j = 0 ; j < camera_parameters_.width_ir ; ++j) {

//            }
//        }



////        cv::Mat camera_matrix;
////        cv::calibrateCamera(points3d,
////                            map_depth_to_color_,
////                            cv::Size(camera_parameters_.width_rgb,
////                                     camera_parameters_.height_rgb,
////                                     camera_matrix,
////                                     dist_co))

//        cv::Mat rvec;
//        cv::Mat tvec;
//        cv::solvePnP(points3d,
//                     map_depth_to_color_,
//                     camera_matrix_color_,
//                     cv::Mat(),
//                     rvec,
//                     tvec,
//                     false,
//                     CV_P3P);

//        std::cout << rvec << std::endl;
//        std::cout << tvec << std::endl;

//    }


private:
    const float depth_q_ = 0.01;
    const float color_q_ = 0.002199;

    cv::Mat     map_x_;
    cv::Mat     map_y_;

    cv::Mat                            camera_matrix_color_;
    Kinect2Interface::CameraParameters camera_parameters_;

    inline void depth_to_color(const int x,
                               const int y,
                               float    &mx,
                               float    &my) const
    {
        mx = (x - camera_parameters_.ir.cx) * depth_q_;
        my = (y - camera_parameters_.ir.cy) * depth_q_;

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

        mx = (wx / (camera_parameters_.color.fx * color_q_)) -
             (camera_parameters_.color.shift_m / camera_parameters_.color.shift_d);
        my = (wy / color_q_) +
              camera_parameters_.color.cy;
    }
};




#endif // KINECT2_UNDISTORTION_MAPS_HPP
