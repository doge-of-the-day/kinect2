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
                depth_to_color(j,
                               i,
                               map_x_ptr[pos],
                               map_y_ptr[pos]);
            }
        }

        camera_matrix_color_.at<float>(0,0) = params.color.fx;
        camera_matrix_color_.at<float>(1,1) = params.color.fy;
        camera_matrix_color_.at<float>(2,0) = params.color.cx;
        camera_matrix_color_.at<float>(2,1) = params.color.cy;
    }

    inline bool getRGBCoordinates(const std::size_t row,
                                  const std::size_t col,
                                  const float z,
                                  cv::Vec2f &pixel,
                                  const bool flip_horizontal = false)
    {
        assert(row < camera_parameters_.height_ir);
        assert(col < camera_parameters_.width_ir);


        const float *map_x_ptr = map_x_.ptr<float>();
        const float *map_y_ptr = map_y_.ptr<float>();

        const std::size_t pos = row * camera_parameters_.width_ir + col;

        const float mx = map_x_ptr[pos];
        const float my = map_y_ptr[pos];
        const float cx = (mx + (camera_parameters_.color.shift_m / z))
                                    * camera_parameters_.color.fx
                                    + camera_parameters_.color.cx;
        const int   cy = my;
        const int   c_off = cx + cy * camera_parameters_.width_rgb;

        if(c_off < 0 ||
                c_off >= camera_parameters_.width_rgb * camera_parameters_.height_rgb) {
            return false;
        }

        pixel[0] = flip_horizontal ? camera_parameters_.width_rgb - cx - 1 : cx;
        pixel[1] = cy;
        return true;
    }

    inline bool getTransformation(const std::vector<cv::Point3f> &points,
                                  const cv::Mat &depth)
    {
        /// reconstruct the 2D rgb image pixels
        std::vector<cv::Point2f> pixels;
        std::vector<cv::Point3f> valid_points;
        pixels.reserve(points.size());
        const float *depth_ptr = depth.ptr<float>();
        for(std::size_t i = 0 ; i < camera_parameters_.height_ir ; ++i) {
            for(std::size_t j = 0 ; j < camera_parameters_.width_ir ; ++j) {
                const std::size_t pos = i * camera_parameters_.width_ir + j;
                cv::Vec2f pixel;
                if(getRGBCoordinates(i,j,depth_ptr[pos], pixel)) {
                    pixels.emplace_back(pixel);
                    valid_points.emplace_back(points[pos]);
                }
            }
        }

        assert(pixels.size() == valid_points.size());

        cv::Mat rvec, tvec;
        cv::solvePnPRansac(valid_points,
                     pixels,
                     camera_matrix_color_,
                     cv::Mat(),
                     rvec,
                     tvec,
                     false, 2000, 0.5, 1000);

        std::cout << rvec << std::endl;
        std::cout << tvec << std::endl;

    }

private:
    const float                        depth_q_ = 0.01;
    const float                        color_q_ = 0.002199;

    cv::Mat                            map_x_;
    cv::Mat                            map_y_;

    cv::Mat                            camera_matrix_color_;
    Kinect2Interface::CameraParameters camera_parameters_;

    /**
     * @brief depth_to_color is used to build up the lookup tables for mapping depth pixels into
     *        the rgb image plane.
     * @param row   the row of the pixel in the depth image
     * @param col   the column of the pixel in the depth image
     * @param m_col the resulting mapped row value
     * @param m_row the resulting mapped column value
     */
    inline void depth_to_color(const int col,
                               const int row,
                               float    &m_col,
                               float    &m_row) const
    {
        m_col = (col - camera_parameters_.ir.cx) * depth_q_;
        m_row = (row - camera_parameters_.ir.cy) * depth_q_;

        double wx =
                (m_col * m_col * m_col * camera_parameters_.color.mx_x3y0) +
                (m_row * m_row * m_row * camera_parameters_.color.mx_x0y3) +
                (m_col * m_col * m_row * camera_parameters_.color.mx_x2y1) +
                (m_row * m_row * m_col * camera_parameters_.color.mx_x1y2) +
                (m_col * m_col * camera_parameters_.color.mx_x2y0) +
                (m_row * m_row * camera_parameters_.color.mx_x0y2) +
                (m_col * m_row * camera_parameters_.color.mx_x1y1) +
                (m_col * camera_parameters_.color.mx_x1y0) +
                (m_row * camera_parameters_.color.mx_x0y1) +
                (camera_parameters_.color.mx_x0y0);

        double wy =
                (m_col * m_col * m_col * camera_parameters_.color.my_x3y0) +
                (m_row * m_row * m_row * camera_parameters_.color.my_x0y3) +
                (m_col * m_col * m_row * camera_parameters_.color.my_x2y1) +
                (m_row * m_row * m_col * camera_parameters_.color.my_x1y2) +
                (m_col * m_col * camera_parameters_.color.my_x2y0) +
                (m_row * m_row * camera_parameters_.color.my_x0y2) +
                (m_col * m_row * camera_parameters_.color.my_x1y1) +
                (m_col * camera_parameters_.color.my_x1y0) +
                (m_row * camera_parameters_.color.my_x0y1) +
                (camera_parameters_.color.my_x0y0);

        m_col = (wx / (camera_parameters_.color.fx * color_q_)) -
             (camera_parameters_.color.shift_m / camera_parameters_.color.shift_d);
        m_row = (wy / color_q_) +
              camera_parameters_.color.cy;
    }
};




#endif // KINECT2_UNDISTORTION_MAPS_HPP
