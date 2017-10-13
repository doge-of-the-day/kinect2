#ifndef KINECT2_UNDISTORTION_MAPS_HPP
#define KINECT2_UNDISTORTION_MAPS_HPP

#include <opencv2/opencv.hpp>
#include <kinect2/kinect2_interface.h>

namespace kinect2 {
/**
 * @brief The Kinect2DepthToColorMap class can be used to map 3D points / depth values
 *        and indices to the full-hd rgb image.
 */
class Kinect2DepthToColorMap {
public:
    /**
     * @brief Kinect2DepthToColorMap constructor.
     * @param params    - the camera parameters from the kinect device
     */
    Kinect2DepthToColorMap(CameraParameters &params) :
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
                depth2color(j,
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

    inline float projectUndistortedDepth(const cv::Vec3f &pt,
                             std::size_t &row,
                             std::size_t &col)
    {
        const float depth = std::sqrt(pt.dot(pt));
        int c = pt[0] * camera_parameters_.ir.fx / (depth) + camera_parameters_.ir.cx - 0.5f;
        int r = pt[1] * camera_parameters_.ir.fy / (depth) + camera_parameters_.ir.cy - 0.5f;

        if( r >(int) camera_parameters_.height_ir){
            row = camera_parameters_.height_ir;
        }
        else if( r < 0){
            row = 0;
        }
        else{
            row = r;
        }

        if( c > (int) camera_parameters_.width_ir){
            col = camera_parameters_.width_ir;
        }
        else if( c < 0){
            col = 0;
        }
        else{
            col = c;
        }

        return depth;

    }



    /**
     * @brief getRGBCoordinates return column and row indices defined within the
     *        full-hd rgb frame, given row and column in the depth frame.
     * @param row               - the row
     * @param col               - the column
     * @param z                 - the depth
     * @param pixel             - the resulting pixel coordinates
     * @param flip_horizontal   - tell the conversion to flip horizontally
     * @return
     */
    inline bool getRGBCoordinates(const std::size_t row,
                                  const std::size_t col,
                                  const float z,
                                  cv::Vec2f &pixel,
                                  const bool flip_horizontal = false)
    {
        assert(row < camera_parameters_.height_ir);
        assert(col < camera_parameters_.width_ir);

        const int size_rgb = camera_parameters_.width_rgb * camera_parameters_.height_rgb;
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
                c_off >= size_rgb) {
            return false;
        }

        pixel[0] = flip_horizontal ? camera_parameters_.width_rgb - cx - 1 : cx;
        pixel[1] = cy;
        return true;
    }

private:
    const float         depth_q_ = 0.01;
    const float         color_q_ = 0.002199;

    cv::Mat             map_x_;
    cv::Mat             map_y_;

    cv::Mat             camera_matrix_color_;
    CameraParameters    camera_parameters_;

    /**
     * @brief depth_to_color is used to build up the lookup tables for mapping depth pixels into
     *        the rgb image plane.
     * @param row   the row of the pixel in the depth image
     * @param col   the column of the pixel in the depth image
     * @param m_col the resulting mapped row value
     * @param m_row the resulting mapped column value
     */
    inline void depth2color(const int col,
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
}




#endif // KINECT2_UNDISTORTION_MAPS_HPP
