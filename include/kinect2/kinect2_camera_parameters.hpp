#ifndef KINECT2_CAMERA_PARAMETERS_HPP
#define KINECT2_CAMERA_PARAMETERS_HPP

/// SYSTEM
#include <libfreenect2/libfreenect2.hpp>
#include <vector>
#include <boost/array.hpp>

namespace kinect2 {
/**
 * @brief The CameraParameters struct wraps up all camera parameters, as
 *        well as the serial number, firmware version and other useful information.
 */
struct CameraParameters {
    using Ir    = libfreenect2::Freenect2Device::IrCameraParams;
    using Color = libfreenect2::Freenect2Device::ColorCameraParams;
    using Ptr   = std::shared_ptr<CameraParameters>;

    Ir          ir;
    Color       color;

    std::string serial;
    std::string firmware;

    CameraParameters & operator = (CameraParameters &other)
    {
        ir = other.ir;
        color = other.color;
        serial = other.serial;
        firmware = other.firmware;
        return *this;
    }

    void getDistortionCoefficientsIR(std::vector<double> &D) const
    {
        D = {ir.k1, ir.k2, ir.p1, ir.p2, ir.k3};
    }

    void setDistortionCoefficientsIR(const std::vector<double> &D)
    {
        ir.k1 = D[0];
        ir.k2 = D[1];
        ir.p1 = D[2];
        ir.p2 = D[3];
        ir.k3 = D[4];
    }

    void getProjectionMatrixIR(boost::array<double, 12> &P) const
    {
        P[0]  = ir.fx;
        P[2]  = ir.cx;
        P[5]  = ir.fy;
        P[6]  = ir.cy;
        P[11] = 1.0;
    }

    void getRectificationMatrixIR(boost::array<double, 9> &R) const
    {
        R[0] = 1.0;
        R[4] = 1.0;
        R[8] = 1.0;
    }

    void getCameraMatrixIR(boost::array<double, 9> &K) const
    {
        K[0]  = ir.fx;
        K[2]  = ir.cx;
        K[4]  = ir.fy;
        K[5]  = ir.cy;
        K[8]  = 1.0;
    }

    void setCameraMatrixIR(const boost::array<double, 9> &K)
    {
        ir.fx = K[0];
        ir.cx = K[2];
        ir.fy = K[4];
        ir.cy = K[5];
    }

    void getDistortionCoefficientsRGB(std::vector<double> &D) const
    {
        D = {};
    }

    void getProjectionMatrixRGB(boost::array<double, 12> &P) const
    {
        P[0]  = color.fx;
        P[2]  = color.cx;
        P[5]  = color.fy;
        P[6]  = color.cy;
        P[11] = 1.0;
    }

    void getRectificationMatrixRGB(boost::array<double, 9> &R) const
    {
        R[0] = 1.0;
        R[4] = 1.0;
        R[8] = 1.0;
    }

    void getCameraMatrixRGB(boost::array<double, 9> &K) const
    {
        K[0]  = color.fx;
        K[2]  = color.cx;
        K[4]  = color.fy;
        K[5]  = color.cy;
        K[8]  = 1.0;
    }

    void setCameraMatrixRGB(const boost::array<double, 9> &K)
    {
        color.fx = K[0];
        color.cx = K[2];
        color.fy = K[4];
        color.cy = K[5];
    }

    void getMappingCoefficientsRGB(std::vector<double> &mx,
                                   std::vector<double> &my) const
    {
        mx.resize(10);
        my.resize(10);

        mx[0] = color.mx_x3y0;
        mx[1] = color.mx_x0y3;
        mx[2] = color.mx_x2y1;
        mx[3] = color.mx_x1y2;
        mx[4] = color.mx_x2y0;
        mx[5] = color.mx_x0y2;
        mx[6] = color.mx_x1y1;
        mx[7] = color.mx_x1y0;
        mx[8] = color.mx_x0y1;
        mx[9] = color.mx_x0y0;

        my[0] = color.my_x3y0;
        my[1] = color.my_x0y3;
        my[2] = color.my_x2y1;
        my[3] = color.my_x1y2;
        my[4] = color.my_x2y0;
        my[5] = color.my_x0y2;
        my[6] = color.my_x1y1;
        my[7] = color.my_x1y0;
        my[8] = color.my_x0y1;
        my[9] = color.my_x0y0;
    }

    void setMappingCoefficientsRGB(const std::vector<double> &mx,
                                   const std::vector<double> &my)
    {
        color.mx_x3y0 = mx[0];
        color.mx_x0y3 = mx[1];
        color.mx_x2y1 = mx[2];
        color.mx_x1y2 = mx[3];
        color.mx_x2y0 = mx[4];
        color.mx_x0y2 = mx[5];
        color.mx_x1y1 = mx[6];
        color.mx_x1y0 = mx[7];
        color.mx_x0y1 = mx[8];
        color.mx_x0y0 = mx[9];

        color.my_x3y0 = my[0];
        color.my_x0y3 = my[1];
        color.my_x2y1 = my[2];
        color.my_x1y2 = my[3];
        color.my_x2y0 = my[4];
        color.my_x0y2 = my[5];
        color.my_x1y1 = my[6];
        color.my_x1y0 = my[7];
        color.my_x0y1 = my[8];
        color.my_x0y0 = my[9];
    }


    const std::size_t bpp              = 4;
    const std::size_t width_rgb        = 1920;
    const std::size_t height_rgb       = 1080;
    const std::size_t size_rgb         = bpp * height_rgb * width_rgb;
    const std::size_t width_ir         = 512;
    const std::size_t height_ir        = 424;
    const std::size_t size_ir          = height_ir * width_ir * bpp;
};
}


#endif // KINECT2_CAMERA_PARAMETERS_HPP
