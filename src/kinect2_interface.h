#ifndef KINECT2_INTERFACE_H
#define KINECT2_INTERFACE_H


#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>
#include <libfreenect2/config.h>

#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <atomic>
#include <iostream>
#include <memory>
#include <thread>
#include <mutex>
#include <condition_variable>

namespace libfreenect2 {
    using Freenect2DevicePtr = std::shared_ptr<Freenect2Device>;
    using SyncMultiFrameListenerPtr = std::shared_ptr<SyncMultiFrameListener>;
    using FramePtr = std::shared_ptr<libfreenect2::Frame>;
    using RegistrationPtr = std::shared_ptr<libfreenect2::Registration>;
}

class Kinect2Interface
{
public:
    enum PipelineType {CPU, GL, OCL, CUDA};

    template<typename T>
    struct Stamped {
        T data;
        uint32_t stamp;
    };

    struct Data {
        using Ptr = std::shared_ptr<Data>;

        Stamped<cv::Mat>                       ir;
        Stamped<cv::Mat>                       rgb;
        Stamped<cv::Mat>                       depth;
        Stamped<cv::Mat>                       depth_rectified;
        Stamped<cv::Mat>                       rgb_registered;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr points;
    };

    struct CameraParameters {
        using Ir = libfreenect2::Freenect2Device::IrCameraParams;
        using Color = libfreenect2::Freenect2Device::ColorCameraParams;
        using Ptr = std::shared_ptr<CameraParameters>;

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

         const std::size_t bpp              = 4;
         const std::size_t width_rgb        = 1080;
         const std::size_t height_rgb       = 1920;
         const std::size_t size_rgb         = bpp * height_rgb * width_rgb;
         const std::size_t width_ir         = 512;
         const std::size_t height_ir        = 424;
         const std::size_t size_ir          = height_ir * width_ir * bpp;
    };

    struct Parameters {
        Kinect2Interface::PipelineType mode = CUDA;
        bool get_rgb                  = false;
        bool get_ir                   = true;
        bool get_depth                = false;
        bool get_depth_rectified      = false;
        bool get_rgb_registered       = false;

        bool activate_edge_filter     = false;
        bool activate_bilateral_filter= false;
    };

    Kinect2Interface();
    virtual ~Kinect2Interface();

    bool setup(const Parameters &parameters);

    bool start();
    bool stop();

    Data::Ptr getData();
    bool getCameraParameters(CameraParameters &camera_parameters);


private:
    /// general information about kinect data
    std::mutex                              camera_parameters_mutex_;
    CameraParameters                        camera_parameters_;

    std::atomic_bool                        is_running_;
    std::atomic_bool                        shutdown_;

    libfreenect2::Freenect2                 context_;
    libfreenect2::Freenect2DevicePtr        device_;
    libfreenect2::PacketPipeline           *pipeline_;
    libfreenect2::SyncMultiFrameListener   *listener_;
    libfreenect2::FrameMap                  frames_;

    std::string                             serial_;

    libfreenect2::FramePtr                  frame_depth_undistorted_;
    libfreenect2::FramePtr                  frame_rgb_registered_;
    libfreenect2::RegistrationPtr           registration_;

    std::mutex                              data_mutex_;
    std::condition_variable                 data_available_;
    Data::Ptr                               data_;

    Parameters                              parameters_;

    std::thread                             thread_;

    void loop();
    void setupData();
    bool doSetup();
};

#endif // KINECT2_INTERFACE_H
