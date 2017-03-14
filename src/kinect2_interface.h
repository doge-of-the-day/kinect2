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
    using PacketPipelinePtr  = std::shared_ptr<PacketPipeline>;
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
        Stamped<cv::Mat>                       ir;
        Stamped<cv::Mat>                       rgb;
        Stamped<cv::Mat>                       depth;
        Stamped<cv::Mat>                       depth_undistorted;
        Stamped<cv::Mat>                       rgb_registered;
        pcl::PointCloud<pcl::PointXYZRGB>      points;

        void clone(Data &dst) const
        {
            if(!ir.data.empty()) {
                dst.ir.data                 = ir.data.clone();
                dst.ir.stamp                = ir.stamp;
            }
            if(!rgb.data.empty()) {
                dst.rgb.data                = rgb.data.clone();
                dst.rgb.stamp               = rgb.stamp;
            }
            if(!depth.data.empty()) {
                dst.depth.data              = depth.data.clone();
                dst.depth.stamp             = depth.stamp;
            }
            if(!depth_undistorted.data.empty()) {
                dst.depth_undistorted.data  = depth_undistorted.data.clone();
                dst.depth_undistorted.stamp = depth_undistorted.stamp;
            }
            if(!rgb_registered.data.empty()) {
                dst.rgb_registered.data     = rgb_registered.data.clone();
                dst.rgb_registered.stamp    = rgb_registered.stamp;
            }
            dst.points                      = points;
        }
    };

    struct CameraParameters {
        using Ir = libfreenect2::Freenect2Device::IrCameraParams;
        using Color = libfreenect2::Freenect2Device::ColorCameraParams;

         Ir          ir;
         Color       color;

         std::string serial;
         std::string firmware;

         const std::size_t bpp               = 4;
         const std::size_t width_rgb        = 1080;
         const std::size_t height_rgb       = 1920;
         const std::size_t size_rgb         = bpp * height_rgb * width_rgb;
         const std::size_t depth_width      = 512;
         const std::size_t depth_height     = 424;
         const std::size_t depth_size       = depth_height * depth_width * bpp;
    };

    struct Parameters {
        Kinect2Interface::PipelineType mode = CUDA;
        bool buffer_rgb               = false;
        bool buffer_ir                = true;
        bool buffer_depth             = false;
        bool buffer_depth_undistorted = false;
        bool buffer_rgb_registered    = false;

        bool activate_edge_filter     = false;
        bool activate_bilateral_filter= false;
    };

    Kinect2Interface();
    virtual ~Kinect2Interface();

    bool setup(const Parameters &parameters);

    bool start();
    bool stop();

    bool getData(Data &data);
    bool getCameraParameters(CameraParameters &camera_parameters);


private:
    /// general information about kinect data
    CameraParameters                        camera_parameters_;


    std::atomic_bool                        is_running_;
    std::atomic_bool                        shutdown_;

    libfreenect2::Freenect2                 context_;
    libfreenect2::Freenect2DevicePtr        device_;
    libfreenect2::PacketPipelinePtr         pipeline_;
    libfreenect2::SyncMultiFrameListenerPtr listener_;
    libfreenect2::FrameMap                  frames_;

    std::string                             serial_;

    libfreenect2::FramePtr                  frame_depth_undistorted_;
    libfreenect2::FramePtr                  frame_rgb_registered_;
    libfreenect2::RegistrationPtr           registration_;

    std::mutex                              data_mutex_;
    Data                                  data_;

    std::condition_variable                 data_available_;

    std::thread                             thread_;

    void loop();
};

#endif // KINECT2_INTERFACE_H
