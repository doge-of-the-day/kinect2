#ifndef KINECT2_INTERFACE_H
#define KINECT2_INTERFACE_H


/// SYSTEM
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

namespace kinect2 {
/**
 * @brief The Kinect2Interface class wraps the libfreenect device and frames.
 *        The interface runs a backend thread which gathers the data.
 */
class Kinect2Interface
{
public:
    /**
     * @brief The PipelineType enum to choose the execution type.
     */
    enum PipelineType {CPU, GL, OCL, CUDA, KDE_CUDA, KDE_OCL};

    /**
     * @brief The Stamped struct wraps arbitrary data types and makes
     *        it possible to assign time stamps.
     */
    template<typename T>
    struct Stamped {
        T data;
        uint32_t stamp;
    };

    /**
     * @brief The Data struct wraps up all frames which can be received
     *        from the kinect device.
     */
    struct Data {
        using Ptr = std::shared_ptr<Data>;

        Stamped<cv::Mat>                       ir;
        Stamped<cv::Mat>                       rgb;
        Stamped<cv::Mat>                       depth;
        Stamped<cv::Mat>                       depth_rectified;
        Stamped<cv::Mat>                       rgb_registered;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr points;
    };

    /**
     * @brief The CameraParameters struct wraps up all camera parameters, as
     *        well as the serial number, firmware version and other useful information.
     */
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
         const std::size_t width_rgb        = 1920;
         const std::size_t height_rgb       = 1080;
         const std::size_t size_rgb         = bpp * height_rgb * width_rgb;
         const std::size_t width_ir         = 512;
         const std::size_t height_ir        = 424;
         const std::size_t size_ir          = height_ir * width_ir * bpp;
    };

    /**
     * @brief The Parameters struct can be used to configure which frames
     *        should be converted for further usage with ros.
     *        Additionally, the computation pipeline type can be set.
     */
    struct Parameters {
        Kinect2Interface::PipelineType mode = CUDA;
        bool get_color                  = false;
        bool get_ir                   = true;
        bool get_depth                = false;
        bool get_depth_rectified      = false;
        bool get_color_registered       = false;

        bool activate_edge_filter     = false;
        bool activate_bilateral_filter= false;
    };


    /**
     * @brief Kinect2Interface constructor.
     */
    Kinect2Interface();
    /**
     * @brief ~Kinect2Interface destructor.
     */
    virtual ~Kinect2Interface();

    /**
     * @brief setup configures the interface and sets up all necessary frames and
     *        the data storage.
     * @param parameters    - the parameter object
     * @return
     */
    bool setup(const Parameters &parameters);

    /**
     * @brief isRunning returns if the background thread is active.
     * @return              - activity of the background thread
     */
    bool isRunning();
    /**
     * @brief start the background thread.
     * @return              - returns success of starting the background thread
     */
    bool start();
    /**
     * @brief stop the background thread
     * @return              - returns success of ending the background thread
     */
    bool stop();

    /**
     * @brief getData returns the currenlty filled up data object.
     *        This method is blocking for the foreground thread.
     * @return
     */
    Data::Ptr getData();
    /**
     * @brief getCameraParameters returns the camera parameters by reference.
     * @param camera_parameters     - reference to return the value
     * @return                      - returns if the camera parameter object is yet valid
     */
    bool getCameraParameters(CameraParameters &camera_parameters);


private:
    //// device information
    std::mutex                              camera_parameters_mutex_;
    CameraParameters                        camera_parameters_;

    //// background thread execution
    std::atomic_bool                        is_running_;
    std::atomic_bool                        shutdown_;
    std::thread                             thread_;

    //// necessary libfreenect2 components
    libfreenect2::Freenect2                 context_;
    libfreenect2::Freenect2DevicePtr        device_;
    libfreenect2::PacketPipeline           *pipeline_;
    libfreenect2::SyncMultiFrameListener   *listener_;
    libfreenect2::FrameMap                  frames_;

    libfreenect2::FramePtr                  frame_depth_undistorted_;
    libfreenect2::FramePtr                  frame_rgb_registered_;
    libfreenect2::RegistrationPtr           registration_;

    //// data storage to return read depth and rgb data
    std::mutex                              data_mutex_;
    std::condition_variable                 data_available_;
    Data::Ptr                               data_;

    //// parameters with which the interface was configured
    Parameters                              parameters_;

    /**
     * @brief loop resembles the main loop for the background process.
     */
    void loop();
    /**
     * @brief setupData initializes all buffers.
     */
    void setupData();
    /**
     * @brief doSetup sets up the computation pipeline.
     * @return  returns the successs of setting up the processing pipeline
     */
    bool doSetup();
};
}

#endif // KINECT2_INTERFACE_H
