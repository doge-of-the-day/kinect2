/// HEADER
#include <kinect2/kinect2_interface.h>

/// SYSTEM
#include <cstring>
#include <libfreenect2/packet_pipeline.h>

/// PROJECT
#include <kinect2/kinect2_depth_to_color_map.hpp>

using namespace kinect2;

Kinect2Interface::Kinect2Interface() :
    is_running_(false),
    shutdown_(false)
{
}

Kinect2Interface::~Kinect2Interface()
{
    stop();
}

bool Kinect2Interface::setup(const Parameters &parameters)
{
    if(is_running_)
        return false;

    std::cout << "freenect2 version: " << LIBFREENECT2_VERSION << std::endl;

    parameters_ = parameters;

    return true;
}

bool Kinect2Interface::isRunning()
{
    return is_running_;
}

bool Kinect2Interface::start()
{
    if(is_running_)
        return false;

    if(!doSetup())
        return false;

    shutdown_ = false;
    thread_ = std::thread([this](){loop();});
    thread_.detach();
    return true;
}

bool Kinect2Interface::stop()
{
    if(!is_running_)
        return false;

    shutdown_ = true;
    if(thread_.joinable())
        thread_.join();
    return true;
}

Kinect2Interface::Data::Ptr Kinect2Interface::getData()
{

    std::unique_lock<std::mutex> l(data_mutex_);
    if(!is_running_)
        return Data::Ptr();
    else
        data_available_.wait(l);

    return data_;
}

bool Kinect2Interface::getCameraParameters(CameraParameters &camera_parameters)
{
    std::unique_lock<std::mutex> l(camera_parameters_mutex_);
    camera_parameters = camera_parameters_;
    return !is_running_;
}

void Kinect2Interface::loop()
{
    if(!device_->start()) {
        is_running_ = false;
    }

    camera_parameters_.serial   = device_->getSerialNumber();
    camera_parameters_.firmware = device_->getFirmwareVersion();

    camera_parameters_.color    = device_->getColorCameraParams();
    camera_parameters_.ir       = device_->getIrCameraParams();

    registration_.reset(new libfreenect2::Registration(camera_parameters_.ir,
                                                       camera_parameters_.color));
    is_running_ = true;

    while(!shutdown_) {
        if(!listener_->waitForNewFrame(frames_, 1000)) {
            std::cerr << "[Kinect2Interface]: Listener Timeout!" << std::endl;
            shutdown_ = true;
        } else {
            libfreenect2::Frame *rgb    = frames_[libfreenect2::Frame::Color];
            libfreenect2::Frame *ir     = frames_[libfreenect2::Frame::Ir];
            libfreenect2::Frame *depth  = frames_[libfreenect2::Frame::Depth];

            registration_->apply(rgb, depth, frame_depth_undistorted_.get(), frame_rgb_registered_.get());

            {
                std::unique_lock<std::mutex> l(data_mutex_);
                setupData();
                /// BUFFER THE DATA IN THE BUNDLE OBJECT
                auto now =
                        std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now());
                long now_ns =
                        std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();

                if(parameters_.get_color) {
                    std::memcpy(data_->rgb.data.data, rgb->data, camera_parameters_.size_rgb);
                    data_->rgb.stamp = now_ns;
                }
                if(parameters_.get_ir) {
                    std::memcpy(data_->ir.data.data, ir->data, camera_parameters_.size_ir);
                    data_->ir.stamp = now_ns;
                }
                if(parameters_.get_depth) {
                    std::memcpy(data_->depth.data.data, depth->data, camera_parameters_.size_ir);
                    data_->depth.stamp = now_ns;
                }
                if(parameters_.get_depth_rectified) {
                    std::memcpy(data_->depth_rectified.data.data, frame_depth_undistorted_->data, camera_parameters_.size_ir);
                    data_->depth_rectified.stamp = now_ns;
                }
                if(parameters_.get_color_registered) {
                    std::memcpy(data_->rgb_registered.data.data, frame_rgb_registered_->data, camera_parameters_.size_ir);
                    data_->rgb_registered.stamp = now_ns;
                }

                pcl::PointXYZRGB *points_ptr = data_->points->points.data();
                data_->points->header.stamp = now_ms / 1000;
		for(std::size_t i = 0 ; i < camera_parameters_.height_ir ; ++i) {
                    for(std::size_t j = 0 ; j < camera_parameters_.width_ir ; ++j) {
                        pcl::PointXYZRGB &p = points_ptr[i * camera_parameters_.width_ir + j];
                        registration_->getPointXYZRGB(frame_depth_undistorted_.get(), frame_rgb_registered_.get(), i, j, p.x, p.y, p.z, p.rgb);
                        p.x = -p.x;
                    }
                }
            }

            data_available_.notify_one();
            listener_->release(frames_);
        }
    }

    device_->stop();
    device_->close();
    is_running_ = false;
    data_available_.notify_one(); /// drop waiting thread
    device_.reset();
}

void Kinect2Interface::setupData()
{
    data_.reset(new Data);

    if(parameters_.get_color)
        data_->rgb.data                = cv::Mat(camera_parameters_.height_rgb,
                                                 camera_parameters_.width_rgb,
                                                 CV_8UC4, cv::Scalar());
    if(parameters_.get_ir)
        data_->ir.data                 = cv::Mat(camera_parameters_.height_ir,
                                                 camera_parameters_.width_ir,
                                                 CV_32FC1, cv::Scalar());
    if(parameters_.get_depth)
        data_->depth.data              = cv::Mat(camera_parameters_.height_ir,
                                                 camera_parameters_.width_ir,
                                                 CV_32FC1, cv::Scalar());
    if(parameters_.get_color_registered)
        data_->rgb_registered.data     = cv::Mat(camera_parameters_.height_ir,
                                                 camera_parameters_.width_ir,
                                                 CV_8UC4, cv::Scalar());
    if(parameters_.get_depth_rectified)
        data_->depth_rectified.data    = cv::Mat(camera_parameters_.height_ir,
                                                 camera_parameters_.width_ir,
                                                 CV_32FC1, cv::Scalar());

    data_->points.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    data_->points->height  = camera_parameters_.height_ir;
    data_->points->width    = camera_parameters_.width_ir;
    data_->points->is_dense = true;
    data_->points->points.resize(camera_parameters_.height_ir *
                                 camera_parameters_.width_ir);

}

bool Kinect2Interface::doSetup()
{
    switch(parameters_.mode) {
    case CPU:
        pipeline_ = (new libfreenect2::CpuPacketPipeline());
        break;
    case GL:
#ifdef LIBFREENECT2_WITH_OPENGL_SUPPORT
        pipeline_ = (new libfreenect2::OpenGLPacketPipeline());
#else
        std::cerr << "[Kinect2Interface]: GL is not supported!" << std::endl;
        return false;
#endif
        break;
    case OCL:
#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
        pipeline_ = (new libfreenect2::OpenCLPacketPipeline(-1));
#else
        std::cerr << "[Kinect2Interface]: OCL is not supported!" << std::endl;
        return false;
#endif
        break;
    case CUDA:
#ifdef LIBFREENECT2_WITH_CUDA_SUPPORT
        pipeline_ = (new libfreenect2::CudaPacketPipeline());
#else
        std::cerr << "[Kinect2Interface]: CUDA is not supported!" << std::endl;
        return false;
#endif
        break;
    case KDE_CUDA:
#ifdef LIBFREENECT2_WITH_CUDA_SUPPORT
        pipeline_ = (new libfreenect2::CudaKdePacketPipeline());
#else
        std::cerr << "[Kinect2Interface]: CUDA is not supported!" << std::endl;
        return false;
#endif
        break;
    case KDE_OCL:
#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
        pipeline_ = (new libfreenect2::OpenCLKdePacketPipeline());
#else
        std::cerr << "[Kinect2Interface]: OCL is not supported!" << std::endl;
        return false;
#endif
        break;
    default:
        std::cerr << "[Kinect2Interface]: Unknown processing mode, using 'CPU'!" << std::endl;
        pipeline_ = (new libfreenect2::CpuPacketPipeline());
        break;
    }

    camera_parameters_.serial = context_.getDefaultDeviceSerialNumber();

    /// setup the listeners
    listener_ = (new libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Ir |
                                                          libfreenect2::Frame::Depth |
                                                          libfreenect2::Frame::Color));

    frame_depth_undistorted_.reset(new libfreenect2::Frame (camera_parameters_.width_ir,
                                                            camera_parameters_.height_ir,
                                                            camera_parameters_.bpp));
    frame_rgb_registered_.reset(new libfreenect2::Frame (camera_parameters_.width_ir,
                                                         camera_parameters_.height_ir,
                                                         camera_parameters_.bpp));


    device_.reset(context_.openDevice(camera_parameters_.serial, pipeline_));

    if(!device_) {
        std::cerr << "[Kinect2Interface]: Cannot open device!" << std::endl;
        return false;
    }

    device_->setColorFrameListener(listener_);
    device_->setIrAndDepthFrameListener(listener_);

    return true;
}
