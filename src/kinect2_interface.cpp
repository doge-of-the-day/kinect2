#include "kinect2_interface.h"

#include <signal.h>
#include <cstring>


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

    switch(parameters.mode) {
    case CPU:
        pipeline_.reset(new libfreenect2::CpuPacketPipeline());
        break;
    case GL:
#ifdef LIBFREENECT2_WITH_OPENGL_SUPPORT
        pipeline_.reset(new libfreenect2::OpenGLPacketPipeline());
#else
        std::cerr << "[Kinect2Interface]: GL is not supported!" << std::endl;
        return false;
#endif
        break;
    case OCL:
#ifdef LIBFREENECT2_WITH_OPENCL_SUPPORT
        pipeline_.reset(new libfreenect2::OpenCLPacketPipeline(-1));
#else
        std::cerr << "[Kinect2Interface]: OCL is not supported!" << std::endl;
        return false;
#endif
        break;
    case CUDA:
#ifdef LIBFREENECT2_WITH_CUDA_SUPPORT
        pipeline_.reset(new libfreenect2::CudaPacketPipeline());
#else
        std::cerr << "[Kinect2Interface]: CUDA is not supported!" std::endl;
        return false;
#endif
        break;
    default:
        std::cerr << "[Kinect2Interface]: Unknown processing mode, using 'CPU'!" << std::endl;
        pipeline_.reset(new libfreenect2::CpuPacketPipeline());
        break;
    }

    serial_ = context_.getDefaultDeviceSerialNumber();
    device_.reset(context_.openDevice(serial_, pipeline_.get()));

    if(!device_) {
        std::cerr << "[Kinect2Interface]: Cannot open device!" << std::endl;
        return false;
    }

    /// setup the listeners
    listener_.reset(new libfreenect2::SyncMultiFrameListener(libfreenect2::Frame::Ir |
                                                             libfreenect2::Frame::Depth |
                                                             libfreenect2::Frame::Color));
    device_->setColorFrameListener(listener_.get());
    device_->setIrAndDepthFrameListener(listener_.get());

    frame_depth_undistorted_.reset(new libfreenect2::Frame (512, 424, 4));
    frame_rgb_registered_.reset(new libfreenect2::Frame (512, 424, 4));

    if(parameters.buffer_rgb)
        data_.rgb.data                = cv::Mat(height_rgb, width_rgb, CV_8UC4, cv::Scalar());
    if(parameters.buffer_ir)
        data_.ir.data                 = cv::Mat(height, width, CV_32FC1,  cv::Scalar());
    if(parameters.buffer_depth)
        data_.depth.data              = cv::Mat(height, width, CV_32FC1,  cv::Scalar());
    if(parameters.buffer_rgb_registered)
        data_.rgb_registered.data     = cv::Mat(height, width, CV_8UC4,   cv::Scalar());
    if(parameters.buffer_depth_undistorted)
        data_.depth_undistorted.data  = cv::Mat(height, width, CV_32FC1,  cv::Scalar());

    data_.points.height           = height;
    data_.points.width            = width;
    data_.points.is_dense         = true;
    data_.points.points.resize(height * width);

    return true;
}

bool Kinect2Interface::start()
{
    if(is_running_)
        return false;
    thread_ = std::thread([this](){loop();});
    thread_.detach();
}

bool Kinect2Interface::stop()
{
    if(!is_running_)
        return false;

    shutdown_ = true;
    if(thread_.joinable())
        thread_.join();
}

bool Kinect2Interface::getData(Bundle &data)
{
    std::unique_lock<std::mutex> l(data_mutex_);
    data_available_.wait(l);
    if(!is_running_)
        return false;
    data_.clone(data);
    return true;
}

void Kinect2Interface::loop()
{
    is_running_ = true;

    if(!device_->start()) {
        is_running_ = false;
    }

    std::cout << "device serial: " << device_->getSerialNumber() << std::endl;
    std::cout << "device firmware: " << device_->getFirmwareVersion() << std::endl;

    registration_.reset(new libfreenect2::Registration(device_->getIrCameraParams(),
                                                       device_->getColorCameraParams()));

    while(!shutdown_) {
        if(!listener_->waitForNewFrame(frames_, 10*1000)) {
            std::cerr << "[Kinect2Interface]: Listener Timeout!" << std::endl;
            shutdown_ = true;
        } else {
            libfreenect2::Frame *rgb    = frames_[libfreenect2::Frame::Color];
            libfreenect2::Frame *ir     = frames_[libfreenect2::Frame::Ir];
            libfreenect2::Frame *depth  = frames_[libfreenect2::Frame::Depth];

            registration_->apply(rgb, depth, frame_depth_undistorted_.get(), frame_rgb_registered_.get());

            {
                std::unique_lock<std::mutex> l(data_mutex_);
                /// BUFFER THE DATA IN THE BUNDLE OBJECT
                if(!data_.rgb.data.empty()) {
                    std::memcpy(data_.rgb.data.data, rgb->data, size_rgb);
                    data_.rgb.stamp = rgb->timestamp;
                }
                if(!data_.ir.data.empty()) {
                    std::memcpy(data_.ir.data.data, ir->data, size);
                    data_.ir.stamp = ir->timestamp;
                }
                if(!data_.depth.data.empty()) {
                    std::memcpy(data_.depth.data.data, depth->data, size);
                    data_.depth.stamp = depth->timestamp;
                }
                if(!data_.depth_undistorted.data.empty()) {
                    std::memcpy(data_.depth_undistorted.data.data, frame_depth_undistorted_->data, size);
                    data_.depth_undistorted.stamp = frame_depth_undistorted_->timestamp;
                }
                if(!data_.rgb_registered.data.empty()) {
                    std::memcpy(data_.rgb_registered.data.data, frame_rgb_registered_->data, size);
                    data_.rgb_registered.stamp = frame_rgb_registered_->timestamp;
                }

                pcl::PointXYZRGB *points_ptr = data_.points.points.data();
                for(std::size_t i = 0 ; i < height ; ++i) {
                    for(std::size_t j = 0 ; j < width ; ++j) {
                        pcl::PointXYZRGB &p = points_ptr[i * width + j];
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
}
