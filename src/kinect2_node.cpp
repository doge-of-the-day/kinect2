/// HEADER
#include <kinect2/kinect2_node.h>
#include <Eigen/Core>

/// SYSTEM
#include <sensor_msgs/distortion_models.h>
#include <sensor_msgs/image_encodings.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <functional>

using namespace kinect2;

Kinect2Node::Kinect2Node() :
    nh_private_("~")
{
}

Kinect2Node::~Kinect2Node()
{
    kinterface_.stop();
}

bool Kinect2Node::setup()
{
    const std::string topic_rgb               = nh_private_.param<std::string>("topic_rgb",                 "/kinect2/image_rgb/raw");
    const std::string topic_rgb_info          = nh_private_.param<std::string>("topic_rgb_info",            "/kinect2/image_rgb/camera_info");
    const std::string topic_rgb_rectified     = nh_private_.param<std::string>("topic_rgb_rectified",       "/kinect2/image_rgb/retified");
    const std::string topic_rgb_registered    = nh_private_.param<std::string>("topic_color_registered",    "/kinect2/rgb_registered");
    const std::string topic_depth             = nh_private_.param<std::string>("topic_depth",               "/kinect2/depth/raw");
    const std::string topic_depth_info        = nh_private_.param<std::string>("topic_depth_info",          "/kinect2/depth/camera_info");
    const std::string topic_depth_rectified   = nh_private_.param<std::string>("topic_depth_rectified",     "/kinect2/depth/rectified");
    const std::string topic_ir                = nh_private_.param<std::string>("topic_ir",                  "/kinect2/image_ir/raw");
    const std::string topic_ir_info           = nh_private_.param<std::string>("topic_ir_info",             "/kinect2/image_ir/camera_info");
    const std::string topic_ir_rectified      = nh_private_.param<std::string>("topic_ir_rectified",        "/kinect2/image_ir/rectified");
    const std::string topic_pointcloud        = nh_private_.param<std::string>("topic_rgb",                 "/kinect2/points");
    const std::string service_name_wakeup     = nh_private_.param<std::string>("service_name_wakeup",       "/kinect2/wakeup");
    const std::string service_name_sleep      = nh_private_.param<std::string>("service_name_sleep",        "/kinect2/sleep");
    const std::string calibration_color       = nh_private_.param<std::string>("calibration_color",         "");
    const std::string calibration_depth       = nh_private_.param<std::string>("calibration_depth",         "");
    const std::string calibration_ir          = nh_private_.param<std::string>("calibration_ir",            "");
    const std::string calibration_pose        = nh_private_.param<std::string>("calibration_pose",          "");
    const double      time_offset_ir          = nh_private_.param<double>("time_offset_ir", 0.0);
    const double      time_offset_rgb         = nh_private_.param<double>("time_offset_rgb", 0.0);;

    /// load the calibration files
    if(!kcalibration_.loadColorCalibration(calibration_color)) {
        std::cerr << "[Kinect2Node]: Error loading a calibration file '" << calibration_color << "'!" << std::endl;
        return false;
    }
    if(!kcalibration_.loadDepthCalibration(calibration_depth)) {
        std::cerr << "[Kinect2Node]: Error loading calibration file '" << calibration_depth << "'!" << std::endl;
        return false;
    }
    if(!kcalibration_.loadIrCalibration(calibration_ir)) {
        std::cerr << "[Kinect2Node]: Error loading calibration file '" << calibration_ir << "'!" << std::endl;
        return false;
    }
    if(!kcalibration_.loadExtrinsicCalibration(calibration_pose)) {
        std::cerr << "[Kinect2Node]: Error loading calibration file '" << calibration_pose << "'!" << std::endl;
        return false;
    }

    time_offset_ir_.fromNSec(static_cast<long>(std::floor(1e9 * time_offset_ir + 0.5)));
    time_offset_rgb_.fromNSec(static_cast<long>(std::floor(1e9 * time_offset_rgb + 0.5)));

    pub_rate_preferred_       = nh_private_.param<double>("preferred_publication_rate",     -1.0);
    frame_id_rgb_             = nh_private_.param<std::string>("frame_id_color",            "kinect2_color_optical_frame");
    frame_id_ir_              = nh_private_.param<std::string>("frame_id_ir",               "kinect2_depth_optical_frame");

    publish_rgb_              = nh_private_.param<bool>("publish_rgb", false);
    publish_rgb_rectified_    = nh_private_.param<bool>("publish_rgb_rectified", false);
    publish_rgb_registered_   = nh_private_.param<bool>("publish_rbg_registered", false);
    publish_ir_               = nh_private_.param<bool>("publish_ir",    false);
    publish_ir_rectified_     = nh_private_.param<bool>("publish_ir_rectified", false);
    publish_depth_            = nh_private_.param<bool>("publish_depth", false);
    publish_depth_rectified_  = nh_private_.param<bool>("publish_depth_rectified", false);

    kinterface_parameters_.get_color          = true;
    kinterface_parameters_.get_ir             = true;
    kinterface_parameters_.get_depth          = true;

    const std::string pipeline_type = nh_private_.param<std::string>("pipeline_type", "CUDA");
    if(pipeline_type == "GL") {
        kinterface_parameters_.mode = Kinect2Interface::GL;
    } else if(pipeline_type == "OCL") {
        kinterface_parameters_.mode = Kinect2Interface::OCL;
    } else if(pipeline_type == "CUDA") {
        kinterface_parameters_.mode = Kinect2Interface::CUDA;
    } else if(pipeline_type == "KDE_CUDA") {
        kinterface_parameters_.mode = Kinect2Interface::KDE_CUDA;
    } else if(pipeline_type == "KDE_OCL") {
        kinterface_parameters_.mode = Kinect2Interface::KDE_OCL;
    } else {
        kinterface_parameters_.mode = Kinect2Interface::CPU;
    }

    if(!kinterface_.setup(kinterface_parameters_)) {
        std::cerr << "[Kinect2Node]: Cannot setup Kinect2Interface!" << std::endl;
        ros::shutdown();
    }

    if(publish_rgb_) {
        pub_rgb_ = nh_.advertise<sensor_msgs::Image>(topic_rgb, 1);
        pub_rgb_info_ = nh_.advertise<sensor_msgs::CameraInfo>(topic_rgb_info, 1);
    }
    if(publish_rgb_registered_) {
        pub_rgb_registered_ = nh_.advertise<sensor_msgs::Image>(topic_rgb_registered, 1);
    }
    if(publish_rgb_rectified_) {
        pub_depth_rectified_ = nh_.advertise<sensor_msgs::Image>(topic_rgb_rectified, 1);
    }

    if(publish_depth_) {
        pub_depth_ = nh_.advertise<sensor_msgs::Image>(topic_depth, 1);
    }
    if(publish_depth_rectified_) {
        pub_depth_rectified_ = nh_.advertise<sensor_msgs::Image>(topic_depth_rectified, 1);
    }
    if(publish_depth_ || publish_depth_rectified_) {
        pub_depth_info_ = nh_.advertise<sensor_msgs::CameraInfo>(topic_depth_info, 1);
    }

    if(publish_ir_) {
        pub_ir_ = nh_.advertise<sensor_msgs::Image>(topic_ir, 1);
    }
    if(publish_ir_rectified_) {
        pub_ir_rectified_ = nh_.advertise<sensor_msgs::Image>(topic_ir_rectified, 1);
    }

    if(publish_ir_ || publish_ir_rectified_) {
        pub_ir_info_ = nh_.advertise<sensor_msgs::CameraInfo>(topic_ir_info, 1);
    }

    pub_pointcloud_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB>>(topic_pointcloud, 1);
    service_sleep_  = nh_.advertiseService(service_name_sleep, &Kinect2Node::sleep, this);
    service_wakeup_ = nh_.advertiseService(service_name_wakeup, &Kinect2Node::wakeup, this);

    return kinterface_.start();
}

int Kinect2Node::run()
{
    cv::namedWindow("debug", 0);

    if(pub_rate_preferred_ <= 0.0) {
        while(ros::ok()) {
            if(kinterface_.isRunning()) {
                publish();
            } else {
                ros::Rate(1.0).sleep();
            }
            ros::spinOnce();
        }
    } else {
        ros::Rate rate(pub_rate_preferred_);
        while(ros::ok()) {
            if(kinterface_.isRunning()) {
                publish();
                rate.sleep();
            } else {
                ros::Rate(1.0).sleep();
            }
        }
    }
    kinterface_.stop();
    ros::shutdown();
    return 0;
}

void Kinect2Node::publish()
{
    auto convertRGB = [](const Kinect2Interface::Stamped<cv::Mat> &rgb,
            const std::string &frame_id,
            sensor_msgs::Image::Ptr &image){
        const std::size_t rows         = rgb.data.rows;
        const std::size_t cols         = rgb.data.cols;
        const std::size_t rgb_channels = rgb.data.channels();
        const std::size_t channels     = 3;

        if(!image) {
            image.reset(new sensor_msgs::Image);
            const std::size_t bbc = 1;

            image->height       = rows;
            image->width        = cols;
            image->encoding     = sensor_msgs::image_encodings::BGR8;
            image->is_bigendian = false;
            image->step         = cols * channels * bbc;
            image->data.resize(channels * rows * cols);
            image->header.frame_id = frame_id;
        }

        image->header.stamp.fromNSec(rgb.stamp);

        const uchar * rgb_ptr = rgb.data.data;
        uchar * image_ptr = image->data.data();
        for(std::size_t i = 0 ; i < rows ; ++i) {
            for(std::size_t j = 0 ; j < cols ; ++j) {
                for(std::size_t k = 0 ; k < 3 ; ++k) {
                    image_ptr[(i * cols + j) * channels + k] =
                            rgb_ptr[(i * cols + (cols - 1 - j)) * rgb_channels + k];
                }
            }
        }
    };

    auto convertFloat = [](const Kinect2Interface::Stamped<cv::Mat> &mat,
            const std::string &frame_id,
            sensor_msgs::Image::Ptr &image)
    {
        const std::size_t rows = mat.data.rows;
        const std::size_t cols = mat.data.cols;
        const std::size_t bbp = 2;

        if(!image) {
            image.reset(new sensor_msgs::Image);
            image->height       = rows;
            image->width        = cols;
            image->encoding     = sensor_msgs::image_encodings::MONO16;
            image->is_bigendian = false;
            image->step         = mat.data.cols * bbp;
            image->data.resize(bbp * rows * cols);
            image->header.frame_id = frame_id;
        }

        image->header.stamp.fromNSec(mat.stamp);

        const float * mat_ptr = mat.data.ptr<float>();
        ushort * image_ptr = (ushort*) image->data.data();
        for(std::size_t i = 0 ; i < rows ; ++i) {
            for(std::size_t j = 0 ; j < cols ; ++j) {
                image_ptr[i * cols + j] =
                        mat_ptr[i * cols + (cols - j - 1)];
            }
        }
    };

    auto clone = [](const Kinect2Interface::Stamped<cv::Mat> &s)
    {
        Kinect2Interface::Stamped<cv::Mat> clone;
        clone.stamp = s.stamp;
        clone.data  = s.data.clone();
        return clone;
    };

    /// reprojection and stuff
    Kinect2Interface::Stamped<cv::Mat> depth_rectified;
    Kinect2Interface::Stamped<cv::Mat> rgb_rectified;
    Kinect2Interface::Stamped<cv::Mat> ir_recitifed;
    Kinect2Interface::Stamped<cv::Mat> rgb_registered;

    Kinect2Interface::Data::Ptr data = kinterface_.getData();
    if(data) {
        /// CAMERA INFO UPDATE
        updateCameraInfo();
        /// PUBLISH RAW RGB IMAGE
        if(publish_rgb_) {
            convertRGB(data->rgb, frame_id_rgb_, image_rgb_);
            image_rgb_->header.stamp += time_offset_rgb_;
            pub_rgb_.publish(image_rgb_);
            pub_rgb_info_.publish(camera_info_rgb_);
        }
        /// PUBLISH RAW DEPTH IMAGE
        if(publish_depth_) {
            convertFloat(data->depth, frame_id_ir_, image_depth_);
            image_depth_->header.stamp +=  time_offset_ir_;
            pub_depth_.publish(image_depth_);
            pub_depth_info_.publish(camera_info_ir_);
        }
        /// PUBLISH RAW IR IMAGE
        if(publish_ir_) {
            convertFloat(data->ir, frame_id_ir_, image_ir_);
            image_ir_->header.stamp +=  time_offset_ir_;
            pub_ir_.publish(image_ir_);
            pub_ir_info_.publish(camera_info_ir_);

        }

        /// DO THE RECTIFICATEION
        depth_rectified = clone(data->depth);
        rgb_rectified   = clone(data->rgb);

        cv::remap(depth_rectified.data, depth_rectified.data,
                  ir_rectification_map_x_, ir_rectification_map_y_,
                  CV_INTER_NN);
        cv::remap(rgb_rectified.data, rgb_rectified.data,
                  rgb_rectification_map_x_, rgb_rectification_map_y_,
                  CV_INTER_NN);


        /// RECTIFIED RGB IMAGE
        if(publish_rgb_rectified_) {
            convertRGB(rgb_rectified, frame_id_ir_, image_depth_rectified_);
            image_depth_rectified_->header.stamp +=  time_offset_rgb_;
            pub_rgb_registered_.publish(image_rgb_rectified_);
        }
        /// RECTIFIED DEPTH IMAGE
        if(publish_depth_rectified_) {
            convertFloat(depth_rectified, frame_id_ir_, image_depth_rectified_);
            image_depth_rectified_->header.stamp +=  time_offset_ir_;
            pub_depth_rectified_.publish(image_depth_rectified_);
        }
        /// RECTIFIED IR IMAGE
        if(publish_ir_rectified_) {
            ir_recitifed = clone(data->ir);

            cv::remap(ir_recitifed.data, ir_recitifed.data,
                      ir_rectification_map_x_, ir_rectification_map_y_,
                      CV_INTER_LINEAR);

            convertFloat(ir_recitifed, frame_id_ir_, image_ir_rectified_);
            image_ir_rectified_->header.stamp +=  time_offset_ir_;
            pub_depth_rectified_.publish(image_ir_rectified_);
        }

        /// REGISTER THE DATA AND PRODUCE AND REGISTERED IMAGE AND A POINT CLOUD
        rgb_registered.data = cv::Mat(depth_rectified.data.rows, depth_rectified.data.cols,
                                      CV_8UC4,
                                      cv::Scalar());
        const float rgb_fx = rgb_optimal_camera_matrix_.at<double>(0,0);
        const float rgb_fy = rgb_optimal_camera_matrix_.at<double>(1,1);
        const float rgb_cx = rgb_optimal_camera_matrix_.at<double>(0,2);
        const float rgb_cy = rgb_optimal_camera_matrix_.at<double>(1,2);

        const cv::Vec4b *rgb_rectified_ptr = rgb_rectified.data.ptr<cv::Vec4b>();
        cv::Vec4b       *rgb_registered_ptr = rgb_registered.data.ptr<cv::Vec4b>();
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr points(new pcl::PointCloud<pcl::PointXYZRGB>);
        points->height  = kinterface_camera_parameters_->height_ir;
        points->width   = kinterface_camera_parameters_->width_ir;
        points->is_dense = true;
        points->points.resize(points->height *
                              points->width);
        pcl::PointXYZRGB *points_ptr = points->points.data();
        for(int i = 0 ; i < depth_rectified.data.rows ; ++i) {
            for(int j = 0 ; j < depth_rectified.data.cols ; ++j) {
                const float depth_d = data->depth.data.at<float>(i,j) * 1e-3f;
                const float depth_r = depth_rectified.data.at<float>(i,j) * 1e-3f;

                if(std::isnormal(depth_r)) {
                    Eigen::Vector4f p (depth_lookup_rectified_x_.at<float>(i,j) * depth_r,
                                       depth_lookup_rectified_y_.at<float>(i,j) * depth_r,
                                       depth_r,
                                       1.f);
                    Eigen::Vector4f p_ = kcalibration_.extr_transformation * p;

                    /// exchange through eigen
                    double depth_ = 1.0 / p_(2);
                    const int rgb_x = std::floor((p_(0) * rgb_fx) * depth_ + rgb_cx + 0.5f);
                    const int rgb_y = std::floor((p_(1) * rgb_fy) * depth_ + rgb_cy + 0.5f);

                    if(rgb_x >= 0 &&
                            rgb_y >= 0 &&
                            rgb_x < rgb_rectified.data.cols &&
                            rgb_y < rgb_rectified.data.rows) {
                        const auto &rgb = rgb_rectified_ptr[rgb_rectified.data.cols * rgb_y + rgb_x];
                        *rgb_registered_ptr = rgb;
                        pcl::PointXYZRGB &prgb = points_ptr[i * points->width + j];
                        prgb.x = p(0);
                        prgb.y = p(1);
                        prgb.z = p(2);
                        prgb.r = rgb[2];
                        prgb.g = rgb[1];
                        prgb.b = rgb[1];
                    }
                    ++rgb_registered_ptr;
                }
            }
        }

        cv::normalize(depth_rectified.data, depth_rectified.data, 0.0, 1.0, cv::NORM_MINMAX);
        cv::imshow("debug",  depth_rectified.data);
        cv::waitKey(19);

        /// REGISTERED RGB IMAGE
        if(publish_rgb_registered_) {
            /// register the shit out of it
            convertRGB(rgb_registered, frame_id_ir_, image_rgb_registered_);
            image_rgb_registered_->header.stamp +=  time_offset_rgb_;
            pub_rgb_registered_.publish(image_rgb_registered_);
        }
        points->header.frame_id = frame_id_ir_;
        points->header.stamp += time_offset_ir_.toNSec() / 1000;
        pub_pointcloud_.publish(points);

    }
}

bool Kinect2Node::sleep(std_srvs::Empty::Request &req,
                        std_srvs::Empty::Response &res)
{
    return kinterface_.stop();
}

bool Kinect2Node::wakeup(std_srvs::Empty::Request &req,
                         std_srvs::Empty::Response &res)
{
    return kinterface_.start();
}

void Kinect2Node::updateCameraInfo()
{
    if(!kinterface_camera_parameters_) {
        kinterface_camera_parameters_.reset(new CameraParameters);
        kinterface_.getCameraParameters(*kinterface_camera_parameters_);

        ir_optimal_camera_matrix_ = cv::getOptimalNewCameraMatrix(kcalibration_.ir_camera_matrix,
                                                                  kcalibration_.ir_distortion_coefficients,
                                                                  cv::Size(kinterface_camera_parameters_->width_ir,
                                                                           kinterface_camera_parameters_->height_ir),
                                                                  0);

        rgb_optimal_camera_matrix_ = cv::getOptimalNewCameraMatrix(kcalibration_.rgb_camera_matrix,
                                                                   kcalibration_.rgb_distortion_coefficients,
                                                                   cv::Size(kinterface_camera_parameters_->width_rgb,
                                                                            kinterface_camera_parameters_->height_rgb),
                                                                   0);
        cv::initUndistortRectifyMap(kcalibration_.ir_camera_matrix,
                                    kcalibration_.ir_distortion_coefficients,
                                    cv::Mat(),
                                    ir_optimal_camera_matrix_,
                                    cv::Size(kinterface_camera_parameters_->width_ir,
                                             kinterface_camera_parameters_->height_ir),
                                    CV_32FC1,
                                    ir_rectification_map_x_, ir_rectification_map_y_);

        cv::initUndistortRectifyMap(kcalibration_.rgb_camera_matrix,
                                    kcalibration_.rgb_distortion_coefficients,
                                    cv::Mat(),
                                    rgb_optimal_camera_matrix_,
                                    cv::Size(kinterface_camera_parameters_->width_rgb,
                                             kinterface_camera_parameters_->height_rgb),
                                    CV_32FC1,
                                    rgb_rectification_map_x_, rgb_rectification_map_y_);

        depth_lookup_rectified_x_ = cv::Mat(kinterface_camera_parameters_->height_ir,
                                            kinterface_camera_parameters_->width_ir,
                                            CV_32FC1,
                                            cv::Scalar());
        depth_lookup_rectified_y_ = cv::Mat(kinterface_camera_parameters_->height_ir,
                                            kinterface_camera_parameters_->width_ir,
                                            CV_32FC1,
                                            cv::Scalar());

        float *depth_lookup_rectified_x_ptr = depth_lookup_rectified_x_.ptr<float>();
        float *depth_lookup_rectified_y_ptr = depth_lookup_rectified_y_.ptr<float>();
        const double fx = 1.0 / ir_optimal_camera_matrix_.at<double>(0,0);
        const double fy = 1.0 / ir_optimal_camera_matrix_.at<double>(1,1);
        const double cx = ir_optimal_camera_matrix_.at<double>(0,2);
        const double cy = ir_optimal_camera_matrix_.at<double>(1,2);

        for(int i = 0 ; i < depth_lookup_rectified_x_.rows ; ++i) {
            for(int j = 0 ; j < depth_lookup_rectified_x_.cols ; ++j) {
                const float x = ir_rectification_map_x_.at<float>(i,j);
                const float y = ir_rectification_map_y_.at<float>(i,j);


                *depth_lookup_rectified_x_ptr = (j - cx) * fx;
                *depth_lookup_rectified_y_ptr = (i - cy) * fy;
                ++depth_lookup_rectified_x_ptr;
                ++depth_lookup_rectified_y_ptr;
            }
        }
    }

    if(!camera_info_ir_) {
        camera_info_ir_.reset(new sensor_msgs::CameraInfo);

        camera_info_ir_->header.frame_id = frame_id_ir_;

        camera_info_ir_->width  = kinterface_camera_parameters_->width_ir;
        camera_info_ir_->height = kinterface_camera_parameters_->height_ir;

        camera_info_ir_->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

        kcalibration_.getDistortionCoefficientsIR(camera_info_ir_->D);
        kcalibration_.getCameraMatrixIR(camera_info_ir_->K);
        kcalibration_.getRectificationMatrixIR(camera_info_ir_->R);
        kcalibration_.getProjectionMatrixIR(camera_info_ir_->P);
    }

    if(!camera_info_rgb_) {
        camera_info_rgb_.reset(new sensor_msgs::CameraInfo);

        camera_info_rgb_->header.frame_id  = frame_id_rgb_;
        camera_info_rgb_->width            = kinterface_camera_parameters_->width_rgb;
        camera_info_rgb_->height           = kinterface_camera_parameters_->height_rgb;
        camera_info_rgb_->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

        kcalibration_.getDistortionCoefficientsRGB(camera_info_rgb_->D);
        kcalibration_.getCameraMatrixRGB(camera_info_rgb_->K);
        kcalibration_.getRectificationMatrixRGB(camera_info_rgb_->R);
        kcalibration_.getProjectionMatrixRGB(camera_info_rgb_->P);

    }

    ros::Time stamp = ros::Time::now();
    camera_info_rgb_->header.stamp = stamp;
    camera_info_ir_->header.stamp  = stamp;

}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "kinect2_node");

    Kinect2Node kn;
    kn.setup();

    return kn.run();
}
