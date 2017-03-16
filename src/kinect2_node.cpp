/// HEADER
#include <kinect2/kinect2_node.h>

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
    const std::string topic_color             = nh_private_.param<std::string>("topic_color",               "/kinect2/image_color");
    const std::string topic_color_info        = nh_private_.param<std::string>("topic_color_info",          "/kinect2/image_color/camera_info");
    const std::string topic_depth             = nh_private_.param<std::string>("topic_depth",               "/kinect2/depth");
    const std::string topic_depth_info        = nh_private_.param<std::string>("topic_depth_info",          "/kinect2/depth/camera_info");
    const std::string topic_ir                = nh_private_.param<std::string>("topic_ir",                  "/kinect2/image_ir");
    const std::string topic_ir_info           = nh_private_.param<std::string>("topic_ir_info",             "/kinect2/image_ir/camera_info");
    const std::string topic_kinect2_info      = nh_private_.param<std::string>("topic_kinect2_info",        "/kinect2/camera_info");
    const std::string topic_color_registered  = nh_private_.param<std::string>("topic_colo_registered",     "/kinect2/color_registered");
    const std::string topic_depth_rectified   = nh_private_.param<std::string>("topic_depth_undistorted",   "/kinect2/depth_rectified");
    const std::string topic_pointcloud        = nh_private_.param<std::string>("topic_rgb",                 "/kinect2/points");
    const std::string service_name_wakeup     = nh_private_.param<std::string>("service_name_wakeup",       "/kinect2/wakeup");
    const std::string service_name_sleep      = nh_private_.param<std::string>("service_name_sleep",        "/kinect2/sleep");

    pub_rate_preferred_                       = nh_private_.param<double>("preferred_publication_rate",     -1.0);
    frame_id_rgb_                             = nh_private_.param<std::string>("frame_id_color",            "kinect2_color_optical_frame");
    frame_id_ir_                              = nh_private_.param<std::string>("frame_id_ir",               "kinect2_depth_optical_frame");

    kinterface_parameters_.get_color                = nh_private_.param<bool>("publish_color", false);
    kinterface_parameters_.get_ir                   = nh_private_.param<bool>("publish_ir", false);
    kinterface_parameters_.get_depth                 = nh_private_.param<bool>("publish_depth", false);

    kinterface_parameters_.get_color_registered     = nh_private_.param<bool>("publish_color_registered", false);
    kinterface_parameters_.get_depth_rectified      = nh_private_.param<bool>("publish_depth_rectified", false);

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

    if(kinterface_parameters_.get_color) {
        pub_rgb_ = nh_.advertise<sensor_msgs::Image>(topic_color, 1);
        pub_rgb_info_ = nh_.advertise<sensor_msgs::CameraInfo>(topic_color_info, 1);
    }
    if(kinterface_parameters_.get_depth) {
        pub_depth_ = nh_.advertise<sensor_msgs::Image>(topic_depth, 1);
        pub_depth_info_ = nh_.advertise<sensor_msgs::CameraInfo>(topic_depth_info, 1);
    }
    if(kinterface_parameters_.get_ir) {
        pub_ir_ = nh_.advertise<sensor_msgs::Image>(topic_ir, 1);
        pub_ir_info_ = nh_.advertise<sensor_msgs::CameraInfo>(topic_ir_info, 1);
    }
    if(kinterface_parameters_.get_color_registered) {
        pub_rgb_registered_ = nh_.advertise<sensor_msgs::Image>(topic_color_registered, 1);
    }
    if(kinterface_parameters_.get_depth_rectified) {
        pub_depth_undistorted_ = nh_.advertise<sensor_msgs::Image>(topic_depth_rectified, 1);
    }

    pub_pointcloud_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB>>(topic_pointcloud, 1);
    pub_kinect2_info_ = nh_.advertise<Kinect2Info>(topic_kinect2_info, 1);
    service_sleep_  = nh_.advertiseService(service_name_sleep, &Kinect2Node::sleep, this);
    service_wakeup_ = nh_.advertiseService(service_name_wakeup, &Kinect2Node::wakeup, this);

    return kinterface_.start();
}

int Kinect2Node::run()
{
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

        image->header.stamp = ros::Time(rgb.stamp * 1e-4);

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

        image->header.stamp = ros::Time(mat.stamp * 1e-4);

        const float * mat_ptr = mat.data.ptr<float>();
        ushort * image_ptr = (ushort*) image->data.data();
        for(std::size_t i = 0 ; i < rows ; ++i) {
            for(std::size_t j = 0 ; j < cols ; ++j) {
                image_ptr[i * cols + j] =
                        mat_ptr[i * cols + (cols - j - 1)];
            }
        }
    };


    Kinect2Interface::Data::Ptr data = kinterface_.getData();
    if(data) {
        updateCameraInfo();

        if(kinterface_parameters_.get_color) {
            convertRGB(data->rgb, frame_id_rgb_, image_rgb_);
            pub_rgb_.publish(image_rgb_);
            pub_rgb_info_.publish(camera_info_rgb_);
        }
        if(kinterface_parameters_.get_ir) {
            convertFloat(data->ir, frame_id_ir_, image_ir_);
            pub_ir_.publish(image_ir_);
            pub_ir_info_.publish(camera_info_ir_);

        }
        if(kinterface_parameters_.get_depth) {
            convertFloat(data->depth, frame_id_ir_, image_depth_);
            pub_depth_.publish(image_depth_);
            pub_depth_info_.publish(camera_info_ir_);
        }
        if(kinterface_parameters_.get_depth_rectified) {
            convertFloat(data->depth_rectified, frame_id_ir_, image_depth_rectified_);
            pub_depth_undistorted_.publish(image_depth_rectified_);
        }
        if(kinterface_parameters_.get_color_registered) {
            convertRGB(data->rgb_registered, frame_id_ir_, image_rgb_registered_);
            pub_rgb_registered_.publish(image_rgb_registered_);
        }
        if(!data->points->points.empty()) {
            data->points->header.frame_id = frame_id_ir_;
            pub_pointcloud_.publish(data->points);
        }
        pub_kinect2_info_.publish(kinect2_info_);
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
    }

    if(!camera_info_ir_) {
        camera_info_ir_.reset(new sensor_msgs::CameraInfo);

        camera_info_ir_->header.frame_id = frame_id_ir_;

        camera_info_ir_->width  = kinterface_camera_parameters_->width_ir;
        camera_info_ir_->height = kinterface_camera_parameters_->height_ir;

        camera_info_ir_->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

        kinterface_camera_parameters_->getDistortionCoefficientsIR(camera_info_ir_->D);
        kinterface_camera_parameters_->getCameraMatrixIR(camera_info_ir_->K);
        kinterface_camera_parameters_->getRectificationMatrixIR(camera_info_ir_->R);
        kinterface_camera_parameters_->getProjectionMatrixIR(camera_info_ir_->P);
    }

    if(!camera_info_rgb_) {
        camera_info_rgb_.reset(new sensor_msgs::CameraInfo);

        camera_info_rgb_->header.frame_id  = frame_id_rgb_;
        camera_info_rgb_->width            = kinterface_camera_parameters_->width_rgb;
        camera_info_rgb_->height           = kinterface_camera_parameters_->height_rgb;
        camera_info_rgb_->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

        kinterface_camera_parameters_->getDistortionCoefficientsRGB(camera_info_rgb_->D);
        kinterface_camera_parameters_->getCameraMatrixRGB(camera_info_rgb_->K);
        kinterface_camera_parameters_->getRectificationMatrixRGB(camera_info_rgb_->R);
        kinterface_camera_parameters_->getProjectionMatrixRGB(camera_info_rgb_->P);

    }
    if(!kinect2_info_ && camera_info_rgb_ && camera_info_ir_) {
        kinect2_info_.reset(new Kinect2Info);
        kinect2_info_->header = camera_info_ir_->header;

        /// rgb
        kinect2_info_->distortion_model_rgb = camera_info_rgb_->distortion_model;
        kinect2_info_->height_rgb           = camera_info_rgb_->height;
        kinect2_info_->width_rgb            = camera_info_rgb_->width;
        kinect2_info_->D_rgb                = camera_info_rgb_->D;
        kinect2_info_->K_rgb                = camera_info_rgb_->K;
        kinect2_info_->R_rgb                = camera_info_rgb_->R;
        kinect2_info_->P_rgb                = camera_info_rgb_->P;
        kinterface_camera_parameters_->getMappingCoefficientsRGB(kinect2_info_->M_x_rgb,
                                                                 kinect2_info_->M_y_rgb);

        kinect2_info_->shift_d_rgb = kinterface_camera_parameters_->color.shift_d;
        kinect2_info_->shift_m_rgb = kinterface_camera_parameters_->color.shift_m;

        /// ir
        kinect2_info_->distortion_model_ir = camera_info_ir_->distortion_model;
        kinect2_info_->height_ir           = camera_info_ir_->height;
        kinect2_info_->width_ir            = camera_info_ir_->width;
        kinect2_info_->D_ir                = camera_info_ir_->D;
        kinect2_info_->K_ir                = camera_info_ir_->K;
        kinect2_info_->R_ir                = camera_info_ir_->R;
        kinect2_info_->P_ir                = camera_info_ir_->P;
    }

    ros::Time stamp = ros::Time::now();
    camera_info_rgb_->header.stamp = stamp;
    camera_info_ir_->header.stamp  = stamp;
    kinect2_info_->header.stamp    = stamp;
}



int main(int argc, char *argv[])
{
    ros::init(argc, argv, "kinect2_node");

    Kinect2Node kn;
    kn.setup();

    return kn.run();
}
