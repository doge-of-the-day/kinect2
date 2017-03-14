/// HEADER
#include "kinect2_node.h"

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
    const std::string topic_rgb               = nh_private_.param<std::string>("topic_rgb", "/kinect2_node/rgb");
    const std::string topic_depth             = nh_private_.param<std::string>("topic_depth", "/kinect2_node/depth");
    const std::string topic_ir                = nh_private_.param<std::string>("topic_ir", "/kinect2_node/ir");
    const std::string topic_rgb_registered    = nh_private_.param<std::string>("topic_rgb_registered", "/kinect2_node/rgb_registered");
    const std::string topic_depth_unidstorted = nh_private_.param<std::string>("topic_depth_undistorted", "/kinect2_node/depth_undistorted");
    const std::string topic_pointcloud        = nh_private_.param<std::string>("topic_rgb", "/kinect2_node/point");
    pub_rate_preferred_                       = nh_private_.param<double>("preferred_publication_rate", -1.0);
    frame_id_rgb_                             = nh_private_.param<std::string>("frame_id_rgb", "kinect2_rgb_optical_frame");
    frame_id_depth_                           = nh_private_.param<std::string>("frame_id_depth", "kinect_depth_optical_frame");


    bool publish_rgb               = nh_private_.param<bool>("publish_rgb", false);
    bool publish_ir                = nh_private_.param<bool>("publish_ir", true);
    bool publish_rgb_registered    = nh_private_.param<bool>("publish_rgb_registered", false);
    bool publish_depth             = nh_private_.param<bool>("publish_depth", false);
    bool publish_depth_undistorted = nh_private_.param<bool>("publish_depth_undistorted", false);

    Kinect2Interface::Parameters parameters;
    parameters.get_rgb = publish_rgb;
    parameters.get_ir = publish_ir;
    parameters.get_depth = publish_depth;
    parameters.get_rgb_registered = publish_rgb_registered;
    parameters.get_depth = publish_depth;
    parameters.get_depth_undistorted = publish_depth_undistorted;

    if(!kinterface_.setup(parameters)) {
        std::cerr << "[Kinect2Node]: Cannot setup Kinect2Interface!" << std::endl;
        ros::shutdown();
    }

    if(publish_rgb) {
        pub_rgb_ = nh_.advertise<sensor_msgs::Image>(topic_rgb, 1);
    }
    if(publish_depth) {
        publish_depth = nh_.advertise<sensor_msgs::Image>(topic_depth, 1);
    }
    if(publish_ir) {
        pub_ir_ = nh_.advertise<sensor_msgs::Image>(topic_ir, 1);
    }
    if(publish_rgb_registered) {
        pub_rgb_registered_ = nh_.advertise<sensor_msgs::Image>(topic_rgb_registered, 1);
    }
    if(publish_depth_undistorted) {
        pub_depth_undistorted_ = nh_.advertise<sensor_msgs::Image>(topic_depth_unidstorted, 1);
    }
    if(publish_depth) {
        pub_depth_ = nh_.advertise<sensor_msgs::Image>(topic_depth, 1);
    }

    pub_pointcloud_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB>>(topic_pointcloud, 1);

    kinterface_.start();
}

int Kinect2Node::run()
{
    if(pub_rate_preferred_ <= 0.0) {
        while(ros::ok()) {
            publish();
            ros::spinOnce();
        }
    } else {
        ros::Rate rate(pub_rate_preferred_);
        while(ros::ok()) {
            publish();
            rate.sleep();
        }
    }
    kinterface_.stop();
    ros::shutdown();
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
            image->encoding     = sensor_msgs::image_encodings::RGB8;
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
        if(!data->rgb.data.empty()) {
            convertRGB(data->rgb, frame_id_rgb_, image_rgb_);
            pub_rgb_.publish(image_rgb_);
        }
        if(!data->ir.data.empty()) {
            convertFloat(data->ir, frame_id_depth_, image_ir_);
            pub_ir_.publish(image_ir_);
        }
        if(!data->depth.data.empty()) {
            convertFloat(data->depth, frame_id_depth_, image_depth_);
            pub_depth_.publish(image_depth_);
        }
        if(!data->depth_undistorted.data.empty()) {
            convertFloat(data->depth_undistorted, frame_id_depth_, image_depth_undistorted_);
            pub_depth_undistorted_.publish(image_depth_undistorted_);
        }
        if(!data->rgb_registered.data.empty()) {
            convertRGB(data->rgb_registered, frame_id_depth_, image_rgb_registered_);
            pub_rgb_registered_.publish(image_rgb_registered_);
        }
        if(!data->points->points.empty()) {
            data->points->header.frame_id = frame_id_depth_;
            pub_pointcloud_.publish(data->points);
        }
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "kinect2_node");

    Kinect2Node kn;
    kn.setup();

    return kn.run();
}
