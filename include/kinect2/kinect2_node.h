#ifndef KINECT2_NODE_H
#define KINECT2_NODE_H


/// SYSTEM
#include <ros/ros.h>
#include <ros/service_server.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <std_srvs/Empty.h>

/// PROJECT
#include <kinect2/Kinect2Info.h>
#include <kinect2/kinect2_interface.h>
#include <kinect2/kinect2_calibration.hpp>

/**
 * @brief The Kinect2Node class wraps the kinect2 interface class and provides topics
 *        and service to either retrieve data or stop / start the driver.
 */
namespace kinect2 {
class Kinect2Node
{
public:
    /**
     * @brief Kinect2Node constructor.
     */
    Kinect2Node();
    /**
     * @brief ~Kinect2Node destructor.
     */
    virtual ~Kinect2Node();

    /**
     * @brief setup reads the parameters and prepares the kinect2 interface.
     * @return sucess of initialization
     */
    bool setup();
    /**
     * @brief run the ros loop and return ros integer code on shutdown.
     * @return  shutdown code
     */
    int run();

private:
    /// ros
    ros::NodeHandle              nh_private_;
    ros::NodeHandle              nh_;

    double                       pub_rate_preferred_;

    ros::Publisher               pub_rgb_;
    ros::Publisher               pub_rgb_info_;
    ros::Publisher               pub_rgb_rectified_;
    ros::Publisher               pub_rgb_registered_;
    ros::Publisher               pub_depth_;
    ros::Publisher               pub_depth_info_;
    ros::Publisher               pub_depth_rectified_;
    ros::Publisher               pub_ir_;
    ros::Publisher               pub_ir_info_;
    ros::Publisher               pub_ir_rectified_;
    ros::Publisher               pub_pointcloud_;

    bool                         publish_rgb_;
    bool                         publish_rgb_rectified_;
    bool                         publish_rgb_registered_;
    bool                         publish_ir_;
    bool                         publish_ir_rectified_;
    bool                         publish_depth_;
    bool                         publish_depth_rectified_;


    //// stop/start services
    ros::ServiceServer           service_sleep_;
    ros::ServiceServer           service_wakeup_;

    //// tf
    std::string                  frame_id_rgb_;
    std::string                  frame_id_ir_;

    //// kinect2 interface
    CameraParameters::Ptr        kinterface_camera_parameters_;
    Kinect2Interface::Parameters kinterface_parameters_;
    Kinect2Interface             kinterface_;
    Kinect2Calibration           kcalibration_;


    cv::Mat                      depth_lookup_rectified_x_;
    cv::Mat                      depth_lookup_rectified_y_;

    cv::Mat                      ir_rectification_map_x_;
    cv::Mat                      ir_rectification_map_y_;
    cv::Mat                      ir_optimal_camera_matrix_;

    cv::Mat                      rgb_rectification_map_x_;
    cv::Mat                      rgb_rectification_map_y_;
    cv::Mat                      rgb_optimal_camera_matrix_;

    //// ros message buffers for publication
    sensor_msgs::CameraInfo::Ptr camera_info_rgb_;
    sensor_msgs::CameraInfo::Ptr camera_info_ir_;
    kinect2::Kinect2Info::Ptr    kinect2_info_;

    sensor_msgs::Image::Ptr      image_ir_;
    sensor_msgs::Image::Ptr      image_ir_rectified_;
    sensor_msgs::Image::Ptr      image_depth_;
    sensor_msgs::Image::Ptr      image_depth_rectified_;
    sensor_msgs::Image::Ptr      image_rgb_;
    sensor_msgs::Image::Ptr      image_rgb_registered_;
    sensor_msgs::Image::Ptr      image_rgb_rectified_;

    //// time offsets
    ros::Duration                time_offset_ir_;
    ros::Duration                time_offset_rgb_;

    /**
     * @brief publish triggers the publication of gathered 3D and image data.
     */
    void publish();
    /**
     * @brief updateCameraInfo updates the camera information messages which afterwards can
     *        be published.
     */
    void updateCameraInfo();
    /**
     * @brief sleep closes the connection to the kinect device and disables the
     *        publication of data. The ros loop of this node is slowed down to 1Hz.
     * @param req   - empty
     * @param res   - empty
     * @return  success of putting the kinect driver to sleep
     */
    bool sleep(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res);
    /**
     * @brief wakeup reconnects the kinect interface to the sensor and enables
     *        data publication. The ros loop is speed up again to full speed.
     * @param req   - empty
     * @param res   - empty
     * @return  sccess of  waking up the device and reconnecting
     */
    bool wakeup(std_srvs::Empty::Request  &req, std_srvs::Empty::Response &res);

};
}
#endif // KINECT2_NODE_H
