#ifndef KINECT2_NODE_H
#define KINECT2_NODE_H


/// SYSTEM
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <ros/service_server.h>
#include <std_srvs/Empty.h>

/// PROJECT
#include "kinect2_interface.h"

/**
 * @brief The Kinect2Node class
 */
class Kinect2Node
{
public:
    Kinect2Node();
    virtual ~Kinect2Node();

    bool setup();
    int run();

private:
    ros::NodeHandle              nh_private_;
    ros::NodeHandle              nh_;

    double                       pub_rate_preferred_;

    ros::Publisher               pub_rgb_;
    ros::Publisher               pub_rgb_info_;
    ros::Publisher               pub_depth_;
    ros::Publisher               pub_depth_info_;
    ros::Publisher               pub_ir_;
    ros::Publisher               pub_ir_info_;
    ros::Publisher               pub_rgb_registered_;
    ros::Publisher               pub_depth_undistorted_;
    ros::Publisher               pub_pointcloud_;

    ros::ServiceServer           service_sleep_;
    ros::ServiceServer           service_wakeup_;

    std::string                  frame_id_rgb_;
    std::string                  frame_id_ir_;
    std::string                  frame_id_;

    Kinect2Interface::CameraParameters::Ptr kinterface_camera_paramters_;
    Kinect2Interface::Parameters            kinterface_parameters_;
    Kinect2Interface                        kinterface_;


    sensor_msgs::CameraInfo::Ptr camera_info_rgb_;
    sensor_msgs::CameraInfo::Ptr camera_info_ir_;

    sensor_msgs::Image::Ptr      image_ir_;
    sensor_msgs::Image::Ptr      image_depth_;
    sensor_msgs::Image::Ptr      image_depth_rectified_;
    sensor_msgs::Image::Ptr      image_rgb_;
    sensor_msgs::Image::Ptr      image_rgb_registered_;


    void publish();
    bool sleep(std_srvs::Empty::Request  &req,
               std_srvs::Empty::Response &res);
    bool wakeup(std_srvs::Empty::Request  &req,
                std_srvs::Empty::Response &res);

};

#endif // KINECT2_NODE_H
