#ifndef KINECT2_NODE_H
#define KINECT2_NODE_H


/// SYSTEM
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>


/// PROJECT
#include "kinect2_interface.h"

class Kinect2Node
{
public:
    Kinect2Node();
    virtual ~Kinect2Node();

    bool setup();

    int run();

    ros::NodeHandle              nh_private_;
    ros::NodeHandle              nh_;

    double                       pub_rate_preferred_;

    ros::Publisher               pub_rgb_;
    ros::Publisher               pub_rgb_info_;
    ros::Publisher               pub_depth_;
    ros::Publisher               pub_ir_;
    ros::Publisher               pub_ir_info_;
    ros::Publisher               pub_rgb_registered_;
    ros::Publisher               pub_depth_undistorted_;
    ros::Publisher               pub_pointcloud_;

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





private:
    void publish();

};

#endif // KINECT2_NODE_H
