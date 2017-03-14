#ifndef KINECT2_NODE_H
#define KINECT2_NODE_H


#include <ros/ros.h>

#include "kinect2_interface.h"

class Kinect2Node
{
public:
    Kinect2Node();
    virtual ~Kinect2Node();

    bool setup();

    int run();

    ros::NodeHandle nh_private_;
    ros::NodeHandle nh_;

    double         pub_rate_preferred_;
    double         maximum_depth_;

    ros::Publisher pub_rgb_;
    ros::Publisher pub_depth_;
    ros::Publisher pub_ir_;
    ros::Publisher pub_rgb_registered_;
    ros::Publisher pub_depth_undistorted_;
    ros::Publisher pub_pointcloud_;

    std::string    frame_id_rgb_;
    std::string    frame_id_depth_;

    Kinect2Interface kinterface_;

private:
    void publish();

};

#endif // KINECT2_NODE_H
