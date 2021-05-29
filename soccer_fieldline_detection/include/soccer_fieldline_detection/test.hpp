#pragma once


#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <opencv2/core/cvdef.h>

class test_node {

    image_transport::Subscriber image_subscriber;
    image_transport::Publisher image_publisher;
    image_transport::Publisher image_publisher2;
    image_transport::Publisher image_publisher3;
    image_transport::Publisher image_publisher4;
    ros::Publisher point_cloud_publisher;

    int cannythreshold1 = 200;
    int cannythreshold2 = 400;

    double rho = 1;
    double theta = CV_PI / 180;
    int threshold = 50;
    int minLineLength = 50;
    int maxLineGap = 50;


public:
    ros::NodeHandle nh;

    test_node();

    void imageCallback();
};