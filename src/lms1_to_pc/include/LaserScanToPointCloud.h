#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>

#include <transformation/VehiclePose.h>
#include "laser_geometry.h"


class LaserScanToPointCloud
{
public:
    ros::NodeHandle m_node;
    laser_geometry::LaserProjection m_projector1;
    tf::TransformListener m_transformListener;
    message_filters::Subscriber<sensor_msgs::LaserScan> m_msgFilterSubscriberLms1;
    tf::MessageFilter<sensor_msgs::LaserScan> m_tfMsgFilter1;
    ros::Publisher m_pubPointCloud1;
    ros::Subscriber m_subVehiclePose;
    transformation::VehiclePose m_VehiclePose;
    //constructor
    LaserScanToPointCloud(ros::NodeHandle n);

    void ScanCallback1(const sensor_msgs::LaserScanConstPtr &scan_in);
    void VehiclePoseCallback(transformation::VehiclePose pose);
};
