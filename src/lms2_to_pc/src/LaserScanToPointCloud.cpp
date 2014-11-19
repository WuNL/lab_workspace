#include "LaserScanToPointCloud.h"
#include <boost/thread/mutex.hpp>

boost::mutex mtx;

LaserScanToPointCloud::LaserScanToPointCloud(ros::NodeHandle node):
    m_node(node),
    m_msgFilterSubscriberLms1(node, "/lms2Scan", 20),
    m_tfMsgFilter1(m_msgFilterSubscriberLms1, m_transformListener, "/world", 20)

{
    m_tfMsgFilter1.registerCallback(boost::bind(&LaserScanToPointCloud::ScanCallback1, this, _1));
    m_tfMsgFilter1.setTolerance(ros::Duration(0.01));
    m_pubPointCloud1 = m_node.advertise<sensor_msgs::PointCloud>("/lms2PC", 20);
    m_subVehiclePose = m_node.subscribe("vehiclePose", 20, &LaserScanToPointCloud::VehiclePoseCallback, this);
}

void LaserScanToPointCloud::ScanCallback1(const sensor_msgs::LaserScanConstPtr &scan_in)
{
    boost::mutex::scoped_lock lock(mtx);
    if(m_VehiclePose.accelerationZ<-10.0 || m_VehiclePose.accelerationZ>-9.65)
    {
        return;
    }
    sensor_msgs::PointCloud cloud;
    m_projector1.transformLaserScanToPointCloud("/world", *scan_in, cloud, m_transformListener, -1.0, laser_geometry::channel_option::Distance | laser_geometry::channel_option::Index | laser_geometry::channel_option::Intensity | laser_geometry::channel_option::Timestamp);
    m_pubPointCloud1.publish(cloud);
}

void LaserScanToPointCloud::VehiclePoseCallback(transformation::VehiclePose pose)
{
    boost::mutex::scoped_lock lock(mtx);
    m_VehiclePose = pose;
}
