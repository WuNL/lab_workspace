#include <ros/ros.h>
#define TF_EULER_DEFAULT_ZYX
#include <tf/transform_broadcaster.h>
#include <tf/LinearMath/Quaternion.h>
#include <iostream>
#include <ros/console.h>
#include <geometry_msgs/PolygonStamped.h>
#include "NCOM.h"
#include "VehiclePose.h"

transformation::VehiclePose vehiclePose;
geometry_msgs::PolygonStamped footPrint;
ros::Subscriber* g_pNCOMSubscriber;
ros::Publisher* g_pPosePublisher;
ros::Publisher* g_pFootPrintPublisher;

std::string strNodeName = "Transformation";
std::string strNCOMTopicName = "inertial";
std::string strPubVehiclePoseTopicName ="vehiclePose";
std::string strPubFootPrintTopicName = "footPrint";
std::string strVehicleFrameID = "vehicle";
std::string strWorldFrameID = "world";
struct CvehicleStatus
{
    float fX;
    float fY;
    float fZ;
    float fYaw;
    float fPitch;
    float fRoll;
    float fVx;
    float fVy;
    float fVz;
    float fVelocity;
}vehicleStatus;


tfScalar Deg2Rad(float deg)
{
    return tfScalar(deg*3.1415926/180.0);
}

void PublishTransform(ros::Time stamp, float fX, float fY, float fZ, float fYaw, float fPitch, float fRoll)
{
    static tf::TransformBroadcaster tfBroadcaster;
    static tf::Transform transform; 

    //from world to vehile;
    transform.setOrigin(tf::Vector3(fX, fY, fZ));
    transform.setRotation(tf::Quaternion(fYaw, fPitch, fRoll));
    tfBroadcaster.sendTransform(tf::StampedTransform(transform, stamp, "/world", "/vehicle"));

    //from vehile to lms1;
    transform.setOrigin(tf::Vector3(1.26, 0.485, 2.196));
    //transform.setRotation(tf::Quaternion(0.0125+0.0026+0.0034, 0.183011, 0.0+0.0017*7));//roll, pitch, yaw
    transform.setRotation(tf::Quaternion(0.0, 0.183, 0.0));//roll, pitch, yaw
    tfBroadcaster.sendTransform(tf::StampedTransform(transform, stamp, "/vehicle", "/lms1"));

    //from vehicle to lms2;
    transform.setOrigin(tf::Vector3(1.26, -0.467, 2.208));
    //transform.setRotation(tf::Quaternion(0.0125003, 0.142386, 6.27694+0.0017*5));
    transform.setRotation(tf::Quaternion(0.0, 0.142386, 0.0));
    tfBroadcaster.sendTransform(tf::StampedTransform(transform, stamp, "/vehicle", "/lms2"));

    //from vehicle to velodyne1;
    transform.setOrigin(tf::Vector3(1.147, 0.477, 2.405));
    //transform.setRotation(tf::Quaternion(0.0, 0.0017, 0.0));  //yaw, pitch, roll
    transform.setRotation(tf::Quaternion(0.0, 0.0, 0.0));  //yaw, pitch, roll
    tfBroadcaster.sendTransform(tf::StampedTransform(transform, stamp, "/vehicle", "/velodyne1"));

    //from vehicle to velodyne2;
    transform.setOrigin(tf::Vector3(1.152, -0.445,2.45));
    //transform.setRotation(tf::Quaternion(6.28006,0.000175, 0.0));
    transform.setRotation(tf::Quaternion(0.0, 0.0, 0.0));
    tfBroadcaster.sendTransform(tf::StampedTransform(transform, stamp, "/vehicle", "/velodyne2"));

}

void PublishVehiclePose(const inertial::NCOMConstPtr& msg)
{

    static int poseCounter = 0;
    vehiclePose.header.frame_id = strWorldFrameID;
    vehiclePose.header.stamp = msg->header.stamp;
    vehiclePose.header.seq = poseCounter++;

    vehiclePose.heading = vehicleStatus.fYaw;
    vehiclePose.pitch = vehicleStatus.fPitch;
    vehiclePose.roll = vehicleStatus.fRoll;

    vehiclePose.velocityX = vehicleStatus.fVx;
    vehiclePose.velocityY = vehicleStatus.fVy;
    vehiclePose.velocityZ = vehicleStatus.fVz;

    vehiclePose.x = vehicleStatus.fX;
    vehiclePose.y = vehicleStatus.fY;
    vehiclePose.z = vehicleStatus.fZ;

    vehiclePose.latitude = msg->latitude;
    vehiclePose.longitude = msg->longitude;
    vehiclePose.altitude = msg->altitude;

    vehiclePose.accelerationX = msg->accelerationX;
    vehiclePose.accelerationY = msg->accelerationY;
    vehiclePose.accelerationZ = msg->accelerationZ;
    vehiclePose.angularRateX = msg->angularRateX;
    vehiclePose.angularRateY = msg->angularRateY;
    vehiclePose.angularRateZ = msg->angularRateZ;

    g_pPosePublisher->publish(vehiclePose);

    poseCounter = poseCounter%100;
    if(poseCounter%20 == 0)
    {
        g_pFootPrintPublisher->publish(footPrint);
    }

}
void PoseCallback(const inertial::NCOMConstPtr& msg)
{
    if(msg->navStatus != 4)//if not locked, publish old transform;
    {

        PublishTransform(msg->header.stamp, vehicleStatus.fX, vehicleStatus.fY, vehicleStatus.fZ, vehicleStatus.fYaw, vehicleStatus.fPitch, vehicleStatus.fRoll);
        PublishVehiclePose(msg);
        return;
    }

    //transformed into ros coordinates;

    vehicleStatus.fYaw = -msg->heading;
    vehicleStatus.fPitch = -msg->pitch;
    vehicleStatus.fRoll = msg->roll;
    vehicleStatus.fVx = msg->velocityNorth;
    vehicleStatus.fVy = -msg->velocityEast;
    vehicleStatus.fVz = -msg->velocityDown;
    vehicleStatus.fVelocity = sqrt(vehicleStatus.fVx*vehicleStatus.fVx+vehicleStatus.fVy*vehicleStatus.fVy);


    static ros::Time timeNow = ros::Time(0);
    static ros::Time timePast = ros::Time(0);


    /**initialize time stamp*/
    timeNow = msg->header.stamp;
    if(timePast.nsec == 0 && timePast.sec==0)
    {
        timePast = timeNow;
    }
//    std::cout<<"past:"<<std::setprecision(16)<<timePast.toSec()<<std::endl;
//    std::cout<<"now:"<<std::setprecision(16)<<timeNow.toSec()<<std::endl;

    /**calculate time duration;*/
    ros::Duration duration = timeNow-timePast;
    timePast = timeNow;
    float fT = duration.toSec();
    if(fT<0.0)
    {
        fT = 0.0;
    }

    /**integrate velocity on time*/
    if(vehicleStatus.fVelocity>0.05)
    {
        vehicleStatus.fX += fT*vehicleStatus.fVx;
        vehicleStatus.fY += fT*vehicleStatus.fVy;
        vehicleStatus.fZ += fT*vehicleStatus.fVz;
    }
    PublishTransform(msg->header.stamp, vehicleStatus.fX, vehicleStatus.fY, vehicleStatus.fZ, vehicleStatus.fYaw, vehicleStatus.fPitch, vehicleStatus.fRoll);
    PublishVehiclePose(msg);

}

int main(int argc, char** argv)
{
    /**initialize vehicle status*/
    vehicleStatus.fYaw = 0.0;
    vehicleStatus.fPitch = 0.0;
    vehicleStatus.fRoll = 0.0;
    vehicleStatus.fVx = 0.0;
    vehicleStatus.fVy = 0.0;
    vehicleStatus.fVz = 0.0;
    vehicleStatus.fX = 0.0;
    vehicleStatus.fY = 0.0;
    vehicleStatus.fZ = 0.0;
    vehicleStatus.fVelocity = 0.0;


    ros::init(argc, argv, strNodeName);
    ros::NodeHandle node;
    g_pPosePublisher = new ros::Publisher;
    g_pNCOMSubscriber = new ros::Subscriber;
    g_pFootPrintPublisher = new ros::Publisher;

    *g_pPosePublisher = node.advertise<transformation::VehiclePose>(strPubVehiclePoseTopicName, 20);
    *g_pFootPrintPublisher = node.advertise<geometry_msgs::PolygonStamped>(strPubFootPrintTopicName, 20);

    footPrint.header.frame_id = strVehicleFrameID;
    footPrint.polygon.points.resize(4);
    footPrint.polygon.points[0].x = 2.0;
    footPrint.polygon.points[0].y = -0.6;
    footPrint.polygon.points[0].z = 0.0;

    footPrint.polygon.points[1].x = -0.5;
    footPrint.polygon.points[1].y = -0.6;
    footPrint.polygon.points[1].z = 0.0;

    footPrint.polygon.points[2].x = -0.5;
    footPrint.polygon.points[2].y = 0.6;
    footPrint.polygon.points[2].z = 0.0;

    footPrint.polygon.points[3].x = 2.0;
    footPrint.polygon.points[3].y = 0.6;
    footPrint.polygon.points[3].z = 0.0;

    *g_pNCOMSubscriber = node.subscribe(strNCOMTopicName, 100, &PoseCallback);
    ros::spin();
}
