#include <iostream>

#include "LaserScanToPointCloud.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "Lms1ToPC");
    ros::NodeHandle node;    
    LaserScanToPointCloud Lms1(node);
    ros::spin();
}
