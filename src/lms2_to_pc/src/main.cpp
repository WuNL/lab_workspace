#include <iostream>
#include "LaserScanToPointCloud.h"
int main(int argc, char** argv)
{
    ros::init(argc, argv, "Lms2ToPC");
    ros::NodeHandle node;
    LaserScanToPointCloud Lms2(node);
    ros::spin();

    return (0);
}
