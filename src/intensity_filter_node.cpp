#include <iostream>
#include <ros/ros.h>
#include "lidar_road/intensity_filter.hpp"

int main(int argc, char** argv)
{
    ros::init(argc,argv,"lidar_road_node");

    ros::NodeHandle nh;
    intensity_filter::IntensityFilter rd_det(nh);
    ros::spin();
    
}