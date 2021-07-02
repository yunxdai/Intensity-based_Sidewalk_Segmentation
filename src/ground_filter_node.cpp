// #pragma once
#include <iostream>
#include <ros/ros.h>
#include "lidar_road/ground_filter.hpp"

int main(int argc, char** argv)
{
    ros::init(argc,argv,"ground_filter_node");
    ros::NodeHandle nh("~");
    // rosFilter filter;
    ground_filter::GroundFilter ft(nh);
    ros::spin();
}