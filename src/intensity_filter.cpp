#include <iostream>
#define PCL_NO_PRECOMPILE

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/centroid.h>
#include <Eigen/Dense>
#include <pcl/features/intensity_gradient.h>
#include <pcl/features/normal_3d.h>
#include "lidar_road/jenk.hpp"
#include "lidar_road/dkm_parallel.hpp"
#include "lidar_road/intensity_filter.hpp"
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/filters/convolution.h>

#include <Eigen/Dense>

namespace intensity_filter
{


bool comp_up(const IPoint &a, const IPoint &b)
{
    return a.y < b.y;
}

// Descending. (For points where y is less than 0)
bool comp_down(const IPoint &a, const IPoint &b)
{
    return a.y > b.y;
}

IntensityFilter::IntensityFilter(ros::NodeHandle& nh): nh_(nh)
{
    // Parameters
    // nh_.param<double>("sensor_height",sensor_height_,0.83);
    nh_.param<double>("ground_height_tolerance",max_ground_height_,0.2);
    nh_.param<double>("ground_normal_deviation",max_deviation_angle_,0.1);
    nh_.param<double>("intensity_gradient",intensity_gradient_threshold_,200.0);
    nh_.param<double>("texture_intensity",cement_intensity_center_,6.0);
    nh_.param<double>("intensity_tolerance",cement_intensity_tolerance_,3.5);
    nh_.param<int>("terrain_number",terrain_cls_num_,2);

    //Topic
    nh_.param<std::string>("pb_ground_topic",ground_topic_,"ground_points");
    nh_.param<std::string>("pb_road_topic",road_topic_,"road_points");
    nh_.param<std::string>("input_pointcloud",pointcloud_topic_,"velodyne_points");

    // Publishers & Subscribers
    ground_publisher = nh_.advertise<sensor_msgs::PointCloud2>(ground_topic_,1);
    intensity_edge_publisher= nh_.advertise<sensor_msgs::PointCloud2>(road_topic_,1);
    sub=nh_.subscribe(pointcloud_topic_,8,&IntensityFilter::pointCloudCallback,this);

}

void IntensityFilter::pointCloudCallback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input)
{
    pcl::PointCloud<IPoint>::Ptr original_cloud_ptr(new pcl::PointCloud<IPoint>);
    
    // extract approximate ground, filter out high obstacle (like trees, buildings etc.)
    pcl::fromROSMsg(*input,*original_cloud_ptr);
    pcl::PointCloud<IPoint>::Ptr downsampled_cloud_ptr(new pcl::PointCloud<IPoint>);
    pcl::PointCloud<IPoint>::Ptr ground_cloud_ptr(new pcl::PointCloud<IPoint>);
    preProcess(original_cloud_ptr,downsampled_cloud_ptr);
    // preProcess(original_cloud_ptr,ground_cloud_ptr);
    getGround(downsampled_cloud_ptr,ground_cloud_ptr);

    // publish ground point cloud
    sensor_msgs::PointCloud2 ground_cloud;
    pcl::toROSMsg(*ground_cloud_ptr,ground_cloud);
    ground_cloud.header.stamp = input->header.stamp;
    ground_cloud.header.frame_id = input->header.frame_id;
    ground_publisher.publish(ground_cloud);
    
    kmeansThresholdSelection(ground_cloud_ptr);
    // jenksThresholdSelection(ground_cloud_ptr); // problem: breakpoint from jenk is bigger 


    pcl::PointCloud<IPoint>::Ptr cement_cloud_ptr(new pcl::PointCloud<IPoint>);
    intensitySegmentation(ground_cloud_ptr,cement_cloud_ptr);


    sensor_msgs::PointCloud2 cement_cloud;
    pcl::toROSMsg(*cement_cloud_ptr,cement_cloud);
    cement_cloud.header.stamp = input->header.stamp;
    cement_cloud.header.frame_id = input->header.frame_id;
    intensity_edge_publisher.publish(cement_cloud);
}

void IntensityFilter::preProcess(const pcl::PointCloud<IPoint>::Ptr &in_cloud_ptr, const pcl::PointCloud<IPoint>::Ptr &out_cloud_ptr)
{
    pcl::PointCloud<IPoint>::Ptr filtered_cloud_ptr(new pcl::PointCloud<IPoint>);

    // Downsampling
    pcl::VoxelGrid<IPoint> voxelSampler;
    voxelSampler.setInputCloud(in_cloud_ptr);
    voxelSampler.setLeafSize(0.06f,0.06f,0.06f);
    voxelSampler.filter(*filtered_cloud_ptr);
    // Clipping Range
    int x_range,y_range;
    nh_.param<int>("x_range",x_range,10);
    nh_.param<int>("y_range",y_range,10);

    // Clipping
    pcl::ConditionalRemoval<IPoint> condition;
    pcl::ConditionAnd<IPoint>::Ptr and_cond(new pcl::ConditionAnd<IPoint>());
    and_cond->addComparison(pcl::FieldComparison<IPoint>::ConstPtr(new pcl::FieldComparison<IPoint>("y",pcl::ComparisonOps::GT,-y_range)));
    and_cond->addComparison(pcl::FieldComparison<IPoint>::ConstPtr(new pcl::FieldComparison<IPoint>("y",pcl::ComparisonOps::LT,y_range)));
    and_cond->addComparison(pcl::FieldComparison<IPoint>::ConstPtr(new pcl::FieldComparison<IPoint>("x",pcl::ComparisonOps::GT,-x_range)));
    and_cond->addComparison(pcl::FieldComparison<IPoint>::ConstPtr(new pcl::FieldComparison<IPoint>("x",pcl::ComparisonOps::LT,x_range)));
    condition.setCondition(and_cond);
    condition.setInputCloud(filtered_cloud_ptr);
    condition.filter(*out_cloud_ptr);
}

void IntensityFilter::getGround(const pcl::PointCloud<IPoint>::Ptr &in_cloud_ptr, const pcl::PointCloud<IPoint>::Ptr &out_cloud_ptr)
{
    pcl::SACSegmentation<IPoint> seg;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setAxis(Eigen::Vector3f(0, 0, 1));
    seg.setEpsAngle(max_deviation_angle_);
    seg.setDistanceThreshold(max_ground_height_);//floor distance
    seg.setOptimizeCoefficients(true);
    seg.setInputCloud(in_cloud_ptr);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.empty()) {
        ROS_INFO("Could not estimate a planar model for the given dataset");
    }

    //EXTRACT THE FLOOR FROM THE CLOUD
    pcl::ExtractIndices<IPoint> extract;
    extract.setInputCloud(in_cloud_ptr);
    extract.setIndices(inliers);
    
    extract.setNegative(false);//true removes the indices, false leaves only the indices
    extract.filter(*out_cloud_ptr);
}

void IntensityFilter::intensitySegmentation(const pcl::PointCloud<IPoint>::Ptr &in_cloud_ptr, const pcl::PointCloud<IPoint>::Ptr &out_cloud_ptr)
{
    ROS_INFO("CEMENT INTENSITY = %f",cement_intensity_center_);
    // std::cout << "intensity_cement = " << cement_intensity_center_ << std::endl;

    pcl::ConditionalRemoval<IPoint> condition;
    pcl::ConditionAnd<IPoint>::Ptr and_cond(new pcl::ConditionAnd<IPoint>());
    pcl::PointCloud<IPoint>::Ptr temp_cloud_ptr(new pcl::PointCloud<IPoint>);

    // and_cond->addComparison(pcl::FieldComparison<IPoint>::ConstPtr(new pcl::FieldComparison<IPoint>("intensity",pcl::ComparisonOps::GT,cement_intensity_center_ - cement_intensity_tolerance_)));
    // and_cond->addComparison(pcl::FieldComparison<IPoint>::ConstPtr(new pcl::FieldComparison<IPoint>("intensity",pcl::ComparisonOps::LT,cement_intensity_center_ + cement_intensity_tolerance_)));
    
    and_cond->addComparison(pcl::FieldComparison<IPoint>::ConstPtr(new pcl::FieldComparison<IPoint>("intensity",pcl::ComparisonOps::GT,0)));
    and_cond->addComparison(pcl::FieldComparison<IPoint>::ConstPtr(new pcl::FieldComparison<IPoint>("intensity",pcl::ComparisonOps::LT,cement_intensity_center_)));
    condition.setCondition(and_cond);
    condition.setInputCloud(in_cloud_ptr);
    condition.filter(*temp_cloud_ptr);

    pcl::StatisticalOutlierRemoval<IPoint> statFilter;
    statFilter. setInputCloud(temp_cloud_ptr);
    statFilter.setMeanK(50);
    statFilter.setStddevMulThresh(0.3);
    statFilter.filter(*out_cloud_ptr);

}

void IntensityFilter::jenksThresholdSelection(const pcl::PointCloud<IPoint>::Ptr &in_cloud_ptr)
{
    std::vector<double> intensity_vec;
    for (auto point:in_cloud_ptr->points)
    {
        intensity_vec.push_back(point.intensity);
    }
    ValueCountPairContainer sortedUniqueValueCounts;
	GetValueCountPairs(sortedUniqueValueCounts, &intensity_vec[0], intensity_vec.size());
    LimitsContainer resultingbreaksArray;
	ClassifyJenksFisherFromValueCountPairs(resultingbreaksArray, terrain_cls_num_, sortedUniqueValueCounts);
    cement_intensity_center_ = resultingbreaksArray[1];
}

void IntensityFilter::kmeansThresholdSelection(const pcl::PointCloud<IPoint>::Ptr &in_cloud_ptr)
{
    std::vector<std::array<double,1>> intensity_vec;
    for (auto point:in_cloud_ptr->points)
    {
        std::array<double,1> temp{point.intensity};
        intensity_vec.push_back(temp);
    }
    auto cluster_data = dkm::kmeans_lloyd_parallel(intensity_vec,2);

    cement_intensity_center_ = 255;

    for (const auto & mean:std::get<0>(cluster_data))
    {
        if (mean[0] < cement_intensity_center_) cement_intensity_center_ = mean[0];
    }
    
}


}




