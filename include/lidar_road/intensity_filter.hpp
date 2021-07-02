#pragma once

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
#include <pcl/kdtree/kdtree_flann.h>

typedef pcl::PointXYZI IPoint;

namespace intensity_filter
{

bool comp_up(const IPoint &a, const IPoint &b);

// Descending. (For points where y is less than 0)
bool comp_down(const IPoint &a, const IPoint &b);


class IntensityFilter
{
private:
    // parameters
    double max_ground_height_;
    double intensity_gradient_threshold_;
    double max_deviation_angle_;
    double cement_intensity_center_;
    double cement_intensity_tolerance_;
    int terrain_cls_num_;

    // topic
    std::string ground_topic_;
    std::string road_topic_;
    std::string pointcloud_topic_;
    // ros
    ros::NodeHandle nh_;
    ros::Publisher ground_publisher;
    ros::Publisher intensity_edge_publisher;
    ros::Subscriber sub;

protected:
    // auxiliary functions
    void preProcess(const pcl::PointCloud<IPoint>::Ptr &in_cloud_ptr, const pcl::PointCloud<IPoint>::Ptr &out_cloud_ptr);
    void getGround(const pcl::PointCloud<IPoint>::Ptr &in_cloud_ptr, const pcl::PointCloud<IPoint>::Ptr &out_cloud_ptr);
    void intensitySegmentation(const pcl::PointCloud<IPoint>::Ptr &in_cloud_ptr, const pcl::PointCloud<IPoint>::Ptr &out_cloud_ptr);
    void jenksThresholdSelection(const pcl::PointCloud<IPoint>::Ptr &in_cloud_ptr);
    void kmeansThresholdSelection(const pcl::PointCloud<IPoint>::Ptr &in_cloud_ptr);
    void prePointCleaner(std::vector<pcl::PointCloud<IPoint> > ringed_cloud);

public:
    // public methods
    IntensityFilter(ros::NodeHandle& nh);    
    void pointCloudCallback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input);
    // using ring numbers
    void ringCallback(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input);
    // ~IntensityFilter();
};

}
