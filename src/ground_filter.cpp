// #pragma once
#define PCL_NO_PRECOMPILE

#include <iostream>
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

#include "lidar_road/ground_filter.hpp"




namespace ground_filter
{

bool point_cmp(VPoint a,VPoint b)
{
    return a.z < b.z;
}

GroundFilter::GroundFilter(ros::NodeHandle& nh):  nh_(nh)
{
    nh_.param<std::string>("output_topic",output_topic_,"final_road_points");
    nh_.param<std::string>("input_topic",input_topic_,"road_points");
    nh_.param<float>("sensor_height",sensor_height_,0.83);
    nh_.param<int>("iteration_number",num_iter_,10);
    nh_.param<int>("lpr_number",num_lpr_,100);
    nh_.param<double>("seeds_threshold",th_seeds_,0.05);
    nh_.param<double>("distance_threshold",th_dist_,0.07);


    pub = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_,3);
    sub=nh_.subscribe(input_topic_,3,&GroundFilter::groundExtraction,this);
    g_seeds_pc.reset(new pcl::PointCloud<VPoint>());
    g_ground_pc.reset(new pcl::PointCloud<VPoint>());
    g_not_ground_pc.reset(new pcl::PointCloud<VPoint>());
}

void GroundFilter::groundExtraction(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input)
{
    pcl::fromROSMsg(*input,cloud_in);
    /*
    pcl::PointCloud<VPoint>::Ptr cloud_pre(new pcl::PointCloud<VPoint>);
    pcl::fromROSMsg(*input,*cloud_pre);
    
    // Clipping Range
    int x_range,y_range;
    nh_.param<int>("x_range",x_range,3);
    nh_.param<int>("y_range",y_range,3);
    // Clipping
    pcl::ConditionalRemoval<VPoint> condition;
    pcl::ConditionAnd<VPoint>::Ptr and_cond(new pcl::ConditionAnd<VPoint>());
    and_cond->addComparison(pcl::FieldComparison<VPoint>::ConstPtr(new pcl::FieldComparison<VPoint>("y",pcl::ComparisonOps::GT,-y_range)));
    and_cond->addComparison(pcl::FieldComparison<VPoint>::ConstPtr(new pcl::FieldComparison<VPoint>("y",pcl::ComparisonOps::LT,y_range)));
    condition.setCondition(and_cond);
    condition.setInputCloud(cloud_pre);
    condition.filter(cloud_in);
    */
    // pcl::fromROSMsg(*input,cloud_in);
    pcl::PointCloud<VPoint> cloud_org(cloud_in);

    // sort by height
    sort(cloud_in.points.begin(),cloud_in.points.end(),point_cmp);

    // error point removal
    pcl::PointCloud<VPoint>::iterator it = cloud_in.points.begin();
    for (auto point_i : cloud_in.points)
    {
        if (point_i.z < -1.5 * sensor_height_)
        {
            it++;
        }
        else
        {
            break; // sorted pointcloud, so once condition not satisfied, break
        } 
    }
    cloud_in.erase(cloud_in.points.begin(),it);

    // extract init ground seeds
    extractInitialSeeds(cloud_in);
    g_ground_pc = g_seeds_pc;

    // ground plane fitter mainloop
    for (int i=0;i<num_iter_;i++)
    {
        estimatePlane();
        g_ground_pc->clear();
        g_not_ground_pc->clear();

        // pointcloud to matrix
        MatrixXf points(cloud_org.points.size(),3);
        int j = 0;
        for (auto p:cloud_org.points)
        {
            points.row(j++) << p.x,p.y,p.z;
        }

        VectorXf result = points * normal_; // result=Ax+By+Cz
        //threshold filter
        for (int r=0;r<result.rows();r++)
        {
            if (result[r]<th_dist_d_)
            {
                g_ground_pc->points.push_back(cloud_org[r]);
            }
            else
            {
                g_not_ground_pc->points.push_back(cloud_org[r]);
            }
            
        }
    }

    g_ground_pc->resize(g_ground_pc->points.size());
    g_not_ground_pc->resize(g_not_ground_pc->points.size());
    
    //remove nan points
    std::vector<int> mapping;
    pcl::PointCloud<VPoint> no_ground_noNan(*g_not_ground_pc);
    no_ground_noNan.is_dense=false;
    pcl::removeNaNFromPointCloud(no_ground_noNan,no_ground_noNan,mapping);

    // convert to ROS pointcloud
    pcl::toROSMsg(*g_ground_pc,cloud_out_ros);
    // cloud_out_ros.header.stamp=ros::Time::now();
    cloud_out_ros.header.stamp = input->header.stamp;
    
    cloud_out_ros.header.frame_id="velodyne";
    pub.publish(cloud_out_ros);

}

void GroundFilter::estimatePlane(void)
{
    Eigen::Matrix3f cov;
    Eigen::Vector4f pc_mean;
    pcl::computeMeanAndCovarianceMatrix(*g_ground_pc,cov,pc_mean);
    // SVD for normal vector
    JacobiSVD<MatrixXf> svd(cov,Eigen::DecompositionOptions::ComputeFullU);
    // smallest eigen value as normal
    normal_ = (svd.matrixU().col(2));
    // mean ground seeds value
    Eigen::Vector3f seeds_mean = pc_mean.head<3>();
    
    // Ax+By+Cz=D
    // normal.T*[x,y,z] = -d
    d_ = -(normal_.transpose()*seeds_mean)(0,0);
    th_dist_d_ = th_dist_ - d_;
}

void GroundFilter::extractInitialSeeds(const pcl::PointCloud<VPoint>& p_sorted)
{
    double sum=0;
    int cnt=0;
    // mean height value of lowest n=lpr_ points
    for (size_t i=0;i<p_sorted.points.size() && cnt<num_lpr_;i++)
    {
        sum += p_sorted.points[i].z;
        cnt++;
    }
    double lpr_height = cnt!=0 ? sum/cnt : 0;
    g_seeds_pc->clear();
    
    // filter points whose height <= lpr_height + threshold
    for (auto point_i : p_sorted.points)
    {
        if(point_i.z < lpr_height+th_seeds_)
        {
            g_seeds_pc->points.push_back(point_i);
        }
        else
        {
            break;
        }
    }
}

}