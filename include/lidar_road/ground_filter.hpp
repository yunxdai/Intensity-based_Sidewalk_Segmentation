// #pragma once
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
// #include <velodyne_pcl/point_types.h>
#include <Eigen/Dense>


using namespace std;
using Eigen::JacobiSVD;
using Eigen::VectorXf;
using Eigen::MatrixXf;

typedef pcl::PointXYZI VPoint;
// typedef velodyne_pcl::PointXYZIRT VPoint;

namespace ground_filter
{

class GroundFilter
{
    private:
        float sensor_height_;
        MatrixXf normal_;
        float d_;
        double th_dist_;
        double th_seeds_;
        float th_dist_d_;
        int num_lpr_;
        int num_iter_;
        // float in_max_height;

        std::string output_topic_;
        std::string input_topic_;

        ros::NodeHandle nh_;
        ros::Publisher pub;
        ros::Subscriber sub;
        pcl::PointCloud<VPoint> cloud_in;
        pcl::PointCloud<VPoint> cloud_out;
        pcl::PointCloud<VPoint> cloud_downsampled;
        pcl::PointCloud<VPoint> cloud_segmented;

        pcl::PointCloud<VPoint>::Ptr g_seeds_pc; // ground point seed
        pcl::PointCloud<VPoint>::Ptr g_ground_pc; // initial ground point
        pcl::PointCloud<VPoint>::Ptr g_not_ground_pc; //non ground point

        sensor_msgs::PointCloud2 cloud_out_ros;
        sensor_msgs::PointCloud2 ground_cloud_ros;


        void estimatePlane(void);
        void extractInitialSeeds(const pcl::PointCloud<VPoint>& p_sorted);
        

    public:
        GroundFilter(ros::NodeHandle& nh);
        void groundExtraction(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input);
};

}

