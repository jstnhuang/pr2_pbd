#include <iostream>

#include <ros/ros.h>
// #include <eigen_conversions/eigen_msg.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include "pr2_pbd_interaction/ICPTransform.h"


typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;


bool findICPTransform (pr2_pbd_interaction::ICPTransform::Request &req,
               pr2_pbd_interaction::ICPTransform::Response &res) {
    std::cout << "Recieved request" << std::endl;

    pcl::PCLPointCloud2 cloud_in_PCLP2, cloud_out_PCLP2;
    PointCloud::Ptr cloud_in(new PointCloud);
    PointCloud::Ptr cloud_out(new PointCloud);

    sensor_msgs::PointCloud2 pc1;
    sensor_msgs::PointCloud2 pc2;
    sensor_msgs::convertPointCloudToPointCloud2(req.pc1, pc1);
    sensor_msgs::convertPointCloudToPointCloud2(req.pc2, pc2);

    // convert input clouds to PCLPointCloud2s
    pcl_conversions::toPCL(pc1, cloud_in_PCLP2);
    pcl_conversions::toPCL(pc2, cloud_out_PCLP2);

    // convert PCLPointCloud2s to pcl::PointCloud<pcl::PointXYZ>
    pcl::fromPCLPointCloud2(cloud_in_PCLP2, *cloud_in);
    pcl::fromPCLPointCloud2(cloud_out_PCLP2, *cloud_out);

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputCloud(cloud_in);
    icp.setInputTarget(cloud_out);
    PointCloud Final;
    icp.align(Final);
    std::cout << "has converged: " << icp.hasConverged() << "\nscore: " <<
    icp.getFitnessScore() << std::endl;
    std::cout << icp.getFinalTransformation() << std::endl;

    res.fitness_score = icp.getFitnessScore();
}

int main(int argc, char *argv[])
{
    ros::init (argc, argv, "pbd_icp_transform");
    ros::NodeHandle n;
    ros::ServiceServer icpService = n.advertiseService ("icpTransform", findICPTransform);

    std::cout<<"Spawned ICP service. Ready for request."<<std::endl;

    ros::Duration(1.0).sleep();
    ros::spin ();

    return 0;
}
