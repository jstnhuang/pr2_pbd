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

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

bool computeAverageColors (pr2_pbd_interaction::ICPTransform::Request &req,
               pr2_pbd_interaction::ICPTransform::Response &res) {
    std::cout << "Recieved request" << std::endl;

    pcl::PCLPointCloud2 cloud_in_PCLP2, cloud_out_PCLP2;
    PointCloud::Ptr cloud_in(new PointCloud);

    sensor_msgs::PointCloud2 pc1;
    sensor_msgs::convertPointCloudToPointCloud2(req.pc1, pc1);

    // convert input clouds to PCLPointCloud2s
    pcl_conversions::toPCL(pc1, cloud_in_PCLP2);

    // convert PCLPointCloud2s to pcl::PointCloud<pcl::PointXYZ>
    pcl::fromPCLPointCloud2(cloud_in_PCLP2, *cloud_in);

    redness = 0;
    blueness = 0;
    greenness = 0;

    for (size_t i = 0; i < cloud_in->points.size (); ++i) {
    butt_in->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
    butt_in->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
    butt_in->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
  }


    res.redness = 
    res.blueness = 
    res.greenness = 
}

int main(int argc, char *argv[])
{
    ros::init (argc, argv, "pbd_icp_transform");
    ros::NodeHandle n;
    ros::ServiceServer icpService = n.advertiseService ("icpTransform", computeAverageColors);

    std::cout<<"Spawned ICP service. Ready for request."<<std::endl;

    ros::Duration(1.0).sleep();
    ros::spin ();

    return 0;
}
