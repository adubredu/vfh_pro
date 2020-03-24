#include <ros/ros.h>
#include <iostream>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

ros::Publisher pub;

void cloud_callback(const pcl::PCLPointCloud2ConstPtr& input)
{
	pcl::PCLPointCloud2::Ptr cloud_filtered(new pcl::PCLPointCloud2);
	pcl::PCLPointCloud2::Ptr cloud_pass(new pcl::PCLPointCloud2);

	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;

    pcl::PassThrough<pcl::PCLPointCloud2> pass_through_filter;

    //pass through filter
    pass_through_filter.setInputCloud (input);
    pass_through_filter.setFilterFieldName("y");
    pass_through_filter.setFilterLimits(-0.3,0.2);
    pass_through_filter.filter (*cloud_pass);

    //voxel grid filter
    sor.setInputCloud(cloud_pass);
    sor.setLeafSize(0.05, 0.05, 0.05);
    sor.filter(*cloud_filtered);


    // Publish the data
    pub.publish(*cloud_filtered);
}




int main (int argc, char** argv)
{
	
	ros::init (argc, argv, "passthrough_filter");
	ros::NodeHandle nh;

	ros::Subscriber sub = nh.subscribe ("/camera/depth/points", 1, cloud_callback);
	pub = nh.advertise<sensor_msgs::PointCloud2> ("/filtered_cloud", 1);

	// Spin
	ros::spin ();
}