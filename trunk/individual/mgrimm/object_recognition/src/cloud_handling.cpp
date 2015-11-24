/*
 * cloud_handling.cpp
 *
 *  Created on: Nov 23, 2015
 *      Author: marius
 */

#include <cloud_handling.h>

cloud_handling::cloud_handling (ros::NodeHandle nodehandle, parameter_bag params_bag)
{
	nh = nodehandle;
	parameter = params_bag;
}

void cloud_handling::Match ()
{
	ros::Subscriber sub = nh.subscribe(parameter.subscribed_rostopic, parameter.queue_size_subscriber, &cloud_handling::Callback, this);
}

void cloud_handling::LoadFromPCD ()
{

}

void cloud_handling::Callback (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
	// Container for original & filtered data
	pcl::PCLPointCloud2* temp_cloud (new pcl::PCLPointCloud2);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_resolution (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_passthrough (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_outlier (new pcl::PointCloud<pcl::PointXYZ>);

	// Convert from sensor_msgs to PCLPointCloud2 (PCL data type)
	pcl_conversions::toPCL(*cloud_msg, *temp_cloud);
	// Convert from PCLPointCloud2 to PointCloud
	pcl::fromPCLPointCloud2(*temp_cloud, *cloud);

	// Construct the class cloud_filter with its parameters
	cloud_filter c_filter(parameter.filter);
	// Apply resolution_filter
	c_filter.resolution_filter(cloud, cloud_filtered_resolution);

	// Apply passthrough_filter
	c_filter.passthrough_filter(cloud_filtered_resolution, cloud_filtered_passthrough);

	// Apply statistical_outlier_removal_filter
	c_filter.statistical_outlier_removal_filter(cloud_filtered_passthrough, cloud_filtered_outlier);

	// Construct the class cloud_segmentation with its parameters
	cloud_segmentation c_segmentation (parameter.segmentation);
	// Apply euclidean cluster extraction and return vector of cloud cluster
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>* cloud_cluster (new std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>);
	c_segmentation.euclidean_cluster_extraction(cloud_filtered_outlier, cloud_cluster);
}
