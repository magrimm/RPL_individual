/*
 * cloud_handling.h
 *
 *  Created on: Nov 23, 2015
 *      Author: marius
 */

#ifndef CLOUD_HANDLING_H
#define CLOUD_HANDLING_H

#include <parameter/parameter_bag.h>
#include <cloud_filter.h>
#include <cloud_segmentation.h>
#include <cloud_matching.h>
#include <cloud_visualization.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

class cloud_handling
{
public:
	cloud_handling (ros::NodeHandle nh, parameter_bag parameter);
	void Callback (const sensor_msgs::PointCloud2ConstPtr& a_cloud_msg);
	void features_of_objects ();
	std::vector<std::vector<pcl::PointCloud<pcl::Histogram<153> > > > object_database_spin_images;

private:
	parameter_bag parameter;
	ros::NodeHandle nh;
	ros::Subscriber sub;
	ros::Publisher pub, vis_pub;
};

#endif /* CLOUD_HANDLING_H */
