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
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/Marker.h>
#include <pcl/common/common.h>

class cloud_handling
{
public:
	cloud_handling (ros::NodeHandle nh, parameter_bag parameter);
	void Match ();
	void Callback (const sensor_msgs::PointCloud2ConstPtr& a_cloud_msg);
	void features_of_objects ();
	void visualize_marker (pcl::PointCloud<pcl::PointXYZ>::Ptr a_cloud_cluster,
						   visualization_msgs::Marker::Ptr a_marker,
						   int a_marker_id, float color_r, float color_g, float color_b);
private:
	parameter_bag parameter;
	ros::NodeHandle nh;
	ros::Subscriber sub;
};

#endif /* CLOUD_HANDLING_H */
