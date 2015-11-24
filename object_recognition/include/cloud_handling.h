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
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

class cloud_handling
{
public:
	cloud_handling (ros::NodeHandle nh, parameter_bag parameter);
	void Match ();
	void Callback (const sensor_msgs::PointCloud2ConstPtr& a_cloud_msg);
	void LoadFromPCD ();
private:
	parameter_bag parameter;
	ros::NodeHandle nh;
	ros::Subscriber sub;
};

#endif /* CLOUD_HANDLING_H */
