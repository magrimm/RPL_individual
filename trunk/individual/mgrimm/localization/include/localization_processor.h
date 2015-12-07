/*
 * localization_processor.h
 *
 *  Created on: Dec 4, 2015
 *      Author: marius
 */

#ifndef _LOCALIZATION_PROCESSOR_H_
#define _LOCALIZATION_PROCESSOR_H_

#include <parameter/parameter_bag.h>
#include <parameter/distribution_bag.h>
#include <parameter/resample_bag.h>
#include <parameter/motion_update_bag.h>
#include <parameter/sensor_update_bag.h>
#include <parameter/visualization_bag.h>
#include <ros/ros.h>
#include <motion_update.h>
#include <sensor_update.h>
#include <resample.h>
#include <visualization.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <math.h>
#include <numeric>

struct position3D
{
	float x, y, z;
};

struct orientation_quat
{
	float x, y, z, w;
};

struct pose
{
	position3D position;
	orientation_quat orientation;
};

struct robot_control
{
	pose odometry[2];
};


class localization_processor
{
public:
	localization_processor (ros::NodeHandle nh, parameter_bag parameter);
	void Callback_map (const nav_msgs::OccupancyGrid::ConstPtr& a_map_msg);
	void Callback_odom (const nav_msgs::OdometryConstPtr& a_odom_msg);
	void Callback_scan (const sensor_msgs::LaserScanConstPtr& a_scan_msg);
	void get_particles ();

private:
	ros::NodeHandle nh;
	parameter_bag parameter;
	nav_msgs::OccupancyGrid::ConstPtr map;
	std::vector<int> map_data;
	ros::Publisher pub;
	std::vector<pose> particles;
	std::vector<float> weights;

	robot_control control;
	pose pose_empty;
	std::vector<robot_control> control_vec;

	bool sig_odom, sig_scan;
};

#endif /* _LOCALIZATION_PROCESSOR_H_ */
