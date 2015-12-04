/*
 * localization_node.cpp
 *
 *  Created on: Dec 3, 2015
 *      Author: marius
 */

#include <parameter/parameter_bag.h>
#include <ros/ros.h>

int main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "localization_node");
	ros::NodeHandle nh;

	// Initialize parameter structure
	parameter_bag parameter;

	// Retrieve all parameters
	nh.getParam("test_string", parameter.test);
	std::cout << parameter.test << std::endl;

	// Spin
	ros::spin ();
}
