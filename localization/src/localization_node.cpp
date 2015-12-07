/*
 * localization_node.cpp
 *
 *  Created on: Dec 3, 2015
 *      Author: marius
 */

#include <parameter/parameter_bag.h>
#include <localization_processor.h>

int main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "localization_node");
	ros::NodeHandle nh;

	// Initialize parameter structure
	parameter_bag parameter;

	// Retrieve all parameters
	nh.getParam("subscribed_rostopic_odom", parameter.subscribed_rostopic_odom);
	nh.getParam("queue_size_subscriber_odom", parameter.queue_size_subscriber_odom);
	nh.getParam("subscribed_rostopic_map", parameter.subscribed_rostopic_map);
	nh.getParam("queue_size_subscriber_map", parameter.queue_size_subscriber_map);
	nh.getParam("subscribed_rostopic_scan", parameter.subscribed_rostopic_scan);
	nh.getParam("queue_size_subscriber_scan", parameter.queue_size_subscriber_scan);
	nh.getParam("pub_topic_particles", parameter.pub_topic_particles);
	nh.getParam("queue_size_pub_particles", parameter.queue_size_pub_particles);

	// Parameters for distributions
	nh.getParam("max_value_norm_dist_sq", parameter.distribution.max_value_norm_dist_sq);
	nh.getParam("max_value_tri_dist_sq", parameter.distribution.max_value_tri_dist_sq);
	// Parameters for motion update
	nh.getParam("alpha1", parameter.motion_update.alpha1);
	nh.getParam("alpha2", parameter.motion_update.alpha2);
	nh.getParam("alpha3", parameter.motion_update.alpha3);
	nh.getParam("alpha4", parameter.motion_update.alpha4);
	// Parameters for visualization
	nh.getParam("frame_id", parameter.visualization.frame_id);
	nh.getParam("ns", parameter.visualization.ns);
	nh.getParam("mesh_resource", parameter.visualization.mesh_resource);
	nh.getParam("color_alpha", parameter.visualization.color_alpha);

	// Construct class localization_processor with ros::NodeHandle and parameter structure
	localization_processor localization (nh, parameter);

	// Create ROS subscriber for the map, odometry and scan
	ros::Subscriber sub_map = nh.subscribe(parameter.subscribed_rostopic_map,
									   parameter.queue_size_subscriber_map,
									   &localization_processor::Callback_map,
									   &localization);
	ros::Subscriber sub_odom = nh.subscribe(parameter.subscribed_rostopic_odom,
									   parameter.queue_size_subscriber_odom,
									   &localization_processor::Callback_odom,
									   &localization);
	ros::Subscriber sub_scan = nh.subscribe(parameter.subscribed_rostopic_scan,
									   parameter.queue_size_subscriber_scan,
									   &localization_processor::Callback_scan,
									   &localization);
	// Spin
	ros::spin ();
}
