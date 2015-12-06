/*
 * parameter_bag.h
 *
 *  Created on: Dec 4, 2015
 *      Author: marius
 */

#ifndef _PARAMETER_BAG_H_
#define _PARAMETER_BAG_H_

#include <string>
#include <parameter/distribution_bag.h>
#include <parameter/resample_bag.h>
#include <parameter/motion_update_bag.h>
#include <parameter/visualization_bag.h>

struct parameter_bag
{
	std::string node_name;

	std::string subscribed_rostopic_odom, subscribed_rostopic_map, subscribed_rostopic_scan;
	int queue_size_subscriber_odom, queue_size_subscriber_map, queue_size_subscriber_scan;

	std::string pub_topic_map, pub_topic_particles, pub_topic_scan, pub_topic_tf;
	int queue_size_pub_map, queue_size_pub_particles, queue_size_pub_scan, queue_size_pub_tf;

	distribution_bag distribution;
	resample_bag resample;
	motion_update_bag motion_update;
	visualization_bag visualization;
};

#endif /* _PARAMETER_BAG_H_ */
