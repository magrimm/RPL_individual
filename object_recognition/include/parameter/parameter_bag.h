/*
 * parameter_bag.h
 *
 *  Created on: Nov 24, 2015
 *      Author: marius
 */

#ifndef PARAMETER_BAG_H
#define PARAMETER_BAG_H

#include <string>
#include <parameter/filter_bag.h>
#include <parameter/segmentation_bag.h>
#include <parameter/recognition_bag.h>
#include <parameter/visualization_bag.h>

struct parameter_bag
{
	filter_bag filter;
	segmentation_bag segmentation;
	recognition_bag recognition;
	visualization_bag visualization;

	std::string node_name, cloud_frame_id, object_files_path;
	std::string subscribed_rostopic, pub_topic_pointcloud, pub_topic_marker;
	int queue_size_subscriber, queue_size_pub_pointcloud, queue_size_pub_marker;
};

#endif /* PARAMETER_BAG_H_ */
