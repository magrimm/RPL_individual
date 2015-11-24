/*
 * cloud_visualization.h
 *
 *  Created on: Nov 24, 2015
 *      Author: marius
 */

#ifndef CLOUD_VISUALIZATION_H
#define CLOUD_VISUALIZATION_H

#include <parameter/visualization_bag.h>
#include <visualization_msgs/Marker.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/common.h>

class cloud_visualization
{
public:

	cloud_visualization (visualization_bag visual_param);

	void visualize_marker (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster,
						   visualization_msgs::Marker::Ptr marker, int marker_id,
						   float color_r, float color_g, float color_b);

private:

	visualization_bag visual_param;

};

#endif /* CLOUD_VISUALIZATION_H */
