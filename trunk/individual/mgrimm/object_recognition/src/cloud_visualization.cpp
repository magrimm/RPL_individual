/*
 * cloud_visualization.cpp
 *
 *  Created on: Nov 24, 2015
 *      Author: marius
 */

#include <cloud_visualization.h>

cloud_visualization::cloud_visualization(visualization_bag vis_bag)
{
	visual_param = vis_bag;
}

void cloud_visualization::visualize_marker (pcl::PointCloud<pcl::PointXYZ>::Ptr a_cloud_cluster,
											visualization_msgs::Marker::Ptr a_marker, int a_marker_id,
											float a_color_r, float a_color_g, float a_color_b)
{
	  // Get smallest and biggest point coordinates
	  Eigen::Vector4f min_vector, max_vector;
	  pcl::getMinMax3D(*a_cloud_cluster, min_vector, max_vector);

	  // SET MARKER
	  a_marker->header.frame_id = visual_param.marker.frame_id;
	  a_marker->header.stamp = ros::Time();
	  a_marker->ns = visual_param.marker.ns;
	  a_marker->id = a_marker_id;
	  a_marker->type = visualization_msgs::Marker::CUBE;
	  a_marker->action = visualization_msgs::Marker::ADD;
	  // Calculate position of marker
	  a_marker->pose.position.x = (max_vector[0] + min_vector[0])/2;
	  a_marker->pose.position.y = (max_vector[1] + min_vector[1])/2;
	  a_marker->pose.position.z = (max_vector[2] + min_vector[2])/2;
	  a_marker->pose.orientation.x = visual_param.marker.orientation_x;
	  a_marker->pose.orientation.y = visual_param.marker.orientation_y;
	  a_marker->pose.orientation.z = visual_param.marker.orientation_z;
	  a_marker->pose.orientation.w = visual_param.marker.orientation_w;
	  // Calculate dimensions of marker
	  a_marker->scale.x = std::abs(max_vector[0] - min_vector[0]);
	  a_marker->scale.y = std::abs(max_vector[1] - min_vector[1]);
	  a_marker->scale.z = std::abs(max_vector[2] - min_vector[2]);
	  // Set color and transparancy of marker
	  a_marker->color.a = visual_param.marker.color_alpha;
	  a_marker->color.r = a_color_r;
	  a_marker->color.g = a_color_g;
	  a_marker->color.b = a_color_b;
}


