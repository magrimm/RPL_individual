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
	  Eigen::Vector4f min_vector, max_vector;
	  pcl::getMinMax3D(*a_cloud_cluster, min_vector, max_vector);

	  // SET MARKER
	  a_marker->header.frame_id = "camera_depth_optical_frame";
	  a_marker->header.stamp = ros::Time();
	  a_marker->ns = "my_namespace";
	  a_marker->id = a_marker_id;
	  a_marker->type = visualization_msgs::Marker::CUBE;
	  a_marker->action = visualization_msgs::Marker::ADD;
	  a_marker->pose.position.x = (max_vector[0] + min_vector[0])/2;
	  a_marker->pose.position.y = (max_vector[1] + min_vector[1])/2;
	  a_marker->pose.position.z = (max_vector[2] + min_vector[2])/2;
	  a_marker->pose.orientation.x = 0.0;
	  a_marker->pose.orientation.y = 0.0;
	  a_marker->pose.orientation.z = 0.0;
	  a_marker->pose.orientation.w = 1.0;
	  a_marker->scale.x = std::abs(max_vector[0] - min_vector[0]);
	  a_marker->scale.y = std::abs(max_vector[1] - min_vector[1]);
	  a_marker->scale.z = std::abs(max_vector[2] - min_vector[2]);
	  a_marker->color.a = 0.5;
	  a_marker->color.r = a_color_r;
	  a_marker->color.g = a_color_g;
	  a_marker->color.b = a_color_b;
	  //only if using a MESH_RESOURCE marker type:
	  a_marker->mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
}


