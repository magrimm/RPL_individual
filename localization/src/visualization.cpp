/*
 * visualization.cpp
 *
 *  Created on: Dec 6, 2015
 *      Author: marius
 */

#include <visualization.h>

visualization::visualization (visualization_bag visualization_params)
{
	visual_param = visualization_params;
}


void visualization::visualize_particle (std::vector<geometry_msgs::Pose> particles,
											visualization_msgs::MarkerArray::Ptr a_marker_array, int a_marker_id,
											float a_color_r, float a_color_g, float a_color_b)
{
	for (int i = 0; i < particles.size(); ++i)
	{
		visualization_msgs::Marker::Ptr a_marker;

		// SET MARKER
		a_marker->header.frame_id = visual_param.frame_id;
		a_marker->header.stamp = ros::Time();
		a_marker->ns = visual_param.ns;
		a_marker->id = a_marker_id;
		a_marker->type = visualization_msgs::Marker::ARROW;
		a_marker->action = visualization_msgs::Marker::ADD;
		// Calculate position of marker
		a_marker->pose.position = particles.at(i).position;
		a_marker->pose.orientation = particles.at(i).orientation;
		// Calculate dimensions of marker
		a_marker->scale.x = 1.0;
		a_marker->scale.y = 1.0;
		a_marker->scale.z = 1.0;
		// Set color and transparancy of marker
		a_marker->color.a = visual_param.color_alpha;
		a_marker->color.r = 1.0;//a_color_r;
		a_marker->color.g = 0.0;//a_color_g;
		a_marker->color.b = 0.0;//a_color_b;

		a_marker_array->markers.push_back(*a_marker);
	}
}

void visualization::visualize_particle_pose (geometry_msgs::PoseArray::Ptr poseArray, std::vector<pose> particles)
{
	poseArray->poses.clear(); // Clear last block perception result
	poseArray->header.stamp = ros::Time::now();
	poseArray->header.frame_id = "odometry_link";

	for (int i = 0; i < particles.size(); ++i)
	{
		geometry_msgs::Pose pose;

		//adding pose to pose array
		pose.position.x = particles.at(i).position.x;
		pose.position.y = particles.at(i).position.y;
		pose.position.z = particles.at(i).position.z;
		pose.orientation.x = particles.at(i).orientation.x;
		pose.orientation.y = particles.at(i).orientation.y;
		pose.orientation.z = particles.at(i).orientation.z;
		pose.orientation.w = particles.at(i).orientation.w;
		poseArray->poses.push_back(pose);
	}
}


