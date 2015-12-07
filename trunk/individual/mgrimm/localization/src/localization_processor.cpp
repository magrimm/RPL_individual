/*
 * localization_processor.cpp
 *
 *  Created on: Dec 4, 2015
 *      Author: marius
 */

#include <localization_processor.h>

localization_processor::localization_processor (ros::NodeHandle nodehandle, parameter_bag params_bag)
{
	nh = nodehandle;
	parameter = params_bag;
	// First run odometry callback
	sig_odom = true;
	sig_scan = false;

	// Create a ROS publisher
	pub = nh.advertise<geometry_msgs::PoseArray> (parameter.pub_topic_particles,
							  	  	  	  	  	  parameter.queue_size_pub_particles,
												  true);

	// Initialize robot control, initial and current pose
	pose_empty.position.x = parameter.pose_empty_pos_x;
	pose_empty.position.y = parameter.pose_empty_pos_y;
	pose_empty.position.z = parameter.pose_empty_pos_z;
	pose_empty.orientation.x = parameter.pose_empty_orient_x;
	pose_empty.orientation.y = parameter.pose_empty_orient_y;
	pose_empty.orientation.z = parameter.pose_empty_orient_z;
	pose_empty.orientation.w = parameter.pose_empty_orient_w;

	control.odometry[0].position.x = parameter.control_odom_prev_pos_x;
	control.odometry[0].position.y = parameter.control_odom_prev_pos_y;
	control.odometry[0].position.z = parameter.control_odom_prev_pos_z;
	control.odometry[0].orientation.x = parameter.control_odom_prev_orient_x;
	control.odometry[0].orientation.y = parameter.control_odom_prev_orient_y;
	control.odometry[0].orientation.z = parameter.control_odom_prev_orient_z;
	control.odometry[0].orientation.w = parameter.control_odom_prev_orient_w;

	control.odometry[1].position.x = parameter.control_odom_cur_pos_x;
	control.odometry[1].position.y = parameter.control_odom_cur_pos_y;
	control.odometry[1].position.z = parameter.control_odom_cur_pos_z;
	control.odometry[1].orientation.x = parameter.control_odom_cur_orient_x;
	control.odometry[1].orientation.y = parameter.control_odom_cur_orient_y;
	control.odometry[1].orientation.z = parameter.control_odom_cur_orient_z;
	control.odometry[1].orientation.w = parameter.control_odom_cur_orient_w;
}

void localization_processor::Callback_map (const nav_msgs::OccupancyGrid::ConstPtr& map_msg)
{
	// Save map data and info
	map.height = map_msg->info.height;
	map.width = map_msg->info.width;
	map.resolution = map_msg->info.resolution;

	// Save the map.data as integer 0 or 100
	for (int i=0; i<map_msg->data.size(); ++i)
	{
		map.data.push_back(map_msg->data.at(i));
	}

	// Create the set of particles
	get_particles();

	// Initialize
	for (int i = 0; i < particles.size(); ++i)
	{
		control_vec.push_back(control);
	}
}

void localization_processor::Callback_odom (const nav_msgs::OdometryConstPtr& odom_msg)
{
	// After call set sig_odom == false to sequentially synchronize the callback on
	// odometry and the laser scan
	while (sig_odom == true)
	{
		// Update previous odometry and get new current odometry
		control.odometry[0] = control.odometry[1];
		control.odometry[1].position.x = odom_msg->pose.pose.position.x;
		control.odometry[1].position.y = odom_msg->pose.pose.position.y;
		control.odometry[1].position.z = odom_msg->pose.pose.position.z;
		control.odometry[1].orientation.x = odom_msg->pose.pose.orientation.x;
		control.odometry[1].orientation.y = odom_msg->pose.pose.orientation.y;
		control.odometry[1].orientation.z = odom_msg->pose.pose.orientation.z;
		control.odometry[1].orientation.w = odom_msg->pose.pose.orientation.w;

		// Set signaler such that callbacks of odom and scan wait for each other
		sig_scan = true;
		sig_odom = false;
	}
}

void localization_processor::Callback_scan (const sensor_msgs::LaserScanConstPtr& scan_msg)
{
	// After call set sig_scan == false to sequentially synchronize the callback on
	// odometry and the laser scan
	while (sig_scan == true)
	{
		// Wait unitl odometry first changed
		if ((control.odometry[0].position.x == control.odometry[1].position.x) && (control.odometry[0].position.y == control.odometry[1].position.y))
		{
			sig_scan = false;
			sig_odom = true;
			break;
		}

		// Construct motion_update
		motion_update motion_upd(parameter.motion_update);

		//Construct sensor_update class
		sensor_update sensor_upd(parameter.sensor_update);

		// Construct resample
		roulette_sampling roulette_samp(parameter.resample);

		// Clear the weights
		weights.clear();

		for (int i = 0; i < particles.size(); ++i)
		{
			// Update motion
			motion_upd.particle_motion(control, particles.at(i));

			// Get weights of particles
			weights.push_back(sensor_upd.get_particle_weight(scan_msg, particles.at(i), map));
		}

		// Sum of weights
		float sum_of_weights = std::accumulate(weights.begin(), weights.end(), 0.0);

		// Normalize weights
		std::transform(weights.begin(), weights.end(), weights.begin(), std::bind1st(std::multiplies<float>(), (1/sum_of_weights)));

		// Resample the particles
		roulette_samp.resample_distribution(particles, weights);

		// Construct visualization class
		visualization vis(parameter.visualization);

		// Create particle visualization
		geometry_msgs::PoseArray::Ptr pose_array (new geometry_msgs::PoseArray);
		vis.visualize_particle_pose(pose_array, particles);

		// Publish the marker array of the particles
		pub.publish(pose_array);

		// Set signaler such that callbacks of odom and scan wait for each other
		sig_scan = false;
		sig_odom = true;
	}
}

void localization_processor::get_particles ()
{
	int count;
	// Put particles in x-coordinate in each it_cell_x cell
	for (int i = 0; i < map.width; i += parameter.it_cell_x)
	{
		// Put particles in y-coordinate in each it_cell_y cell
		for (int j = 0; j < map.height; j += parameter.it_cell_y)
		{
			// Distribute particles with different theta orientation
			for (float theta = 0; theta < 2*M_PI; theta += 2*M_PI/parameter.it_theta)
			{
				if (map.data.at(i+j*map.width) == 0)
				{
					pose a_particle;

					// Get particle position from grid
					a_particle.position.x = i*map.resolution;
					a_particle.position.y = j*map.resolution;
					a_particle.position.z = 0;

					// Convert theta to quaternion
					a_particle.orientation.x = tf::createQuaternionFromYaw(theta).getX();
					a_particle.orientation.y = tf::createQuaternionFromYaw(theta).getY();
					a_particle.orientation.z = tf::createQuaternionFromYaw(theta).getZ();
					a_particle.orientation.w = tf::createQuaternionFromYaw(theta).getW();

					// Add particle to set of particles
					particles.push_back(a_particle);

					// Count number of particles
					count += 1;
				}
			}
		}
	}
}


