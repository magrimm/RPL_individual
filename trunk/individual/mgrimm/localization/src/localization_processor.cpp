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
	pose_empty.position.x = 0;
	pose_empty.position.y = 0;
	pose_empty.position.z = 0;
	pose_empty.orientation.x = 0;
	pose_empty.orientation.y = 0;
	pose_empty.orientation.z = 0;
	pose_empty.orientation.w = 1.0;

	control.odometry[0].position.x = 0;
	control.odometry[0].position.y = 0;
	control.odometry[0].position.z = 0;
	control.odometry[0].orientation.x = 0;
	control.odometry[0].orientation.y = 0;
	control.odometry[0].orientation.z = 0;
	control.odometry[0].orientation.w = 1.0;

	control.odometry[1].position.x = 0;
	control.odometry[1].position.y = 0;
	control.odometry[1].position.z = 0;
	control.odometry[1].orientation.x = 0;
	control.odometry[1].orientation.y = 0;
	control.odometry[1].orientation.z = 0;
	control.odometry[1].orientation.w = 1.0;
}

void localization_processor::Callback_map (const nav_msgs::OccupancyGrid::ConstPtr& map_msg)
{
	// Save the map
	map = map_msg;

	// Save the map.data as integer 0 or 100
	for (int i=0; i<map_msg->data.size(); ++i)
	{
		map_data.push_back(map_msg->data.at(i));
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
	while (sig_odom == true)
	{
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
	while (sig_scan == true)
	{
		// Only start when odometry changes
		if ((control.odometry[0].position.x == control.odometry[1].position.x) && (control.odometry[0].position.y == control.odometry[1].position.y))
		{
			sig_scan = false;
			sig_odom = true;
			break;
		}

		// Construct motion_update
		motion_update motion_upd(parameter.motion_update);

		//Construct sensor_update class
		sensor_update sensor_upd(parameter.sensor_update, map_data);

		// Construct resample
		roulette_sampling roulette_samp(parameter.resample);

		// Clear the weights
		weights.clear();

		// Get weights of particles
		for (int i = 0; i < particles.size(); ++i)
		{
			// Update motion
			motion_upd.particle_motion(control, particles.at(i));

			weights.push_back(sensor_upd.get_particle_weight(scan_msg, particles.at(i)));
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
	for (int i = 0; i < 200; i += 10)
	{
		for (int j = 0; j < 200; j += 10)
		{
			for (float theta = 0; theta < 2*M_PI; theta += M_PI_4/2)
			{
				if (map_data.at(i+j*200) == 0)
				{
					pose a_particle;

					float x = 0.01*i;
					float y = 0.01*j;

					a_particle.position.x = x;
					a_particle.position.y = y;
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

					// TESTPRINT
					std::cout << "QUAT_TEST_x: " << a_particle.orientation.x
							  << " QUAT_TEST_y: " << a_particle.orientation.y
							  << " QUAT_TEST_z: " << a_particle.orientation.z
							  << " QUAT_TEST_w: " << a_particle.orientation.w
							  << std::endl;
				}
			}
		}
	}
}


