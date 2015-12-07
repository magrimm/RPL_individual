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
	std::cout << "MAP_SIZE: " << map->data.size() << std::endl;

	// Count number of empty cells without obstacle
	int count = 0;
	for (int i = 0; i < map->data.size(); ++i)
	{
		if (map->data.at(i) == 0)
		{
			count = count + 1;
		}
	}
	std::cout << "MAP_SIZE_WITHOUT_OBST: " << count << std::endl;

	// Create the set of particles
	get_particles();

	// Initialize
	for (int i = 0; i < particles.size(); ++i)
	{
		control_vec.push_back(control);
		ini_pose.push_back(pose_empty);
		cur_pose.push_back(pose_empty);
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

//		for (int i = 0; i < particles.size(); ++i)
//		{
//			std::cout << "|  i  | " << i << std::endl;
//
//			std::cout << "particles[i]: " << particles.at(i).position.x << " | "
//										  << particles.at(i).position.y << " | "
//										  << particles.at(i).position.z
//										  << std::endl;

//			// Control of previous and current odometry data
//			control_vec.at(i).odometry[0] = control_vec.at(i).odometry[1];
//			control_vec.at(i).odometry[1].position.x = odom_msg->pose.pose.position.x;
//			control_vec.at(i).odometry[1].position.y = odom_msg->pose.pose.position.y;
//			control_vec.at(i).odometry[1].position.z = odom_msg->pose.pose.position.z;
//			control_vec.at(i).odometry[1].orientation.x = odom_msg->pose.pose.orientation.x;
//			control_vec.at(i).odometry[1].orientation.y = odom_msg->pose.pose.orientation.y;
//			control_vec.at(i).odometry[1].orientation.z = odom_msg->pose.pose.orientation.z;
//			control_vec.at(i).odometry[1].orientation.w = odom_msg->pose.pose.orientation.w;

//			control.odometry[0] = control.odometry[1];
//			control.odometry[1].position.x = odom_msg->pose.pose.position.x;
//			control.odometry[1].position.y = odom_msg->pose.pose.position.y;
//			control.odometry[1].position.z = odom_msg->pose.pose.position.z;
//			control.odometry[1].orientation.x = odom_msg->pose.pose.orientation.x;
//			control.odometry[1].orientation.y = odom_msg->pose.pose.orientation.y;
//			control.odometry[1].orientation.z = odom_msg->pose.pose.orientation.z;
//			control.odometry[1].orientation.w = odom_msg->pose.pose.orientation.w;

//			// Construct motion_update
//			motion_update motion_upd(parameter.motion_update);
//
//			// Update motion
//			motion_upd.particle_motion(control, particles.at(i));//control_vec.at(i), particles.at(i));
//		}
//		// Construct visualization class
//		visualization vis(parameter.visualization);
//
//		// Create particle visualization
//		geometry_msgs::PoseArray::Ptr pose_array (new geometry_msgs::PoseArray);
//		vis.visualize_particle_pose(pose_array, particles);
//
//		// Publish the marker array of the particles
//		pub.publish(pose_array);

		std::cout << "-----------------------------------------------------------------------" << std::endl;

		// Set signaler such that callbacks of odom and scan wait for each other
		sig_scan = true;
		sig_odom = false;
	}
}

void localization_processor::Callback_scan (const sensor_msgs::LaserScanConstPtr& scan_msg)
{

//	--------------------------

//	//  Only start
//	if (control.odometry[0].position.x == control.odometry[1].position.x)
//	{
//		sig_scan = false;
//		sig_odom = true;
//	}
//	else
//	{
//		sig_scan = true;
//		sig_odom = false;
//	}

//	---------------------------------

	while (sig_scan == true)
	{
		std::cout << "control[0]: "
				  << control.odometry[0].position.x << " | "
				  << control.odometry[0].position.y << " | "
				  << control.odometry[0].position.z
				  << std::endl;

		std::cout << "control[1]: "
				  << control.odometry[1].position.x << " | "
				  << control.odometry[1].position.y << " | "
				  << control.odometry[1].position.z
				  << std::endl;

		if ((control.odometry[0].position.x == control.odometry[1].position.x) && (control.odometry[0].position.y == control.odometry[1].position.y))
		{
			sig_scan = false;
			sig_odom = true;
			break;
		}

		// Clear the weights
		weights.clear();

		//Construct sensor_update class
		sensor_update sensor_upd(parameter.sensor_update, map_data);

		std::cout << "POS_1" << std::endl;

		// Get weights of particles
		for (int i = 0; i < particles.size(); ++i)
		{

//			------------------------------------

			// Construct motion_update
			motion_update motion_upd(parameter.motion_update);

			// Update motion
			motion_upd.particle_motion(control, particles.at(i));//control_vec.at(i), particles.at(i));

//			-----------------------

			std::cout << "|  i  | " << i << std::endl;
			std::cout << "particles.at(" << i << ").pos_x: " << particles.at(i).position.x << std::endl;
			std::cout << "size_weights: " << weights.size() << std::endl;

			weights.push_back(sensor_upd.get_particle_weight(scan_msg, particles.at(i)));
		}

		// Sum of weights
		float sum_of_weights = std::accumulate(weights.begin(), weights.end(), 0.0);

		std::cout << "sum_weights: " << sum_of_weights << std::endl;

		// Normalize weights
		std::transform(weights.begin(), weights.end(), weights.begin(), std::bind1st(std::multiplies<float>(), (1/sum_of_weights)));

		// DEBUG
		float after_sum_of_weights = std::accumulate(weights.begin(), weights.end(), 0.0);
		std::cout << "sum_weights_after: " << after_sum_of_weights << std::endl;

		// Construct resample
		roulette_sampling roulette_samp(parameter.resample);

		// Resample the particles
		roulette_samp.resample_distribution(particles, weights);

//		----------------------
		// Construct visualization class
		visualization vis(parameter.visualization);

		// Create particle visualization
		geometry_msgs::PoseArray::Ptr pose_array (new geometry_msgs::PoseArray);
		vis.visualize_particle_pose(pose_array, particles);

		// Publish the marker array of the particles
		pub.publish(pose_array);

//		------------------------

		// Set signaler such that callbacks of odom and scan wait for each other
		sig_scan = false;
		sig_odom = true;
	}
}

void localization_processor::get_particles ()
{
	int count;
	for (int i = 0; i < 200; i += 6)
	{
		for (int j = 0; j < 200; j += 6)
		{
			if (map_data.at(i+j*200) == 0)
			{
				pose a_particle;

//				float x = (i % map->info.width)*map->info.resolution;
//				float y = ((int)i/map->info.width)*map->info.resolution;

				float x = 0.01*i;
				float y = 0.01*j;

				a_particle.position.x = x;
				a_particle.position.y = y;
				a_particle.position.z = 0;

				std::cout << "i: " << i
						  << " x_pos: " << x
						  << " y_pos: " << y
						  << std::endl;

				// Construct math_util
				math_util math;

				// Get random theta betwenn 0 and 2*pi
				float theta = math.random_uniform_sampling_positive(2*M_PI);

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


