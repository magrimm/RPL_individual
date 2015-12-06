/*
 * motion_update.cpp
 *
 *  Created on: Dec 5, 2015
 *      Author: marius
 */

#include <motion_update.h>

motion_update::motion_update(motion_update_bag motion_update_parameter)
{
	motion_update_params = motion_update_parameter;
}

void motion_update::particle_motion(robot_control control, pose& particle)
{
//	std::cout << "control_vec[i]: " << control.odometry[1].position.x << " | "
//									<< control.odometry[1].position.y << " | "
//									<< control.odometry[1].position.z
//									<< std::endl;

	// Get euler angles from quaternion for control
	tf::Quaternion q0(control.odometry[0].orientation.x,
					  control.odometry[0].orientation.y,
					  control.odometry[0].orientation.z,
					  control.odometry[0].orientation.w);
	tf::Matrix3x3 m0(q0);
	double roll0, pitch0, yaw0;
	m0.getRPY(roll0, pitch0, yaw0);

	tf::Quaternion q1(control.odometry[1].orientation.x,
					  control.odometry[1].orientation.y,
					  control.odometry[1].orientation.z,
					  control.odometry[1].orientation.w);
	tf::Matrix3x3 m1(q1);
	double roll1, pitch1, yaw1;
	m1.getRPY(roll1, pitch1, yaw1);

	// Recover relative motion parameters from control
	float delta_rot1 = atan2f(control.odometry[1].position.y - control.odometry[0].position.y, control.odometry[1].position.x - control.odometry[0].position.x) - yaw0;
	float delta_trans = sqrtf(pow(control.odometry[0].position.x - control.odometry[1].position.x, 2) + pow(control.odometry[0].position.y - control.odometry[1].position.y, 2));
	float delta_rot2 = yaw1 - yaw0 - delta_rot1;

	// Construct random distributions class
	normal_distribution_approximation norm_dist_approx;

	// Perturb the motion parameters by noise in robot motion
	float delta_rot1_perturbed = delta_rot1 - norm_dist_approx.sample_distribution((motion_update_params.alpha1 * pow(delta_rot1, 2)) +
																				   (motion_update_params.alpha2 * pow(delta_trans, 2)));
	float delta_trans_perturbed = delta_trans - norm_dist_approx.sample_distribution((motion_update_params.alpha3 * pow(delta_trans, 2)) +
																					 (motion_update_params.alpha4 * pow(delta_rot1, 2)) +
																					 (motion_update_params.alpha4 * pow(delta_rot2, 2)));
	float delta_rot2_perturbed = delta_rot2 - norm_dist_approx.sample_distribution((motion_update_params.alpha1 * pow(delta_rot2, 2)) +
																				   (motion_update_params.alpha2 * pow(delta_trans, 2)));

//	std::cout << "delta_rot1: " << delta_rot1 << " | "
//			  << "delta_rot1_p: " << delta_rot1_perturbed << " | "
//			  << "delta_trans: " << delta_trans << " | "
//			  << "delta_trans_p: " << delta_trans_perturbed << " | "
//			  << "delta_rot2: " << delta_rot2 << " | "
//			  << "delta_rot2_p: " << delta_rot2_perturbed
//			  << std::endl;

	// Get euler angles from quaternion for initial_pose
	tf::Quaternion q_ini(particle.orientation.x,
			particle.orientation.y,
			particle.orientation.z,
			particle.orientation.w);
	tf::Matrix3x3 m_ini(q_ini);
	double roll_ini, pitch_ini, yaw_ini;
	m_ini.getRPY(roll_ini, pitch_ini, yaw_ini);

//	std::cout << "Roll_ini: " << roll_ini << " | Pitch_ini: " << pitch_ini << " | Yaw_ini: " << yaw_ini << std::endl;
//	std::cout << "initial_pose[i]: " << particle.position.x << " | "
//									<< particle.position.y << " | "
//									<< particle.position.z
//									<< std::endl;

	// Update the output pose x_t using the sample motion parameters
	float x = particle.position.x + delta_trans_perturbed * cosf(yaw_ini + delta_rot1_perturbed);
	float y = particle.position.y + delta_trans_perturbed * sinf(yaw_ini + delta_rot1_perturbed);
	float theta = yaw_ini + delta_rot1_perturbed + delta_rot2_perturbed;

	particle.position.x = x;
	particle.position.y = y;
	// Convert back angle to quaternion
	particle.orientation.x = tf::createQuaternionFromYaw(theta).getX();
	particle.orientation.y = tf::createQuaternionFromYaw(theta).getY();
	particle.orientation.z = tf::createQuaternionFromYaw(theta).getZ();
	particle.orientation.w = tf::createQuaternionFromYaw(theta).getW();
}


