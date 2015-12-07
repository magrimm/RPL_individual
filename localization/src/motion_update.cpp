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
	float delta_rot1 = float (atan2f(control.odometry[1].position.y - control.odometry[0].position.y, control.odometry[1].position.x - control.odometry[0].position.x) - float (yaw0));
	float delta_trans = float (sqrtf(powf(control.odometry[0].position.x - control.odometry[1].position.x, 2) + powf(control.odometry[0].position.y - control.odometry[1].position.y, 2)));
	float delta_rot2 = float (yaw1 - yaw0 - delta_rot1);

	// Construct random distributions class
	normal_distribution_approximation norm_dist_approx;

	// Perturb the motion parameters by noise in robot motion
	float rot1_sq = (motion_update_params.alpha1 * powf(delta_rot1, 2)) +
			        (motion_update_params.alpha2 * powf(delta_trans, 2));
	float trans_sq = (motion_update_params.alpha3 * powf(delta_trans, 2)) +
					 (motion_update_params.alpha4 * powf(delta_rot1, 2)) +
					 (motion_update_params.alpha4 * powf(delta_rot2, 2));
	float rot2_sq = (motion_update_params.alpha1 * powf(delta_rot2, 2)) +
 	   	   	   	   	(motion_update_params.alpha2 * powf(delta_trans, 2));

	float sample_rot1 = norm_dist_approx.sample_distribution(rot1_sq);
	float sample_trans = norm_dist_approx.sample_distribution(trans_sq);
	float sample_rot2 = norm_dist_approx.sample_distribution(rot2_sq);

	float delta_rot1_perturbed = delta_rot1 - sample_rot1;
	float delta_trans_perturbed = delta_trans - sample_trans;
	float delta_rot2_perturbed = delta_rot2 - sample_rot2;

	// Get euler angles from quaternion for initial_pose
	tf::Quaternion q_ini(particle.orientation.x,
			particle.orientation.y,
			particle.orientation.z,
			particle.orientation.w);
	tf::Matrix3x3 m_ini(q_ini);
	double roll_ini, pitch_ini, yaw_ini;
	m_ini.getRPY(roll_ini, pitch_ini, yaw_ini);

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


