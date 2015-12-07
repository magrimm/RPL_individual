/*
 * sensor_update.cpp
 *
 *  Created on: Dec 6, 2015
 *      Author: marius
 */

#include <sensor_update.h>

sensor_update::sensor_update (sensor_update_bag sensor_update_parameter, std::vector<int> the_map_data)
{
	sensor_update_param = sensor_update_parameter;
	map_data = the_map_data;
}

float sensor_update::get_particle_weight(const sensor_msgs::LaserScanConstPtr& scan_msg, pose& particle)
{
	std::vector<position3D> points;

	// Convert sensor measurement to points in global map
	convert_sensor_measurement_to_points(scan_msg, particle, points);

	// Get correlation value of particle and map
	int correlation = correlation_particle_map(points);

	float particle_weight = float(correlation);

	clean_weight_of_particle(particle_weight, particle);

	return particle_weight;
}

void sensor_update::convert_sensor_measurement_to_points (const sensor_msgs::LaserScanConstPtr& scan_msg, pose& particle, std::vector<position3D>& points)
{
	for (int i = 0; i < scan_msg->ranges.size(); ++i)
	{
		float range = scan_msg->ranges.at(i);
		float angle = scan_msg->angle_min + i*scan_msg->angle_increment;

		if (range > scan_msg->range_min && range < scan_msg->range_max)
		{
			position3D point_laser, translation, rotation, pos;

			// Calculate polar rotation of the scan point
			point_laser.x = range * cosf(angle);
			point_laser.y = range * sinf(angle);
			point_laser.z = 0.0;

			// Transform quaternion orientation of particle to theta (yaw)
			tf::Quaternion q(particle.orientation.x,
		             	 	 particle.orientation.y,
							 particle.orientation.z,
							 particle.orientation.w);

			tf::Matrix3x3 m(q);
			double roll, pitch, yaw;
			m.getRPY(roll, pitch, yaw);

			// Calculate rotational part
			rotation.x = point_laser.x * cosf((float) yaw) - point_laser.y * sinf((float) yaw);
			rotation.y = point_laser.x * sinf((float) yaw) + point_laser.y * cosf((float) yaw);

			// Calculate translational part
			translation.x = particle.position.x;
			translation.y = particle.position.y;
			translation.z = 0.0;

			// Get final coordinates of point in global map
			pos.x = rotation.x + translation.x;
			pos.y = rotation.y + translation.y;
			pos.z = 0.0;

			points.push_back(pos);

//			// DEBUG
//			std::cout << "angle: " << angle << " | "
//					  << "yaw: " << yaw
//					  << std::endl;
		}
	}
}

int sensor_update::correlation_particle_map (std::vector<position3D>& points)
{
	int correlation = 0;

	for (int i = 0; i < points.size(); ++i)
	{
		int map_index = int(points.at(i).x/0.01) + int(points.at(i).y*200/0.01);

		// Count correlations if point is within the map
		if (map_index < map_data.size())
		{
			if (map_data.at(map_index) == 100)
			{
				correlation += 1;
			}
		}
	}

	return correlation;
}

void sensor_update::clean_weight_of_particle(float& particle_weight, pose& particle)
{
	int map_index = int(particle.position.x/0.01) + int(particle.position.y*200/0.01);
//	if (map_index > 40000)
//	{
//		map_index = 40000;
//	}

	if (particle.position.x > 2.0 || particle.position.x < 0.0 || particle.position.y > 2.0 || particle.position.y < 0.0)// || map_index > 40000 || map_data.at(map_index)==0)
	{
		// Assign low correlation/ particle weight
		particle_weight = 0.0001;
	}
}

void sensor_update::clean_particle_position (pose& particle)
{
	if ((particle.position.x > 2.0 || particle.position.x < 0.0) || (particle.position.y > 2.0 || particle.position.y < 0.0))
	{
		// Assign low correlation/ particle weight

	}
}
