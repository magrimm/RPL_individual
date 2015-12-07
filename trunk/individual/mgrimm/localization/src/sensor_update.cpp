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
	float particle_weight;

	std::vector<position3D> points;

	convert_sensor_measurement_to_points(scan_msg, particle, points);

	int correlation = correlation_particle_map(points);

	particle_weight = correlation;///map_data.size();

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
			position3D pos;
			pos.x = particle.position.x + range * cosf(angle);
			pos.y = particle.position.y + range * sinf(angle);
			pos.z = 0.0;

			points.push_back(pos);
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

	std::cout << "correlation: " << correlation << std::endl;

	return correlation;
}
