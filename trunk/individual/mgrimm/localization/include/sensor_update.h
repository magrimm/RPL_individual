/*
 * sensor_update.h
 *
 *  Created on: Dec 6, 2015
 *      Author: marius
 */

#ifndef _SENSOR_UPDATE_H_
#define _SENSOR_UPDATE_H_

#include <parameter/sensor_update_bag.h>
#include <sensor_msgs/LaserScan.h>
#include <localization_processor.h>

// Forward declaration
struct pose;
struct position3D;

class sensor_update
{
public:
	sensor_update (sensor_update_bag sensor_update_param, std::vector<int> map_data);
	float get_particle_weight (const sensor_msgs::LaserScanConstPtr& a_scan_msg, pose& a_particle);
	void convert_sensor_measurement_to_points (const sensor_msgs::LaserScanConstPtr& a_scan_msg, pose& particle, std::vector<position3D>& a_points);
	int correlation_particle_map (std::vector<position3D>& a_points);

private:
	sensor_update_bag sensor_update_param;
	std::vector<int> map_data;
};

#endif /* _SENSOR_UPDATE_H_ */
