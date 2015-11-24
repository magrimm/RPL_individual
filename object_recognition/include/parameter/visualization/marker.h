/*
 * marker.h
 *
 *  Created on: Nov 24, 2015
 *      Author: marius
 */

#ifndef VISUALIZATION_MARKER_H
#define VISUALIZATION_MARKER_H

#include <string>

struct marker_visualization
{
	std::string frame_id, ns, mesh_resource;
	float orientation_x, orientation_y, orientation_z, orientation_w;
	float color_alpha, color_r, color_g, color_b;
};

#endif /* VISUALIZATION_MARKER_H */
