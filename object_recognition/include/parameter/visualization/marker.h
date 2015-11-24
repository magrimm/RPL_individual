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
	float color_alpha;
	float color_r_duck, color_g_duck, color_b_duck;
	float color_r_human, color_g_human, color_b_human;
	float color_r_unknown, color_g_unknown, color_b_unknown;
};

#endif /* VISUALIZATION_MARKER_H */
