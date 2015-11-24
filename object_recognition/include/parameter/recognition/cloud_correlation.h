/*
 * cloud_correlation.h
 *
 *  Created on: Nov 24, 2015
 *      Author: marius
 */

#ifndef CLOUD_CORRELATION_H
#define CLOUD_CORRELATION_H

struct cloud_correlation
{
	float correlation_thresh, norm_est_radius, spin_image_descriptor_radius, support_angle_cos;
	int image_width, min_pts_neighb;
};

#endif /* CLOUD_CORRELATION_H */
