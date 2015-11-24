/*
 * passthrough_filter.h
 *
 *  Created on: Nov 24, 2015
 *      Author: marius
 */

#ifndef PASSTHROUGH_FILTER_H
#define PASSTHROUGH_FILTER_H

struct passthrough_filter
{
	float min_filterlimit_x, min_filterlimit_y, min_filterlimit_z;
	float max_filterlimit_x, max_filterlimit_y, max_filterlimit_z;
};

#endif /*PASSTHROUGH_FILTER_H */
