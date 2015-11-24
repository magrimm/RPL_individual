/*
 * outlier_filter.h
 *
 *  Created on: Nov 24, 2015
 *      Author: marius
 */

#ifndef OUTLIER_FILTER_H
#define OUTLIER_FILTER_H

struct outlier_filter
{
	int meank;
	float stddev_mul_thresh;
};

#endif /* OUTLIER_FILTER_H */
