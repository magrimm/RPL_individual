/*
 * filter_bag.h
 *
 *  Created on: Nov 24, 2015
 *      Author: marius
 */

#ifndef FILTER_BAG_H
#define FILTER_BAG_H

#include <parameter/filter/resolution_filter.h>
#include <parameter/filter/passthrough_filter.h>
#include <parameter/filter/outlier_filter.h>

struct filter_bag
{
	resolution_filter resolution;
	passthrough_filter passthrough;
	outlier_filter outlier;
};

#endif /* FILTER_BAG_H */
