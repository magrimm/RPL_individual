/*
 * recognition_bag.h
 *
 *  Created on: Nov 24, 2015
 *      Author: marius
 */

#ifndef RECOGNITION_BAG_H
#define RECOGNITION_BAG_H

#include <parameter/recognition/cloud_correlation.h>

struct recognition_bag
{
	cloud_correlation correlation;
	int hist_size;
};

#endif /* RECOGNITION_BAG_H */
