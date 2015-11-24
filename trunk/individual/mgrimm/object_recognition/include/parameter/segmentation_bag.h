/*
 * segmentation_bag.h
 *
 *  Created on: Nov 24, 2015
 *      Author: marius
 */

#ifndef SEGMENTATION_BAG_H
#define SEGMENTATION_BAG_H

#include <parameter/segmentation/euclidean_cluster_segmentation.h>

struct segmentation_bag
{
	euclidean_cluster_segmentation euclidean_cluster;
};

#endif /* SEGMENTATION_BAG_H */
