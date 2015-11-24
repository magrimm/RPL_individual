/*
 * euclidean_cluster_segmentation.h
 *
 *  Created on: Nov 24, 2015
 *      Author: marius
 */

#ifndef EUCLIDEAN_CLUSTER_SEGMENTATION_H
#define EUCLIDEAN_CLUSTER_SEGMENTATION_H

struct euclidean_cluster_segmentation
{
	float cluster_tolerance; // In meters
	int min_cluster_size, max_cluster_size;
};

#endif /* EUCLIDEAN_CLUSTER_SEGMENTATION_H */
