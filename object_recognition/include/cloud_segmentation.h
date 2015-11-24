/*
 * cloud_segmentation.h
 *
 *  Created on: Nov 24, 2015
 *      Author: marius
 */

#ifndef CLOUD_SEGMENTATION_H
#define CLOUD_SEGMENTATION_H

#include <parameter/segmentation_bag.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

class cloud_segmentation
{
public:
	cloud_segmentation (segmentation_bag segment_param);
	void euclidean_cluster_extraction (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered,
										 std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>* vector_of_cloud_cluster);
private:
	segmentation_bag segment_param;
};

#endif /* CLOUD_SEGMENTATION_H */
