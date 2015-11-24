/*
 * cloud_filter.h
 *
 *  Created on: Nov 24, 2015
 *      Author: marius
 */

#ifndef CLOUD_FILTER_H
#define CLOUD_FILTER_H

#include <parameter/filter_bag.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>

class cloud_filter
{
public:
	cloud_filter (filter_bag filter_param);
	void resolution_filter (pcl::PointCloud<pcl::PointXYZ>::Ptr a_cloud,
							pcl::PointCloud<pcl::PointXYZ>::Ptr a_cloud_filtered);
	void passthrough_filter (pcl::PointCloud<pcl::PointXYZ>::Ptr a_cloud,
							 pcl::PointCloud<pcl::PointXYZ>::Ptr a_cloud_filtered);
	void statistical_outlier_removal_filter (pcl::PointCloud<pcl::PointXYZ>::Ptr a_cloud,
											 pcl::PointCloud<pcl::PointXYZ>::Ptr a_cloud_filtered);
private:
	filter_bag filter_param;
};

#endif /* CLOUD_FILTER_H */
