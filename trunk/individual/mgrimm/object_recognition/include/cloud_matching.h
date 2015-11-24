/*
 * cloud_matching.h
 *
 *  Created on: Nov 24, 2015
 *      Author: marius
 */

#ifndef CLOUD_MATCHING_H
#define CLOUD_MATCHING_H

#include <parameter/recognition_bag.h>
#include <string>
#include <numeric>
#include <pcl/features/normal_3d.h>
#include <pcl/features/spin_image.h>
#include <pcl/kdtree/kdtree.h>

class cloud_matching
{
public:
	cloud_matching (recognition_bag recog_param);
	void spin_image (pcl::PointCloud<pcl::PointXYZ>::Ptr a_cloud,
									 pcl::PointCloud<pcl::Histogram<153> >::Ptr spin_images);
	float correlation_image (pcl::Histogram<153> P, pcl::Histogram <153> Q);
	float correlation_cloud (pcl::PointCloud<pcl::Histogram<153> > &a_spin_images1,
											 pcl::PointCloud<pcl::Histogram<153> >::Ptr a_spin_images2);

private:
	recognition_bag recog_param;

};

#endif /* CLOUD_MATCHING_H */
