/*
 * cloud_matching.cpp
 *
 *  Created on: Nov 24, 2015
 *      Author: marius
 */

#include <cloud_matching.h>

cloud_matching::cloud_matching (recognition_bag rec_bag)
{
	recog_param = rec_bag;
}

void cloud_matching::spin_image (pcl::PointCloud<pcl::PointXYZ>::Ptr a_cloud, pcl::PointCloud<pcl::Histogram<153> >::Ptr spin_images)
{
	// Compute the normals
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
	normal_estimation.setInputCloud (a_cloud);

	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree (new pcl::search::KdTree<pcl::PointXYZ>);
	normal_estimation.setSearchMethod (kdtree);

	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud< pcl::Normal>);
	normal_estimation.setRadiusSearch (0.05);
	normal_estimation.compute (*normals);

	// Setup spin image computation
	pcl::SpinImageEstimation<pcl::PointXYZ, pcl::Normal, pcl::Histogram<153> > spin_image_descriptor(8, 0.5, 16);
	spin_image_descriptor.setInputCloud (a_cloud);
	spin_image_descriptor.setInputNormals (normals);

	// Use the same KdTree from the normal estimation
	spin_image_descriptor.setSearchMethod (kdtree);
	spin_image_descriptor.setRadiusSearch (0.2);

	// Actually compute the spin images
	spin_image_descriptor.compute (*spin_images);
}

float cloud_matching::correlation_image (pcl::Histogram<153> P, pcl::Histogram <153> Q)
{
	int hist_size = 153;
	float sum_P = std::accumulate(P.histogram, P.histogram + hist_size, 0.0);
	float sum_Q = std::accumulate(Q.histogram, Q.histogram + hist_size, 0.0);
	float sum_PQ = std::inner_product(P.histogram, P.histogram + hist_size, Q.histogram, 0.0);
	float sum_sqr_P = std::inner_product(P.histogram, P.histogram + hist_size, P.histogram, 0.0);
	float sum_sqr_Q = std::inner_product(Q.histogram, Q.histogram + hist_size, Q.histogram, 0.0);

	float R = ((hist_size*sum_PQ)-(sum_P*sum_Q))/std::sqrt(((hist_size*sum_sqr_P)-std::pow(sum_P, 2))*
														((hist_size*sum_sqr_Q)-std::pow(sum_Q, 2)));

	return R;
}

float cloud_matching::correlation_cloud (pcl::PointCloud<pcl::Histogram<153> > &a_spin_images1, pcl::PointCloud<pcl::Histogram<153> >::Ptr a_spin_images2)
{
	std::vector<float> R_vector;
	int histo_size = 153, count_corr;
	for (int i=0; i<a_spin_images1.points.size(); ++i)
	{
		float R=-std::numeric_limits<float>::infinity();
		for (int j=0; j<a_spin_images2->points.size(); ++j)
		{
			float R_temp;
			// Calculate correlation value R between two points in different pointclouds
			R_temp = correlation_image(a_spin_images1.points[i], a_spin_images2->points[j]);

			if (R_temp > R)
			{
				R = R_temp;
			}
		}
		R_vector.push_back(R);
	}
	// Calculate the mean of the R values for the point cloud s
	float sum_R = std::accumulate(R_vector.begin(), R_vector.end(), 0.0);
	float R_tot = (sum_R / R_vector.size());

	return R_tot;
}
