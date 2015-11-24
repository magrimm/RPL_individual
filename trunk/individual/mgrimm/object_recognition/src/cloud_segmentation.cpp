/*
 * cloud_segmentation.cpp
 *
 *  Created on: Nov 24, 2015
 *      Author: marius
 */

#include <cloud_segmentation.h>

cloud_segmentation::cloud_segmentation (segmentation_bag seg_bag)
{
	segment_param = seg_bag;
}

void euclidean_cluster_extraction (pcl::PointCloud<pcl::PointXYZ>::Ptr a_cloud_filtered, std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>* a_vector_of_cloud_cluster)
{
    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (a_cloud_filtered);

    // Seperate clusters by distance
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.02);
    // Filter too small or too big clusters
    ec.setMinClusterSize (10);
    ec.setMaxClusterSize (100000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (a_cloud_filtered);
    ec.extract (cluster_indices);

    std::cout << "Number of Clusters: " << cluster_indices.size() << std::endl;

    int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
            cloud_cluster->points.push_back (a_cloud_filtered->points[*pit]);
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        std::cout << " PointCloud representing the Cluster: " << cloud_cluster->points.size ()
        		  << " data points." << std::endl;

        a_vector_of_cloud_cluster->push_back(cloud_cluster);

        ++j;
    }
}


