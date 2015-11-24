/*
 * cloud_handling.cpp
 *
 *  Created on: Nov 23, 2015
 *      Author: marius
 */

#include <cloud_handling.h>

cloud_handling::cloud_handling (ros::NodeHandle nodehandle, parameter_bag params_bag)
{
	nh = nodehandle;
	parameter = params_bag;
	ros::Publisher pub, vis_pub;
	std::vector<std::vector<pcl::PointCloud<pcl::Histogram<153> > > > object_database_spin_images;
}

void cloud_handling::Match ()
{
	ros::Subscriber sub = nh.subscribe(parameter.subscribed_rostopic, parameter.queue_size_subscriber, &cloud_handling::Callback, this);
}

void cloud_handling::features_of_objects ()
{
//	// Define Path of object .txt file
//	std::string object_files_path = "/home/marius/catkin_ws/src/mgrimm/object_recognition/object_files.txt";
//	std::ifstream inFile_object_files (object_files_path.c_str());
//	// Create pointcloud objects and the referring database
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sampled (new pcl::PointCloud<pcl::PointXYZ>);
//	std::vector<pcl::PointCloud<pcl::PointXYZ> > a_vector;
//	std::vector<std::vector<pcl::PointCloud<pcl::PointXYZ> > > object_database;
//	// Create spin_image objects and the referring database
//	pcl::PointCloud<pcl::Histogram<153> >::Ptr spin_images (new pcl::PointCloud<pcl::Histogram<153> >);
//	std::vector<pcl::PointCloud<pcl::Histogram<153> > > a_vector_spin_images;
//
//	// Printing all pcd-files which are going to be converted to a pointcloud
//	for (std::string line_object_files; std::getline(inFile_object_files, line_object_files);)
//	{
//		std::cout << "Line_object_files: "
//				  << line_object_files
//				  << std::endl;
//
//		std::ifstream inFile_line_object_files (line_object_files.c_str());
//		for (std::string line_object_pcdfiles; std::getline(inFile_line_object_files, line_object_pcdfiles);)
//		{
//			std::cout << "Line_object_pcdfiles: "
//					  << line_object_pcdfiles
//					  << std::endl;
//
//			// Convert the .pcd file to pointcloud data.
//			if (pcl::io::loadPCDFile<pcl::PointXYZ> (line_object_pcdfiles, *cloud) == -1)
//			{
//		    std::cout << "Couldn't read file "
//		    		  << line_object_pcdfiles
//					  << "\n";
//			}
//
//			// Convert Pointcloud to spin_image
//			spin_image(cloud, spin_images);
//			// Push back Pointclouds/spin_images in vector a_vector/a_vector_spin_images
//			a_vector.push_back (*cloud);
//			a_vector_spin_images.push_back (*spin_images);
//		}
//		// Push back vector with pointclouds/vector with spin_images of each object
//		// in object_database/object_database_spin_images
//		object_database.push_back(a_vector);
//		object_database_spin_images.push_back(a_vector_spin_images);
//		// Empty the vector a_vector and a_vector_spin_images thus they can be filled again
//		a_vector.clear();
//		a_vector_spin_images.clear();
//	}
}

void cloud_handling::Callback (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
	// Container for original & filtered data
	pcl::PCLPointCloud2* temp_cloud (new pcl::PCLPointCloud2);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_resolution (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_passthrough (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered_outlier (new pcl::PointCloud<pcl::PointXYZ>);

	// Convert from sensor_msgs to PCLPointCloud2 (PCL data type)
	pcl_conversions::toPCL(*cloud_msg, *temp_cloud);
	// Convert from PCLPointCloud2 to PointCloud
	pcl::fromPCLPointCloud2(*temp_cloud, *cloud);

	// Construct the class cloud_filter with its parameters
	cloud_filter c_filter(parameter.filter);
	// Apply resolution_filter
	c_filter.resolution_filter(cloud, cloud_filtered_resolution);

	// Apply passthrough_filter
	c_filter.passthrough_filter(cloud_filtered_resolution, cloud_filtered_passthrough);

	// Apply statistical_outlier_removal_filter
	c_filter.statistical_outlier_removal_filter(cloud_filtered_passthrough, cloud_filtered_outlier);

	// Construct the class cloud_segmentation with its parameters
	cloud_segmentation c_segmentation (parameter.segmentation);
	// Apply euclidean cluster extraction and return vector of cloud cluster
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>* cloud_cluster (new std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>);
	c_segmentation.euclidean_cluster_extraction(cloud_filtered_outlier, cloud_cluster);

	// Construct the class cloud_matching with its parameters
	cloud_matching c_matching (parameter.recognition);

//	// Get spin_images of clusters of pointclouds and visualize the markers for each cluster
//	std::vector<pcl::PointCloud<pcl::Histogram<153> >::Ptr> cluster_spin_images;
//	for (int i=0; i<cloud_cluster->size(); ++i)
//	{
//		  // Get spin_images of clusters of pointclouds
//		  pcl::PointCloud<pcl::Histogram<153> >::Ptr spin_images (new pcl::PointCloud<pcl::Histogram<153> >);
//		  spin_image ((*cloud_cluster)[i], spin_images);
//
//		  // Save spin_images of the cluster in cluster_spin_images[i]
//		  cluster_spin_images.push_back(spin_images);
//
//		  // Compare the spin_images of the object with the spin_images of the clusters
//		  std::vector<std::vector<float> > R_database(object_database_spin_images.size(), std::vector<float>(object_database_spin_images[i].size()));
//		  std::vector<float> R_object;
//
//		  std::cout << "R_database: \n";
//		  for (int j=0; j<object_database_spin_images.size(); ++j)
//		  {
//			  for (int k=0; k<object_database_spin_images[i].size(); ++k)
//			  {
//				  R_database[j][k] = correlation_cloud(object_database_spin_images[j][k], cluster_spin_images[i]);
//			  }
//			  R_object.push_back(std::accumulate(R_database[j].begin(), R_database[j].end(), 0.0)/R_database[j].size());
//			  std::cout << "Average_R_object_" << j+1 << " : "
//						<< R_object[j]
//						<< "\n";
//		  }
//		  // Print R values for objects
//		  float biggest_R =-std::numeric_limits<float>::infinity();
//		  for (std::vector<float>::const_iterator l = R_object.begin(); l != R_object.end(); ++l)
//		  {
//			  if (*l > biggest_R)
//			  {
//				  biggest_R = *l;
//			  }
//			  std::cout << *l << " ";
//		  }
//		  std:: cout << " biggest_R: " << biggest_R << std::endl;
//
//		  // Create a marker for each cluster
//		  visualization_msgs::Marker::Ptr marker (new visualization_msgs::Marker);
//		  // Give different colours to the markers depending on which object is recognised
//		  if (biggest_R < 0.93)
//		  {
//			  visualize_marker((*cloud_cluster)[i], marker, i, 1.0, 0.0, 0.0);
//		  }
//		  else if (biggest_R == R_object[0])
//		  {
//			  visualize_marker((*cloud_cluster)[i], marker, i, 0.0, 0.0, 1.0);
//		  }
//		  else
//		  {
//			  visualize_marker((*cloud_cluster)[i], marker, i, 0.0, 1.0, 0.0);
//		  }
//
//		  // Publish the marker
//		  vis_pub.publish(marker);
//
//		  // Publish the data
//		  pub.publish (cloud_filtered_outlier);
//	}
}

void cloud_handling::visualize_marker (pcl::PointCloud<pcl::PointXYZ>::Ptr a_cloud_cluster, visualization_msgs::Marker::Ptr a_marker, int a_marker_id, float color_r, float color_g, float color_b)
{
	  Eigen::Vector4f min_vector, max_vector;
	  pcl::getMinMax3D(*a_cloud_cluster, min_vector, max_vector);

	  // SET MARKER
	  a_marker->header.frame_id = "camera_depth_optical_frame";
	  a_marker->header.stamp = ros::Time();
	  a_marker->ns = "my_namespace";
	  a_marker->id = a_marker_id;
	  a_marker->type = visualization_msgs::Marker::CUBE;
	  a_marker->action = visualization_msgs::Marker::ADD;
	  a_marker->pose.position.x = (max_vector[0] + min_vector[0])/2;
	  a_marker->pose.position.y = (max_vector[1] + min_vector[1])/2;
	  a_marker->pose.position.z = (max_vector[2] + min_vector[2])/2;
	  a_marker->pose.orientation.x = 0.0;
	  a_marker->pose.orientation.y = 0.0;
	  a_marker->pose.orientation.z = 0.0;
	  a_marker->pose.orientation.w = 1.0;
	  a_marker->scale.x = std::abs(max_vector[0] - min_vector[0]);
	  a_marker->scale.y = std::abs(max_vector[1] - min_vector[1]);
	  a_marker->scale.z = std::abs(max_vector[2] - min_vector[2]);
	  a_marker->color.a = 0.5;
	  a_marker->color.r = color_r;
	  a_marker->color.g = color_g;
	  a_marker->color.b = color_b;
	  //only if using a MESH_RESOURCE marker type:
	  a_marker->mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
}
