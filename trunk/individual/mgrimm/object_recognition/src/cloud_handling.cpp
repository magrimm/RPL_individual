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
	std::vector<std::vector<pcl::PointCloud<pcl::Histogram<153> > > > object_database_spin_images;

	// Create a ROS publisher for the output point cloud
	pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ> > (parameter.pub_topic_pointcloud,
														 parameter.queue_size_pub_pointcloud,
														 true);
	// Create a ROS publisher for the marker visualization
	vis_pub = nh.advertise<visualization_msgs::Marker>(parameter.pub_topic_marker,
													   parameter.queue_size_pub_marker,
													   true);
}

void cloud_handling::features_of_objects ()
{
	// Define Path of object .txt file
	std::ifstream inFile_object_files (parameter.object_files_path.c_str());

	// Create pointcloud objects and the referring database
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sampled (new pcl::PointCloud<pcl::PointXYZ>);
	std::vector<pcl::PointCloud<pcl::PointXYZ> > a_vector;
	std::vector<std::vector<pcl::PointCloud<pcl::PointXYZ> > > object_database;

	// Create spin_image objects and the referring database
	pcl::PointCloud<pcl::Histogram<153> >::Ptr spin_images (new pcl::PointCloud<pcl::Histogram<153> >);
	std::vector<pcl::PointCloud<pcl::Histogram<153> > > a_vector_spin_images;

	// Scanning all pcd-files which are going to be converted to a pointcloud
	for (std::string line_object_files; std::getline(inFile_object_files, line_object_files);)
	{
		std::cout << "\nLine_object_files: "
				  << boost::filesystem::current_path().string()+line_object_files
				  << std::endl;

		line_object_files = boost::filesystem::current_path().string()+line_object_files;
		std::ifstream inFile_line_object_files (line_object_files.c_str());
		for (std::string line_object_pcdfiles; std::getline(inFile_line_object_files, line_object_pcdfiles);)
		{
			std::cout << "Line_object_pcdfiles: "
					  << boost::filesystem::current_path().string()+line_object_pcdfiles
					  << std::endl;

			// Convert the .pcd file to pointcloud data and print Error if file not readable
			if (pcl::io::loadPCDFile<pcl::PointXYZ> (boost::filesystem::current_path().string()+line_object_pcdfiles, *cloud) == -1)
			{
		    std::cout << "Couldn't read file "
		    		  << boost::filesystem::current_path().string()+line_object_pcdfiles
					  << "\n";
			}
			// Construct the class cloud_matching with its parameters.
			cloud_matching c_matching (parameter.recognition);

			// Convert Pointcloud to spin_image
			c_matching.spin_image(cloud, spin_images);

			// Push back Pointclouds/spin_images in vector a_vector/a_vector_spin_images
			a_vector.push_back (*cloud);
			a_vector_spin_images.push_back (*spin_images);
		}

		// Push back vector with pointclouds/vector with spin_images of each object
		// in object_database/object_database_spin_images
		object_database.push_back(a_vector);
		object_database_spin_images.push_back(a_vector_spin_images);

		// Empty the vector a_vector and a_vector_spin_images thus they can be filled again
		a_vector.clear();
		a_vector_spin_images.clear();
	}
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
	// Apply euclidean cluster extraction (segmentation) and return vector of cloud cluster
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>* cloud_cluster (new std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>);
	c_segmentation.euclidean_cluster_extraction(cloud_filtered_outlier, cloud_cluster);

	// Publish the data
	pub.publish (cloud_filtered_outlier);

	// Construct the class cloud_matching with its parameters
	cloud_matching c_matching (parameter.recognition);

	// Get spin_images of clusters of pointclouds and visualize the markers for each cluster
	std::vector<pcl::PointCloud<pcl::Histogram<153> >::Ptr> cluster_spin_images;
	for (int i=0; i<cloud_cluster->size(); ++i)
	{
		  // Get spin_images of clusters of pointclouds
		  pcl::PointCloud<pcl::Histogram<153> >::Ptr spin_images (new pcl::PointCloud<pcl::Histogram<153> >);
		  c_matching.spin_image ((*cloud_cluster)[i], spin_images);

		  // Save spin_images of the cluster in cluster_spin_images[i]
		  cluster_spin_images.push_back(spin_images);

		  // Compare the spin_images of the object with the spin_images of the clusters
		  std::vector<std::vector<float> > R_database(object_database_spin_images.size(), std::vector<float>(object_database_spin_images[i].size()));
		  std::vector<float> R_object;

		  // Get correlation values of current scene with each object (in R_object)
		  for (int j=0; j<object_database_spin_images.size(); ++j)
		  {
			  for (int k=0; k<object_database_spin_images[i].size(); ++k)
			  {
				  R_database[j][k] = c_matching.correlation_cloud(object_database_spin_images[j][k], cluster_spin_images[i]);
			  }
			  R_object.push_back(std::accumulate(R_database[j].begin(), R_database[j].end(), 0.0)/R_database[j].size());
			  std::cout << "Average_R_object_" << j+1 << " : "
						<< R_object[j]
						<< "\n";
		  }

		  // Print R values for objects
		  float biggest_R =-std::numeric_limits<float>::infinity();
		  for (std::vector<float>::const_iterator l = R_object.begin(); l != R_object.end(); ++l)
		  {
			  if (*l > biggest_R)
			  {
			      biggest_R = *l;
		      }
		  }
		  std:: cout << "| biggest_R: " << biggest_R << " |" << std::endl;

		  // Create a marker for each cluster
		  visualization_msgs::Marker::Ptr marker (new visualization_msgs::Marker);

		  // Construct class cloud_visualization
		  cloud_visualization c_visualization (parameter.visualization);

		  // Give different colours to the markers depending on which object is recognised
		  if (biggest_R < parameter.recognition.correlation.correlation_thresh)
		  {
			  c_visualization.visualize_marker((*cloud_cluster)[i], marker, i, parameter.visualization.marker.color_r_unknown,
					  	  	  	  	  	  	  	  	  	  	  	  	  	  	   parameter.visualization.marker.color_g_unknown,
																			   parameter.visualization.marker.color_b_unknown);
		  }
		  else if (biggest_R == R_object[0])
		  {
			  c_visualization.visualize_marker((*cloud_cluster)[i], marker, i, parameter.visualization.marker.color_r_duck,
					  	  	  	  	  	  	  	  	  	  	  	  	  	  	   parameter.visualization.marker.color_g_duck,
																			   parameter.visualization.marker.color_b_duck);
		  }
		  else
		  {
			  c_visualization.visualize_marker((*cloud_cluster)[i], marker, i, parameter.visualization.marker.color_r_human,
					  	  	  	  	  	  	  	  	  	  	  	  	  	  	   parameter.visualization.marker.color_g_human,
																			   parameter.visualization.marker.color_b_human);
		  }

		  // Publish the marker
		  vis_pub.publish(marker);
		  // Publish the data
		  pub.publish (cloud_filtered_outlier);
	}
}
