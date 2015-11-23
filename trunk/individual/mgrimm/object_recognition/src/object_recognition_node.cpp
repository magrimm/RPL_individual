#include </home/marius/catkin_ws/src/mgrimm/object_recognition/src/header_object_recognition.h>

ros::Publisher pub, vis_pub;
std::vector<std::vector<pcl::PointCloud<pcl::Histogram<153> > > > object_database_spin_images;

struct parameter_bag
{
	struct filter_bag;
	struct segmentation_bag;
	struct recognition_bag;
	struct visualization_bag;

	std::string subscribed_rostopic, pub_topic_pointcloud, pub_topic_marker, cloud_frame_id;
	int queue_size_subscriber;
};

struct filter_bag
{
	struct resolution_filter_bag;
	struct passthrough_filter_bag;
	struct outlier_filter_bag;
};

struct segmentation_bag
{
	struct euclidean_cluster_segmentation_bag;
};

struct recognition_bag
{

};

struct visualization_bag
{
	struct maker_bag;
};

struct resolution_filter_bag
{
	float leafsize_x, leafsize_y, leafsize_z;
};

struct passthrough_filter_bag
{
	float min_filterlimit_x, min_filterlimit_y, min_filterlimit_z;
	float max_filterlimit_x, max_filterlimit_y, max_filterlimit_z;
};

struct outlier_filter_bag
{
	int meank;
	float stddev_mul_thresh;
};

struct euclidean_cluster_segmentation_bag
{
	float cluster_tolerance; // In meters
	int min_cluster_size, max_cluster_size;
};

struct marker_bag
{
	std::string frame_id, ns, mesh_resource;
	float orientation_x, orientation_y, orientation_z, orientation_w;
	float color_alpha, color_r, color_g, color_b;
};

void resolution_filter (pcl::PointCloud<pcl::PointXYZ>::Ptr a_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr a_cloud_filtered, float a_leafsize_x, float a_leafsize_y, float a_leafsize_z)
{
  std::cerr << "\nPointCloud before res_filtering: " << a_cloud->width * a_cloud->height
       << " data points (" << pcl::getFieldsList (*a_cloud) << ").\n";

  // Create the filtering object
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud (a_cloud);
  sor.setLeafSize (a_leafsize_x, a_leafsize_y, a_leafsize_z);
  sor.filter (*a_cloud_filtered);

  std::cerr << "PointCloud after res_filtering: " << a_cloud_filtered->width * a_cloud_filtered->height
       << " data points (" << pcl::getFieldsList (*a_cloud_filtered) << ").\n";
}

void passthrough_filter (pcl::PointCloud<pcl::PointXYZ>::Ptr a_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr a_cloud_filtered)
{
 // Create the filtering object
 pcl::PassThrough<pcl::PointXYZ> pass;
 pass.setInputCloud (a_cloud);
 pass.setFilterFieldName ("z");
 pass.setFilterLimits (0.0, 0.9);
 //pass.setFilterLimitsNegative (true); //For filtering points inbetween min and max range
 pass.filter (*a_cloud_filtered);

 std::cerr << "PointCloud after passthrough_filtering: " << a_cloud_filtered->width * a_cloud_filtered->height
       << " data points (" << pcl::getFieldsList (*a_cloud_filtered) << ").\n";
}

void statistical_outlier_removal_filter (pcl::PointCloud<pcl::PointXYZ>::Ptr a_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr a_cloud_filtered)
{
 // Create the filtering object
 pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
 sor.setInputCloud (a_cloud);
 // Number of neighbors to analyze for each point is set to meank
 sor.setMeanK (10);
 // All points who have a distance larger than stddev_mul_thresh *standard deviation of the mean distance
 // to the query point will be marked as outliers and removed
 sor.setStddevMulThresh (1.0);
 sor.filter (*a_cloud_filtered);

 std::cerr << "PointCloud after outlier_filtering: " << a_cloud_filtered->width * a_cloud_filtered->height
       << " data points (" << pcl::getFieldsList (*a_cloud_filtered) << ").\n";
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

void visualize_marker (pcl::PointCloud<pcl::PointXYZ>::Ptr a_cloud_cluster, visualization_msgs::Marker::Ptr a_marker, int a_marker_id, float color_r, float color_g, float color_b)
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

void spin_image (pcl::PointCloud<pcl::PointXYZ>::Ptr a_cloud, pcl::PointCloud<pcl::Histogram<153> >::Ptr spin_images)
{
	// Compute the normals
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
	normal_estimation.setInputCloud (a_cloud);

	pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree (new pcl::search::KdTree<pcl::PointXYZ>);
	normal_estimation.setSearchMethod (kdtree);

	pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud< pcl::Normal>);
	normal_estimation.setRadiusSearch (0.10);
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
	std::cout << "SI output points.size (): " << spin_images->points.size () << std::endl;
}

float correlation_image (pcl::Histogram<153> P, pcl::Histogram <153> Q)
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

float correlation_cloud (pcl::PointCloud<pcl::Histogram<153> > &a_spin_images1, pcl::PointCloud<pcl::Histogram<153> >::Ptr a_spin_images2)
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
			if (R > 0.93)
			{
				count_corr += count_corr;
			}
		}
		R_vector.push_back(R);
	}
	// Calculate the mean of the R values for the point cloud s
	float sum_R = std::accumulate(R_vector.begin(), R_vector.end(), 0.0);
	float R_tot = (sum_R / R_vector.size());

	return R_tot;
}

void features_of_objects ()
{
	// Define Path of object .txt file 
	std::string object_files_path = "/home/marius/catkin_ws/src/mgrimm/object_recognition/object_files.txt";
	std::ifstream inFile_object_files (object_files_path.c_str()); 
	// Create pointcloud objects and the referring database
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sampled (new pcl::PointCloud<pcl::PointXYZ>);
	std::vector<pcl::PointCloud<pcl::PointXYZ> > a_vector;
	std::vector<std::vector<pcl::PointCloud<pcl::PointXYZ> > > object_database;
	// Create spin_image objects and the referring database
	pcl::PointCloud<pcl::Histogram<153> >::Ptr spin_images (new pcl::PointCloud<pcl::Histogram<153> >);
	std::vector<pcl::PointCloud<pcl::Histogram<153> > > a_vector_spin_images;
	
	// Printing all pcd-files which are going to be converted to a pointcloud
	for (std::string line_object_files; std::getline(inFile_object_files, line_object_files);)
	{
		std::cout << "Line_object_files: "
				  << line_object_files
				  << std::endl;
		
		std::ifstream inFile_line_object_files (line_object_files.c_str());
		for (std::string line_object_pcdfiles; std::getline(inFile_line_object_files, line_object_pcdfiles);)
		{
			std::cout << "Line_object_pcdfiles: "
					  << line_object_pcdfiles 
					  << std::endl;
			
			// Convert the .pcd file to pointcloud data.
			if (pcl::io::loadPCDFile<pcl::PointXYZ> (line_object_pcdfiles, *cloud) == -1)
			{
		    std::cout << "Couldn't read file "
		    		  << line_object_pcdfiles
					  << "\n";
			}

			// Downsample resolution
//			resolution_filter(cloud, cloud_sampled, 0.01, 0.01, 0.01);
			// Convert Pointcloud to spin_image
			spin_image(cloud, spin_images);
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

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)//, parameter_bag parameter)
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

  // Apply resolution_filter
  resolution_filter(cloud, cloud_filtered_resolution, 0.01, 0.01, 0.01);

  // Apply passthrough_filter
  passthrough_filter(cloud_filtered_resolution, cloud_filtered_passthrough);

  // Apply statistical_outlier_removal_filter
  statistical_outlier_removal_filter(cloud_filtered_passthrough, cloud_filtered_outlier);

  // Apply euclidean cluster extraction and return vector of cloud cluster
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>* cloud_cluster (new std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>);
  euclidean_cluster_extraction (cloud_filtered_outlier, cloud_cluster);

  // Get spin_images of clusters of pointclouds and visualize the markers for each cluster
  std::vector<pcl::PointCloud<pcl::Histogram<153> >::Ptr> cluster_spin_images;
  for (int i=0; i<cloud_cluster->size(); ++i)
  {
	  pcl::PointCloud<pcl::Histogram<153> >::Ptr spin_images (new pcl::PointCloud<pcl::Histogram<153> >);
	  spin_image ((*cloud_cluster)[i], spin_images);

	  // Save spin_images of the cluster in cluster_spin_images[i]
	  cluster_spin_images.push_back(spin_images);

	  // Compare the spin_images of the object with the spin_images of the clusters
	  std::vector<std::vector<float> > R_database(object_database_spin_images.size(), std::vector<float>(object_database_spin_images[i].size()));
	  std::vector<float> R_object;
	  std::cout << "R_database: \n";
	  for (int j=0; j<object_database_spin_images.size(); ++j)
	  {
		  for (int k=0; k<object_database_spin_images[i].size(); ++k)
		  {
			  R_database[j][k] = correlation_cloud(object_database_spin_images[j][k], cluster_spin_images[i]);
			  std::cout << " " << R_database[j][k] << "\n";
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
	  std:: cout << "biggest_R: " << biggest_R << std::endl;

	  // Create a marker for each cluster
	  visualization_msgs::Marker::Ptr marker (new visualization_msgs::Marker);
	  // Give different colours to the markers depending on which object is recognised
	  if (biggest_R < 0.93)
	  {
		  visualize_marker((*cloud_cluster)[i], marker, i, 1.0, 0.0, 0.0);
	  }
	  else if (biggest_R == R_object[0])
	  {
		  visualize_marker((*cloud_cluster)[i], marker, i, 0.0, 0.0, 1.0);
	  }
	  else
	  {
		  visualize_marker((*cloud_cluster)[i], marker, i, 0.0, 1.0, 0.0);
	  }

	  // Publish the marker
      vis_pub.publish(marker);

      // Publish the data
      pub.publish (cloud_filtered_outlier);
  }
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "object_recognition_node");
  ros::NodeHandle nh;

  // Get rosparam.
//  std::string subscribed_rostopic, pub_topic_pointcloud, pub_topic_marker;
//  int queue_size_subscriber;

  parameter_bag parameter;

  nh.getParam("subscribed_rostopic", parameter.subscribed_rostopic);
  nh.getParam("queue_size_subscriber", parameter.queue_size_subscriber);
  nh.getParam("pub_topic_pointcloud", parameter.pub_topic_pointcloud);
  nh.getParam("pub_topic_marker", parameter.pub_topic_marker);
  nh.getParam("cloud_frame_id", parameter.cloud_frame_id);

  // Convert pcd to pointcloud.
  features_of_objects ();

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe(parameter.subscribed_rostopic, parameter.queue_size_subscriber, cloud_cb);//boost::bind(cloud_cb, _1, parameter));

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ> > (parameter.pub_topic_pointcloud, 1);
  // Create a ROS publisher for the marker visualization
  vis_pub = nh.advertise<visualization_msgs::Marker>(parameter.pub_topic_marker, 1);

  // Spin
  ros::spin ();
}
