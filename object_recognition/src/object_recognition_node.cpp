#include <parameter/parameter_bag.h>
#include <cloud_handling.h>

int main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "object_recognition_node");
	ros::NodeHandle nh;

	// Initialize parameter structure
	parameter_bag parameter;

	// Retrieve all parameters
	nh.getParam("subscribed_rostopic", parameter.subscribed_rostopic);
	nh.getParam("queue_size_subscriber", parameter.queue_size_subscriber);
	nh.getParam("pub_topic_pointcloud", parameter.pub_topic_pointcloud);
	nh.getParam("pub_topic_marker", parameter.pub_topic_marker);
	nh.getParam("cloud_frame_id", parameter.cloud_frame_id);
	nh.getParam("object_files_path", parameter.object_files_path);
	// Retrieve parameters for resolution_filter
	nh.getParam("leafsize_x", parameter.filter.resolution.leafsize_x);
	nh.getParam("leafsize_y", parameter.filter.resolution.leafsize_y);
	nh.getParam("leafsize_z", parameter.filter.resolution.leafsize_z);
	// Retrieve parameters for passthrough_filter
	nh.getParam("min_filterlimit_x", parameter.filter.passthrough.min_filterlimit_x);
	nh.getParam("min_filterlimit_y", parameter.filter.passthrough.min_filterlimit_y);
	nh.getParam("min_filterlimit_z", parameter.filter.passthrough.min_filterlimit_z);
	nh.getParam("max_filterlimit_x", parameter.filter.passthrough.max_filterlimit_x);
	nh.getParam("max_filterlimit_y", parameter.filter.passthrough.max_filterlimit_y);
	nh.getParam("max_filterlimit_z", parameter.filter.passthrough.max_filterlimit_z);
	// Retrieve parameters for statistical_outlier_removal_filter
	nh.getParam("meank", parameter.filter.outlier.meank);
	nh.getParam("stddev_mul_thresh", parameter.filter.outlier.stddev_mul_thresh);
	// Retrieve parameters for euclidean_cluster_extraction (segmentation)
	nh.getParam("min_cluster_size", parameter.segmentation.euclidean_cluster.min_cluster_size);
	nh.getParam("max_cluster_size", parameter.segmentation.euclidean_cluster.max_cluster_size);
	nh.getParam("cluster_tolerance", parameter.segmentation.euclidean_cluster.cluster_tolerance);
	// Retrieve parameters for object recognition
	nh.getParam("correlation_thresh", parameter.recognition.correlation.correlation_thresh);
	nh.getParam("hist_size", parameter.recognition.hist_size);
	nh.getParam("image_width", parameter.recognition.correlation.image_width);
	nh.getParam("support_angle_cos", parameter.recognition.correlation.support_angle_cos);
	nh.getParam("min_pts_neighb", parameter.recognition.correlation.min_pts_neighb);
	nh.getParam("spin_image_descriptor_radius", parameter.recognition.correlation.spin_image_descriptor_radius);
	nh.getParam("norm_est_radius", parameter.recognition.correlation.norm_est_radius);
	// Retrieve parameters for visualization of the marker
	nh.getParam("color_alpha", parameter.visualization.marker.color_alpha);
	nh.getParam("frame_id", parameter.visualization.marker.frame_id);
	nh.getParam("orientation_x", parameter.visualization.marker.orientation_x);
	nh.getParam("orientation_y", parameter.visualization.marker.orientation_y);
	nh.getParam("orientation_z", parameter.visualization.marker.orientation_z);
	nh.getParam("orientation_w", parameter.visualization.marker.orientation_w);
	nh.getParam("color_r_duck", parameter.visualization.marker.color_r_duck);
	nh.getParam("color_g_duck", parameter.visualization.marker.color_r_duck);
	nh.getParam("color_b_duck", parameter.visualization.marker.color_r_duck);
	nh.getParam("color_r_duck", parameter.visualization.marker.color_r_human);
	nh.getParam("color_g_duck", parameter.visualization.marker.color_r_human);
	nh.getParam("color_b_duck", parameter.visualization.marker.color_r_human);
	nh.getParam("color_r_duck", parameter.visualization.marker.color_r_unknown);
	nh.getParam("color_g_duck", parameter.visualization.marker.color_r_unknown);
	nh.getParam("color_b_duck", parameter.visualization.marker.color_r_unknown);

	// Construct class cloud_processor with ros::NodeHandle and parameter structure
	cloud_handling c_handling (nh, parameter);
	// Load .pcd files of objects and convert to pointclouds and spin_images
	c_handling.features_of_objects();

	// Create a ROS subscriber for the input point cloud
	ros::Subscriber sub = nh.subscribe(parameter.subscribed_rostopic, parameter.queue_size_subscriber, &cloud_handling::Callback, &c_handling);

	// Spin
	ros::spin ();
}
