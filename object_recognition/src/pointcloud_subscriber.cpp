#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>
//#include <boost/foreach.hpp>

void callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
//  printf ("Cloud: width = %d, height = %d\n", msg->width, msg->height);
//  BOOST_FOREACH (const pcl::PointXYZ& pt, msg->points)
//  	printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
}

int main3(int argc, char** argv)
{
//  ros::init(argc, argv, "sub_pcl");
//  ros::NodeHandle nh;
//  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth_registered/points", 1000, callback);
//  ros::spin();
}
