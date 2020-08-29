#include <ctime>
#include <iostream>
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/filter.h>


int main(int argc, char** argv){

	std::cout<<"Start to get the point cloud." <<std::endl;
	ros::init (argc, argv, "simulation_pub");

	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>> ("pointcloud", 1000);
	//pcl::PointCloud<pcl::PointXYZ>::Ptr msg (new pcl::PointCloud<pcl::PointXYZ>(4,1));
	//msg->header.frame_id = "camera_frame";

	//point cloud height and width, for loop to puch back the points clouds
	//TO DO: wirte a for loop to push_bakc all th point in the pcd.
        pcl::PointCloud<pcl::PointXYZ>::Ptr msg (new pcl::PointCloud<pcl::PointXYZ>);
        msg->header.frame_id = "camera_frame";
        //msg->height = msg->width = 1;
        pcl::io::loadPCDFile("/home/miao/catkin_ws/src/vision_module/src/data/test.pcd", *msg);
        
        std::vector<int> mapping;
        pcl::removeNaNFromPointCloud(*msg, *msg, mapping);

        pcl::io::savePCDFileASCII("/home/miao/catkin_ws/src/vision_module/src/data/nan_remove.pcd", *msg);

	ros::Rate loop_rate(0.015);
	while (nh.ok())
	{

		pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
		pub.publish (msg);
		ros::spinOnce ();
		loop_rate.sleep ();
	}

}

