#include <ctime>
#include <iostream>
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <boost/foreach.hpp>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>

static ros::Subscriber sub;
static ros::Publisher pub;

void callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud)
{
  printf ("Cloud: width = %d, height = %d\n", cloud->width, cloud->height);
  /*for( auto pt : msg->points){
  	 printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
  }*/
  // TODO： process the data point.
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

  std::cerr << "Cloud before filtering: " << std::endl;
  std::cerr << *cloud << std::endl;

  // Create the filtering object
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud (cloud);
  sor.setMeanK (20);
  sor.setStddevMulThresh (2.0);
  sor.filter (*cloud_filtered);

  std::cerr << "Cloud after filtering: " << std::endl;
  std::cerr << *cloud_filtered << std::endl;

  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZ> ("/home/miao/catkin_ws/src/vision_module/src/data/inliers.pcd", *cloud_filtered, false);
  
  pub.publish(cloud_filtered);

  
  //sor.setNegative (true);
  //sor.filter (*cloud_filtered);
  //writer.write<pcl::PointXYZ> ("test_outliers.pcd", *cloud_filtered, false);
  
}

int main(int argc, char** argv)
{
  std::cout<<"Init the outlier removal node:" <<std::endl;
  ros::init(argc, argv, "removal_outlier");
  ros::NodeHandle nh;
  sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZ>>("pointcloud", 1000, callback);
  pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>> ("removal_outlier", 1000);
  ros::spin();
}
