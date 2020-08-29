#include <iostream>
#include <cmath>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/extract_clusters.h>
#include <Eigen/Core>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/common/time.h>
#include <pcl/common/angles.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include <ros/ros.h>
#include <boost/foreach.hpp>

#include <vision_module/obj_mass.h>
#include <vision_module/rotation_matrix.h>
#include <vision_module/bbox_size.h>


using namespace std;
typedef pcl::PointXYZ PointType;

static ros::Subscriber sub;
static ros::Publisher pub1;
static ros::Publisher pub2;
static ros::Publisher pub3;


// Get N bits of the string from back to front.
char* Substrend(char*str,int n)
{
	char *substr=(char*)malloc(n+1);
	int length=strlen(str);
	if (n>=length)
	{
		strcpy(substr,str);
		return substr;
	}
	int k=0;
	for (int i=length-n;i<length;i++)
	{
		substr[k]=str[i];
		k++;
	}
	substr[k]='\0';
	return substr;
}


void callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud)
{
    printf ("Cloud: width = %d, height = %d\n", cloud->width, cloud->height);
    // TODO： process the data point.
    
    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(*cloud, pcaCentroid);
    Eigen::Matrix3f covariance;
    pcl::computeCovarianceMatrixNormalized(*cloud, pcaCentroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
    Eigen::Vector3f eigenValuesPCA = eigen_solver.eigenvalues();


    //Correct vertical between main directions
    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1)); 
    eigenVectorsPCA.col(0) = eigenVectorsPCA.col(1).cross(eigenVectorsPCA.col(2));
    eigenVectorsPCA.col(1) = eigenVectorsPCA.col(2).cross(eigenVectorsPCA.col(0));

    std::cout << "eigenValue(3x1):\n" << eigenValuesPCA << std::endl;
    std::cout << "eigenVector(3x3):\n" << eigenVectorsPCA << std::endl;
    std::cout << "mass center in camera coordinate system(4x1):\n" << pcaCentroid << std::endl;

    Eigen::Matrix4f tm = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f tm_inv = Eigen::Matrix4f::Identity();
    tm.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();   //R.
    tm.block<3, 1>(0, 3) = -1.0f * (eigenVectorsPCA.transpose()) *(pcaCentroid.head<3>());//  -R*t
    tm_inv = tm.inverse();

    //std::cout << "transformation matrix(4x4):\n" << tm << std::endl;
    std::cout << "transformation matrix tm'(4x4):\n" << tm_inv << std::endl;


    pcl::PointCloud<PointType>::Ptr transformedCloud(new pcl::PointCloud<PointType>);
    pcl::transformPointCloud(*cloud, *transformedCloud, tm);

    PointType min_p1, max_p1;
    Eigen::Vector3f c1, c;
    pcl::getMinMax3D(*transformedCloud, min_p1, max_p1);
    c1 = 0.5f*(min_p1.getVector3fMap() + max_p1.getVector3fMap());

    //std::cout << "center c1(3x1):\n" << c1 << std::endl;

    Eigen::Affine3f tm_inv_aff(tm_inv);
    pcl::transformPoint(c1, c, tm_inv_aff);

    Eigen::Vector3f whd, whd1;
    whd1 = max_p1.getVector3fMap() - min_p1.getVector3fMap();
    whd = whd1;
    float sc1 = (whd1(0) + whd1(1) + whd1(2)) / 3;

    std::cout << "3D properties:"<< std::endl;
    std::cout << "mass center(3x1):\n" << pcaCentroid << std::endl;
    std::cout << "rotation matrix(3x3):\n" << tm_inv.block<3, 3>(0, 0) << std::endl;
    std::cout << "bounding box size:"<< std::endl;
    std::cout << "width=" << whd1(0) << endl;
    std::cout << "height=" << whd1(1) << endl;
    std::cout << "depth=" << whd1(2) << endl;
    //std::cout << "scale1=" << sc1 << endl;


    /*const Eigen::Quaternionf bboxQ(tm_inv.block<3, 3>(0, 0));
    const Eigen::Vector3f    bboxT(c);
    auto euler = bboxQ1.toRotationMatrix().eulerAngles(0, 1, 2);
    std::cout << "Euler from quaternion in roll, pitch, yaw"<< std::endl << euler/3.14*180 << std::endl;*/

    //Convert the camera coordinate position from the camera coordinate system to the world coordinate system.(1,1,1,0,90,-90)
    /*Eigen::Matrix4f tw = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f tw_inv = Eigen::Matrix4f::Identity();
    Eigen::Vector4f tw_centroid;
    tw(0,0) = 0.0; tw(0,1) = 1.0; tw(0,2) = 0.0; tw(0,3) = 1.0;
    tw(1,0) = 1.0; tw(1,1) = 0.0; tw(1,2) = 0.0; tw(1,3) = 1.0;
    tw(2,0) = 0.0; tw(2,1) = 0.0; tw(2,2) = -1.0; tw(2,3) = 1.0;
    tw_inv = tw.inverse();
    tw_centroid = tw_inv * pcaCentroid;
    std::cout << "mass center in World coordinate system(4x1):\n" << tw_centroid << std::endl;
    */

    vision_module::obj_mass o_mass;
    vision_module::rotation_matrix r_matrix;
    vision_module::bbox_size b_size;
    
    //publish mass center of object
    o_mass.x = tm_inv(0,3);
    o_mass.y = tm_inv(1,3);
    o_mass.z = tm_inv(2,3);
    pub1.publish(o_mass);

    //publish rotation matrix from object to camera
    r_matrix.r11 = tm_inv(0,0);
    r_matrix.r21 = tm_inv(1,0);
    r_matrix.r31 = tm_inv(2,0);
    r_matrix.r12 = tm_inv(0,1);
    r_matrix.r22 = tm_inv(1,1);
    r_matrix.r32 = tm_inv(2,1);
    r_matrix.r13 = tm_inv(0,2);
    r_matrix.r23 = tm_inv(1,2);
    r_matrix.r33 = tm_inv(2,2);    
    pub2.publish(r_matrix);

    //publish mass center of object
    b_size.width = whd1(0);
    b_size.height = whd1(1);
    b_size.depth = whd1(2);
    pub3.publish(b_size);

    //pub.publish(pose_information);
  
}

int main(int argc, char** argv)
{
  std::cout<<"Init the pose information node:" <<std::endl;
  ros::init(argc, argv, "pose_information");
  ros::NodeHandle nh;
  sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZ>> ("region_growing", 1000, callback);
  pub1 = nh.advertise<vision_module::obj_mass> ("obj_mass", 1000);
  pub2 = nh.advertise<vision_module::rotation_matrix> ("rotation_matrix", 1000);
  pub3 = nh.advertise<vision_module::bbox_size> ("BBox_size", 1000);
  ros::spin();
}
