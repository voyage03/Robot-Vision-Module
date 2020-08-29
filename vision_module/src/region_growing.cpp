#include <ctime>
#include <iostream>
#include <ros/ros.h>
#include <boost/foreach.hpp>
#include <iostream>
#include <algorithm>
#include <vector>
#include <queue>
#include <cmath>
#include <ctime>

#include <pcl/common/common.h>
#include <pcl/console/print.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/principal_curvatures.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/pcl_exports.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <ros/ros.h>
#include <boost/foreach.hpp>

using namespace std;

static ros::Subscriber sub;
static ros::Publisher pub;

void callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud)
{
    printf ("Cloud: width = %d, height = %d\n", cloud->width, cloud->height);
    /*for( auto pt : cloud->points){
  	 printf ("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);
    }*/

    //normal estimation;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());//create an empty kd_tree
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());//create an empty normal point cloud
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    tree->setInputCloud(cloud);
    ne.setInputCloud(cloud);
    ne.setSearchMethod(tree);
    ne.setKSearch (20); //number of search neighborhood points
    ne.compute(*normals); //compute normal and save to normal point cloud
    
    
    //neighborhood
    int point_num = cloud->points.size();//number of point cloud
    vector<int>nebor_index;//store index of neighborhood points
    vector<float>nebor_dis;//store distance of neighborhood points
    int nebor_size(20);//number of neighborhood points
    vector<vector<int> > point_nebor;
    point_nebor.resize(point_num, nebor_index);
    for (int i_point = 0; i_point < point_num; i_point++)
    {
        if(tree->nearestKSearch(cloud->points[i_point], nebor_size, nebor_index, nebor_dis))
        {
            //points[i_point]: 20 index of neighborhood point with i_point
            point_nebor[i_point].swap(nebor_index);
            //each points in point_nebor store 20 index of neighborhoood around themselves.
        }
        else
        {
            PCL_ERROR("WARNING:FALSE NEARERTKSEARCH");
        }
    }
	
	
    //curvature
    vector<pair<float, int> > point_curvature_index;
    float curvature = 0.0;
    pcl::PrincipalCurvaturesEstimation<pcl::PointXYZ, pcl::Normal, pcl::PrincipalCurvatures> cur;
    pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr curvatures(new pcl::PointCloud<pcl::PrincipalCurvatures>);
    cur.setInputCloud(cloud);
    cur.setInputNormals(normals);
    cur.setSearchMethod(tree);
    //pc.setRadiusSearch(0.05);
    cur.setKSearch(20);
    cur.compute(*curvatures);
    for (size_t i = 0; i < point_num; i++)
    {
        curvature = (*curvatures)[i].pc1 * (*curvatures)[i].pc2;
        //store point curvature and point(float, int)
        point_curvature_index.push_back(make_pair(curvature,i));
    }
    sort(point_curvature_index.begin(), point_curvature_index.end());//sort according curvature
    //cout<<"curvature finished"<<endl;
	
	
    //region growing
    float normal_threshold_low = cosf(1.0 / 180.0 * M_PI);//Theta_low
    float normal_threshold_high = cosf(10.0 / 180.0 * M_PI);//Theta_high
    float curvature_threshold = 0.7;//curvature threshold
    int counter_(0);
    int segment_label(0);
    int seed_index_orginal = point_curvature_index[0].second;//selected min curvature point as seed points
    vector<int> point_label;
    vector<int> seg_ave_num;
    point_label.resize(point_num, -1);//(size, initial value)
    while(counter_<point_num)//iterate over all the point cloud
    {
        queue<int> seed;//create a queue
        seed.push(seed_index_orginal);//add to the end of queue
        int counter_1(1);
        point_label[seed_index_orginal] = segment_label;//segment_label is 0
        while (!seed.empty())
        {
            int curr_seed = seed.front();//first element of seed
            seed.pop();//Pop-up the first element
            int K_nebor(0);
            while (K_nebor<nebor_size)//detect curr_seed's surrounding points
            {
                int index_nebor = point_nebor[curr_seed][K_nebor];
                //float g_curvature;
                //g_curvature = (*curvatures)[index_nebor].pc1 * (*curvatures)[index_nebor].pc2;
                if (point_label[index_nebor] != -1)//have selected, skip this point
                {
                    K_nebor++;
                    continue;
                }
                bool is_a_seed = false;
                Eigen::Map<Eigen::Vector3f> vec_curr_point(static_cast<float*>(normals->points[curr_seed].normal));
                Eigen::Map<Eigen::Vector3f> vec_nebor_point (static_cast<float*>(normals->points[index_nebor].normal));
                float dot_normal = fabsf(vec_curr_point.dot(vec_nebor_point));
                if (dot_normal<normal_threshold_high)
                {
                    is_a_seed = false;
                }
                else if(dot_normal>normal_threshold_high&&dot_normal<normal_threshold_low)
                {
                    int Edge_nebor(0);
                    int num_out_high(0);
                    //detect if index_nebor point is Edge point or not
                    while (Edge_nebor<nebor_size)
                    {
                        int Edge_index_nebor = point_nebor[index_nebor][Edge_nebor];
                        Eigen::Map<Eigen::Vector3f> vec_curr_edge_point(static_cast<float*>(normals->points[index_nebor].normal));
                        Eigen::Map<Eigen::Vector3f> vec_edge_point_nebor(static_cast<float*>(normals->points[Edge_index_nebor].normal));
                        float dot_normal_edge = fabsf(vec_curr_edge_point.dot(vec_edge_point_nebor));
                        if (dot_normal_edge<normal_threshold_high)
                        {
                            num_out_high++;//number of the point which out high threhold
                        }
                        Edge_nebor++;	
                    }
                    float proportion = num_out_high/nebor_size;
                    if (proportion>0.4)    
                    {
                        is_a_seed = false;
                    }
                    /*else if (g_curvature>curvature_threshold)
                    {
		        is_a_seed = false;
		    }*/
                    else
                    {
                        is_a_seed = true;
                    } 
                }
                /*else if (g_curvature>curvature_threshold)
                {
		    is_a_seed = false;
		}*/
                else
                {
                    is_a_seed = true;
                }
                if (!is_a_seed)//not a seed point, throw it away
                {
                    K_nebor++;
                    continue;
                }
                point_label[index_nebor] = segment_label;
                counter_1++;
                if (is_a_seed)//is a seed, add to seed queue
                {
                    seed.push(index_nebor);
                }
                K_nebor++;
            }
        }
        segment_label++;//number of segmentation
        counter_ += counter_1;//the number of points which have been detected
        seg_ave_num.push_back(counter_1);//number of points on one plane
        for (int i_seg = 0; i_seg < point_num; i_seg++)
        {
            int index_ = point_curvature_index[i_seg].second;
            if (point_label[index_] == -1)//haven't been detected.
            {
                seed_index_orginal = index_;
                break;
            }
        }
    }
	
	
    //summary of segmentation results
    vector<pcl::PointIndices> cluster_;
    pcl::PointIndices segments;
    int seg_num = seg_ave_num.size();//number of all planes
    cluster_.resize(seg_num, segments);
    for(int i_seg = 0; i_seg < seg_num; i_seg++)//iterate over all the planes
    {
        cluster_[i_seg].indices.resize(seg_ave_num[i_seg],0);
    }
    vector<int> counter;
    counter.resize(seg_num, 0);
    for (int i_point = 0; i_point < point_num; i_point++)
    {
        int segment_index = point_label[i_point];//Which plane
        int nebor_idx = counter[segment_index];
        cluster_[segment_index].indices[nebor_idx] = i_point;
        counter[segment_index] += 1;
    }
	
    //Remove outline points
    vector<pcl::PointIndices> clusters;
    int min_number = 100, max_number = point_num/10;
    for (int i_seg = 0; i_seg < seg_num; i_seg++)//iterate over all the planes
    {
        if (cluster_[i_seg].indices.size() > min_number&&cluster_[i_seg].indices.size() < max_number)
        {
            clusters.push_back(cluster_[i_seg]);
        }
    }
	

    pcl::PointCloud<pcl::PointXYZ>::Ptr color_point(new pcl::PointCloud<pcl::PointXYZ>);
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_point(new pcl::PointCloud<pcl::PointXYZRGB>);
    //srand(time(nullptr));
    /*vector<unsigned char> color;
    for (int i_segment = 0; i_segment < clusters.size(); i_segment++)
    {
        color.push_back(static_cast<unsigned char>(rand() % 256));
        color.push_back(static_cast<unsigned char>(rand() % 256));
        color.push_back(static_cast<unsigned char>(rand() % 256));
    }
    int color_index(0);*/
    for (int i_seg = 0; i_seg < clusters.size(); i_seg++)
    {
        int clusters_size = clusters[i_seg].indices.size();
        for (int i_idx = 0; i_idx < clusters_size; i_idx++)
        {
            pcl::PointXYZ n_point;
            n_point.x = cloud->points[clusters[i_seg].indices[i_idx]].x;
            n_point.y = cloud->points[clusters[i_seg].indices[i_idx]].y;
            n_point.z = cloud->points[clusters[i_seg].indices[i_idx]].z;
            //n_point.r = color[3 * color_index];
            //n_point.g = color[3 * color_index + 1];
            //n_point.b = color[3 * color_index + 2];
            color_point->push_back(n_point);
        }
        //color_index++;
    }

    pcl::io::savePCDFileASCII("/home/miao/catkin_ws/src/vision_module/src/data/region.pcd", *color_point);
  
    pub.publish(color_point);
  
}

int main(int argc, char** argv)
{
  std::cout<<"Init the region growing node:" <<std::endl;
  ros::init(argc, argv, "region_growing");
  ros::NodeHandle nh;
  sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZ>>("removal_outlier", 1000, callback);
  pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>> ("region_growing", 1000);
  ros::spin();
}
