# Robot Vision Module

## Functional description


In this project, a vision module is built. It is used to detect and obtain 3D information of unknown objects on the desktop. It is developed based on the ROS platform, which uses the PCL library and the Eigen3 library to process the three-dimensional point cloud data obtained by the RGB-D sensor. This is a package of work space based on ROS.

Firstly, an improved area growth algorithm based on surface smoothness conditions is used to create the surface. The improved region growing algorithm uses dual thresholds and introduces the concept of edge points to segment point clouds. It solves the problem of false boundaries caused by sensor noise. Then filter the segmented point cloud to remove the desktop and background. Use principal component analysis to process the point cloud that contains only the object, calculate the feature value and feature vector of the object point cloud, and then further obtain the bounding box, center of mass, transformation matrix, and BBox size of the object as the 3D information of the object.

It is simple and can be implemented in real time. In the absence of a 3D model of the object, it can extract the 3D information of the object very well. This information will provide powerful support for the robot's grasping.


## Development environment

```
Ubuntu: 18.04
C++: 11.0
ROS: Melodic
PCL: 1.72
Eigen3

packages:
  moveit_msgs
  pcl_conversions
  pcl_msgs
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  message_generation
```
## Package structure


```
-----vision_module
  |
  |----- include
  |  |
  |  |----- vision_module
  |
  |----- launch
  |  |
  |  |----- vision_module.launch
  |
  |----- msg
  |  |
  |  |----- bbox_size.msg
  |  |
  |  |----- obj_mass.msg
  |  |
  |  |----- rotation_matrix.msg
  |
  |----- src
  |  |
  |  |----- data
  |  |  |
  |  |  |----- test.pcd
  |  |
  |  |----- simulation_pub.cpp
  |  |
  |  |----- removal_outlier.cpp
  |  |
  |  |----- region_growing.cpp
  |  |
  |  |----- pose_information.cpp
  |
  |----- CMakeLists.txt
  |
  |----- package.xml
  
When you import the project successfully, you need to set the main file as the vision_module file.
```

## Usage
```
1. Open and set up the connection of elasticsearch and kibana with your computer.
2. Run SearchingEngineIn.py and and just run it once. This converts the csv to json file with different indices and uploads them onto Elasticsearch.
3. Run user_search.py and user can choose two ways to search:
   * Standard searching:
     * Type the query.
     * The Engine will get the doc.
   * Advanced searching:
     * choose models the user want to use
     * Select fields 
     * Choose the weights of different fields
     * Type the query
     * Choose the way of combination
     * The Engine will get the doc.

Note: It is better to ensure queries are more than one word.
```

