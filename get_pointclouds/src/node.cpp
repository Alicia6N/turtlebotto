#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <string>
#include <iostream>
#include <cstdlib>
using namespace std;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr global_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
const string original_pc_name = "src/turtlebotto/get_pointclouds/src/pcd_files/original/pcd_";

void callback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& msg){
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>(*msg));

	cout << "------\n";
	cout << "Amount of captured points: " << cloud->size() << endl;

	global_cloud = cloud;
}

int main(int argc, char** argv){
	ros::init(argc, argv, "get_pointclouds_node");
	ros::NodeHandle nh;
	ros::Subscriber sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZRGB> >("/camera/depth/points", 1, callback);
	int i = 0;
	stringstream ss;
	string path;

	while(ros::ok()){
		ros::spinOnce();
		try{
			//original point cloud
			path = original_pc_name + to_string(i) + ".pcd";
			pcl::PCDWriter writer;
        	writer.write<pcl::PointXYZRGB> (path, *global_cloud, false);
			cout << path << " file saved.\n";
			i++;
		} 
		catch(const std::exception& ex){
			cout << ex.what() << endl;
		}
	}
}
