#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <string>
#include <iostream>
#include <cstdlib>
#include <ctime>
#include <chrono>


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
	pcl::PCDWriter writer;
	while(ros::ok()){
		ros::spinOnce();
		try{
			//original point cloud
			const auto before = std::chrono::system_clock::now();
			path = original_pc_name + to_string(i) + ".pcd";

        	writer.write<pcl::PointXYZRGB> (path, *global_cloud, false);
			cout << path << " file saved.\n";
			const auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now()-before);
			cout << duration.count()/1000.0 << "ms" << endl;
			++i;
		} 
		catch(const std::exception& ex){
			cout << ex.what() << endl;
		}
	}
}
