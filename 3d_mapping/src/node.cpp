#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <string>
#include <iostream>
#include <cstdlib>
using namespace std;

string pcd_file_path;

int main(int argc, char** argv){
	string argumento = argv[1];

	if (argc < 2){
		if (argumento == "--o")
			pcd_file_path = "../../get_pointclouds/src/pcd_files/original/pcd_";
		if (argumento == "--s")
			pcd_file_path = "../../get_pointclouds/src/pcd_files/sor/sor_pcd_";
		if (argumento == "--sv")
			pcd_file_path = "../../get_pointclouds/src/pcd_files/sor_filtered/sor_filtered_pcd_";
		if (argumento == "--v")
			pcd_file_path = "../../get_pointclouds/src/pcd_files/filtered/filtered_pcd_";
	}
	else{
		string usage = "{--o (Original pcds) | --s (SOR pcds) | --v (VoxelGrid pcds)}";
		cout << "Incorrect program use! Usage must be: " << endl;
		cout << "rosrun *package_name* *executable_name* " << usage << endl;
	}

  	ros::init(argc, argv, "3d_mapping_node");

  	while(ros::ok()){
	  	ros::spinOnce();
  	}
}
