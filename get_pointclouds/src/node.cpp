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
pcl::PointCloud<pcl::PointXYZRGB>::Ptr global_cloud_sor (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr global_cloud_sor_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr global_cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);

const string original_pc_name = "original/pcd_";
const string sor_pc_name = "sor/sor_pcd_";
const string sor_filtered_pc_name = "sor_filtered/sor_filtered_pcd_";
const string filtered_pc_name = "filtered/filtered_pcd_";
bool sor_flag = false;

void callback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& msg){
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>(*msg));
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_sor (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_sor_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);

	cout << "Puntos capturados: " << cloud->size() << endl;

	if (sor_flag){
		pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
		sor.setInputCloud (cloud);
		sor.setMeanK (50);
		sor.setStddevMulThresh (1.0);
		sor.filter (*cloud_sor);

		cout << "Puntos tras SOR: " << cloud_sor->size() << endl;
		pcl::VoxelGrid<pcl::PointXYZRGB> vGrid;
		vGrid.setInputCloud (cloud_sor);
		vGrid.setLeafSize (0.05f, 0.05f, 0.05f);
		vGrid.filter (*cloud_sor_filtered);
	}
	
	pcl::VoxelGrid<pcl::PointXYZRGB> vGrid;
	vGrid.setInputCloud (cloud);
	vGrid.setLeafSize (0.05f, 0.05f, 0.05f);
	vGrid.filter (*cloud_filtered);

	cout << "Puntos tras VoxelGrid: " << cloud_filtered->size() << endl;

	global_cloud = cloud;
	global_cloud_sor = cloud_sor;
	global_cloud_sor_filtered = cloud_sor_filtered;
	global_cloud_filtered = cloud_filtered;
}

int main(int argc, char** argv){
	string argumento = argv[1];
	if (argumento == "--sor")
		sor_flag = true;

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
			path = "pcd_files/" + original_pc_name + to_string(i) + ".pcd";
			pcl::PCDWriter writer;
        	writer.write<pcl::PointXYZRGB> (path, *global_cloud, false);
			cout << path << " file saved.\n";
			path = "";

			//sor point cloud
			if (sor_flag){
				path = "pcd_files/" + sor_pc_name + to_string(i) + ".pcd";
        		writer.write<pcl::PointXYZRGB> (path, *global_cloud_sor, false);
				cout << path << " file saved.\n";
				path = "";

				path = "pcd_files/" + sor_filtered_pc_name + to_string(i) + ".pcd";
        		writer.write<pcl::PointXYZRGB> (path, *global_cloud_sor_filtered, false);
				cout << path << " file saved.\n";
				path = "";
			}
			
			//voxelgrid point cloud
			path = "pcd_files/" + filtered_pc_name + to_string(i) + ".pcd";
			writer.write<pcl::PointXYZRGB> (path, *global_cloud_filtered, false);
			cout << path << " file saved.\n";
			path = "";

			boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
			i++;
		} 
		catch(const std::exception& ex){
			cout << ex.what() << endl;
		}
	}
}
