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
pcl::PointCloud<pcl::PointXYZRGB>::Ptr global_cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);

const string original_pc_name = "/original/pcd_";
const string sor_pc_name = "/sor/sor_pcd_";
const string filtered_pc_name = "/filtered/filtered_pcd_";
bool sor_flag = false;

void callback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& msg){
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>(*msg));
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_sor (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);

	cout << "Puntos capturados: " << cloud->size() << endl;

	pcl::VoxelGrid<pcl::PointXYZRGB> vGrid;
	vGrid.setInputCloud (cloud);

	if (sor_flag){
		pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
		sor.setInputCloud (cloud);
		sor.setMeanK (50);
		sor.setStddevMulThresh (1.0);
		sor.filter (*cloud_sor);

		cout << "Puntos tras SOR: " << cloud_sor->size() << endl;
		vGrid.setInputCloud (cloud_sor);
	}
	
	vGrid.setLeafSize (0.05f, 0.05f, 0.05f);
	vGrid.filter (*cloud_filtered);

	cout << "Puntos tras VoxelGrid: " << cloud_filtered->size() << endl;

	global_cloud = cloud;
	global_cloud_sor = cloud_sor;
	global_cloud_filtered = cloud_filtered;
}

int main(int argc, char** argv){
	if (argc > 2)
		if (argv[1] == "--sor")
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
			ss <<  "src/turtlebotto/pcd_files/" << original_pc_name << i << ".pcd";
			path = ss.str();
			pcl::io::savePCDFile (path, *global_cloud, true);
			cout << path << " file saved.\n";
			ss.str(std::string());

			//sor point cloud
			if (sor_flag){
				ss <<  "src/turtlebotto/pcd_files/" << sor_pc_name << i << ".pcd";
				path = ss.str();
				pcl::io::savePCDFile (path, *global_cloud_sor, true);
				cout << path << " file saved.\n";
				ss.str(std::string());
			}

			//voxelgrid point cloud
			ss <<  "src/turtlebotto/pcd_files/" << filtered_pc_name << i << ".pcd";
			path = ss.str();
			pcl::io::savePCDFile (path, *global_cloud_filtered, true);
			cout << path << " file saved.\n";
			ss.str(std::string());

			boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
			i++;
		} catch(const std::exception& ex){ }
	}
}
