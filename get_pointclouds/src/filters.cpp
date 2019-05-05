#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <string>
#include <iostream>
#include <cstdlib>
#include <iostream>
#include <pcl/io/pcd_io.h>
using namespace std;


const string original_pc_name = "src/turtlebotto/get_pointclouds/src/pcd_files/original/pcd_";
const string sor_pc_name = "src/turtlebotto/get_pointclouds/src/pcd_files/sor/sor_pcd_";
const string sor_filtered_pc_name = "src/turtlebotto/get_pointclouds/src/pcd_files/sor_filtered/sor_filtered_pcd_";
const string filtered_pc_name = "src/turtlebotto/get_pointclouds/src/pcd_files/filtered/filtered_pcd_";
bool sor_flag = false;

int main(int argc, char** argv){
    ros::init(argc, argv, "get_pointclouds_filters");
    int id = 0;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_sor (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_sor_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PCDWriter writer;
    pcl::PCDReader reader; 
    pcl::VoxelGrid<pcl::PointXYZRGB> vGrid;
    string path = "";

	while(ros::ok()){
		if(reader.read<pcl::PointXYZRGB> (original_pc_name + std::to_string(id) + ".pcd", *cloud) != 0) {
			cout << "Finished" << "\n";
			break;
        }
        cout << "----------" << "\n";
        cout << "Iteration " << id << "\n";
		cout << "Points before filtering: " << cloud->size() << "\n";

        //voxelgrid point cloud
		vGrid.setInputCloud (cloud);
		vGrid.setLeafSize (0.01f, 0.01f, 0.01f);
		vGrid.filter (*cloud_filtered);
		path = filtered_pc_name + to_string(id) + ".pcd";
		writer.write<pcl::PointXYZRGB> (path, *cloud_filtered, false);
		cout << "Points after VoxelGrid: " << cloud_filtered->size() << "\n";

        //sor point cloud
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
        sor.setInputCloud (cloud);
        sor.setMeanK (50);
        sor.setStddevMulThresh (1.0);
        sor.filter (*cloud_sor);
        path = sor_pc_name + to_string(id) + ".pcd";
        writer.write<pcl::PointXYZRGB> (path, *cloud_sor, false);
        cout << "Points after SOR: " << cloud_sor->size() << "\n";

        //sor voxelgrid point cloud
        vGrid.setInputCloud (cloud_sor);
        vGrid.setLeafSize (0.01f, 0.01f, 0.01f);
        vGrid.filter (*cloud_sor_filtered);
        path = sor_filtered_pc_name + to_string(id) + ".pcd";
        writer.write<pcl::PointXYZRGB> (path, *cloud_sor_filtered, false);
        cout << "Points after SOR: " << cloud_sor_filtered->size() << "\n";

        ++id;
	}
}