#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <string>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <cstdlib>
using namespace std;


pcl::PointCloud<pcl::PointXYZRGB>::Ptr visu_pc (new pcl::PointCloud<pcl::PointXYZRGB>);

void simpleVis (){
  	pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
	while(!viewer.wasStopped()){
	  viewer.showCloud (visu_pc);
	  boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
	}
}

int main(int argc, char** argv){
	ros::init(argc, argv, "pcl_viewer");
	boost::thread t(simpleVis);

	string pcd_file_path = "";
	string argumento = "";
	int index = 0;
    if (argc > 2){
		index = atoi(argv[2]);
        argumento = argv[1];
        if (argumento == "--o")
            pcd_file_path = "src/turtlebotto/get_pointclouds/src/pcd_files/original/pcd_8";
        else if (argumento == "--s")
            pcd_file_path = "src/turtlebotto/get_pointclouds/src/pcd_files/sor/sor_pcd_8";
        else if (argumento == "--sv")
            pcd_file_path = "src/turtlebotto/get_pointclouds/src/pcd_files/sor_filtered/sor_filtered_pcd_8";
        else if (argumento == "--v")
            pcd_file_path = "src/turtlebotto/get_pointclouds/src/pcd_files/filtered/filtered_pcd_8";

		argumento += to_string(index);
    }
    else{
		argumento = argv[1];
		if (argc == 2){
			if (argumento == "--f")
				pcd_file_path = "src/turtlebotto/mapping_3d/src/final_cloud";
			else{
				cout << argc;
				string usage = "{--o (Original pcds) | --s (SOR pcds) | --v (VoxelGrid pcds) | --f (final cloud)}";
				cout << "Incorrect program use! Usage must be: " << endl;
				cout << "rosrun *package_name* *executable_name* " << usage << endl;
				return 0;
			}
		}
		else{
			cout << argc;
			string usage = "{--o (Original pcds) | --s (SOR pcds) | --v (VoxelGrid pcds) | --f (final cloud)} [index]";
			cout << "Incorrect program use! Usage must be: " << endl;
			cout << "rosrun *package_name* *executable_name* " << usage << endl;
			return 0;
		}
    }

	bool name = false;
	if(argc>=2){
		stringstream ss;
		ss <<  pcd_file_path << ".pcd";
		string path = ss.str();

		if(pcl::io::loadPCDFile<pcl::PointXYZRGB> (path, *visu_pc)==-1){
			cout << "Can't read file\n";
		}
		else if(!name){
			cout << "Viewing file: " << path << "\n";
			name = true;
			while(true){ }
		}
	}
	else{
		cout << "No file name passed\n";
	}
 
}