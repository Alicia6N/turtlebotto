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
void simpleVis ()
{
  	pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
	while(!viewer.wasStopped())
	{
	  viewer.showCloud (visu_pc);
	  boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
	}

}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "pcl_viewer");
  ros::NodeHandle nh;
  boost::thread t(simpleVis);
  bool name = false;
  if(argc>=2){
       while(ros::ok()){ 
        ros::spinOnce();
        stringstream ss;
        ss <<  "src/turtlebotto/pcd_files/pcd_" << argv[1] << ".pcd";
        string path = ss.str();
        if(pcl::io::loadPCDFile<pcl::PointXYZRGB> (path, *visu_pc)==-1){
            cout << "Can't read file\n";
        }
		else if(!name){
			cout << "Viewing file: " << path << "\n";
			name = true;
		}

        }
  }
  else{
	  cout << "No file name passed\n";
  }
 
}