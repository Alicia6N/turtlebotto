#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include "pcl/keypoints/sift_keypoint.h"
#include "pcl/features/pfh.h"
#include "pcl/visualization/pcl_visualizer.h"
#include <string>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
using namespace std;



void visualize_keypoints (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr points,
                          const pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints)
{
  cout << ">>>>> Keypoints detected: " << keypoints->size() << endl;
  // Add the points to the vizualizer
  pcl::visualization::PCLVisualizer viz;
  viz.addPointCloud (points, "points");
  // Draw each keypoint as a sphere 

  for (size_t i = 0; i < keypoints->size (); ++i)
  {
    // Get the point data
    const pcl::PointWithScale & p = keypoints->points[i];
    // Pick the radius of the sphere *
    float r =  p.scale;
    // * Note: the scale is given as the standard deviation of a Gaussian blur, so a
    //   radius of 2*p.scale is a good illustration of the extent of the keypoint
    // Generate a unique string for each sphere
    std::stringstream ss ("keypoint");
    ss << i;
    // Add a sphere at the keypoint
    viz.addSphere (p, 0.1*p.scale, 1.0, 0.0, 0.0, ss.str ());
  }
  // Give control over to the visualizer
  viz.spin ();
}


void detect_keypoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points, pcl::PointCloud<pcl::PointWithScale>::Ptr &keypoints){
    pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointWithScale> sift;
    //FLANN-based Kdtree to perfom neighborhood searches
    sift.setSearchMethod(pcl::search::KdTree<pcl::PointXYZRGB>::Ptr (new pcl::search::KdTree<pcl::PointXYZRGB>));
    //Detection parameters
    const float min_scale = 0.1f;
    const int nr_octaves = 6;
    const int nr_scales_per_octave = 10;
    const float min_contrast = 0.5f;
    sift.setScales(min_scale,nr_octaves,nr_scales_per_octave);
    sift.setMinimumContrast(min_contrast);
    //Input
    sift.setInputCloud(points);
    //Detect and store
    sift.compute(*keypoints);
    visualize_keypoints(points,keypoints);
}





void find_correspondences(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points1, pcl::PointCloud<pcl::PointWithScale>::Ptr &keypoints1,
                          pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points2, pcl::PointCloud<pcl::PointWithScale>::Ptr &keypoints2){
    return;
}

int main (int argc, char** argv)
{
    ros::init(argc, argv, "mapping_3d_node");
    // Pevious, current and accumulated cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr currCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointWithScale>::Ptr currKeypoints (new pcl::PointCloud<pcl::PointWithScale>);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr nextCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointWithScale>::Ptr nextKeypoints (new pcl::PointCloud<pcl::PointWithScale>);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr finalCloud (new pcl::PointCloud<pcl::PointXYZRGB>);

    int id = 0;
    string pcd_file_path = "";
    string argumento = argv[1];
    if (argc == 2){
        if (argumento == "--o")
            pcd_file_path = "src/turtlebotto/get_pointclouds/src/pcd_files/original/pcd_";
        else if (argumento == "--s")
            pcd_file_path = "src/turtlebotto/get_pointclouds/src/pcd_files/sor/sor_pcd_";
        else if (argumento == "--sv")
            pcd_file_path = "src/turtlebotto/get_pointclouds/src/pcd_files/sor_filtered/sor_filtered_pcd_";
        else if (argumento == "--v")
            pcd_file_path = "src/turtlebotto/get_pointclouds/src/pcd_files/filtered/filtered_pcd_";
	}
	else{
        cout << argc;
		string usage = "{--o (Original pcds) | --s (SOR pcds) | --v (VoxelGrid pcds)}";
		cout << "Incorrect program use! Usage must be: " << endl;
		cout << "rosrun *package_name* *executable_name* " << usage << endl;
	}

    pcl::PCDReader reader;    
    while(ros::ok()){
        //Read two pcd files.
        reader.read<pcl::PointXYZRGB> (pcd_file_path + std::to_string(id) + ".pcd", *currCloud);
        if(reader.read<pcl::PointXYZRGB> (pcd_file_path + std::to_string(id+1) + ".pcd", *nextCloud) != 0)    break;
        cout << "Detecting keypoints in " << id << "..." << endl;
        detect_keypoints(currCloud,currKeypoints);
        cout << "Detecting keypoints in " << id+1 << "..." << endl;
        detect_keypoints(nextCloud,nextKeypoints);
        //
        //find_correspondences(currCloud,currKeypoints,nextCloud,nextKeypoints);

    }

    //Write final scene
    pcl::PCDWriter writer;
   // writer.write<pcl::PointXYZRGB> ("," *finalCloud, false);
    return (0);
}
