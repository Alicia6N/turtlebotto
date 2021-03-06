#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include "pcl/keypoints/sift_keypoint.h"
#include "pcl/features/pfh.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <string>
#include <sstream>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <Eigen/Core>
#include <pcl/features/pfhrgb.h>
#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/features/normal_3d.h"
#include <pcl/registration/transforms.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/transformation_estimation_svd.h>
using namespace std;

const float NORMAL_RADIUS = 0.03;
const float FEATURE_RADIUS = 0.08;
const string FINAL_PATH = "src/turtlebotto/mapping_3d/src/final_cloud.pcd";
bool INITIALIZED = false;
string PCD_FILE_PATH = "";
pcl::Feature<pcl::PointXYZRGB, pcl::PFHRGBSignature250>::Ptr ft_descriptor (new pcl::PFHRGBEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::PFHRGBSignature250>);
pcl::CorrespondencesPtr ransac_corr(new pcl::Correspondences);

pcl::PointCloud<pcl::PointXYZRGB>::Ptr nextCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZI>::Ptr nextKeypoints (new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr nextDescriptors (new pcl::PointCloud<pcl::PFHRGBSignature250>);

pcl::PointCloud<pcl::PointXYZRGB>::Ptr finalCloud (new pcl::PointCloud<pcl::PointXYZRGB>);

void simpleVis (){
  	pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
	while(!viewer.wasStopped()){
	  viewer.showCloud (finalCloud);
	  boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
	}
}

void correspondences_filter(pcl::PointCloud<pcl::PointXYZI>::Ptr keypoint1,
                    pcl::PointCloud<pcl::PointXYZI>::Ptr keypoint2, vector<int>& ctnCorrespondences, vector<int>& ntcCorrespondences){
	std::vector<std::pair<unsigned, unsigned>> corr_pair;
  cout << "Correspondences found: " << ctnCorrespondences.size() << "\n";
	 for (unsigned i = 0; i < ctnCorrespondences.size (); ++i) {
        if (ntcCorrespondences[ctnCorrespondences[i]] == i) {
            corr_pair.push_back(std::make_pair(i, ctnCorrespondences[i]));
        }
    }

    ransac_corr->resize (corr_pair.size());
    cout << "Pair of correspondences: " << corr_pair.size() << "\n";
    for (unsigned i = 0; i < corr_pair.size(); ++i)
    {
        (*ransac_corr)[i].index_query = corr_pair[i].first;
        (*ransac_corr)[i].index_match = corr_pair[i].second;
    }

    
    pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZI> ransac;
    ransac.setInputSource(keypoint1);
    ransac.setInputTarget(keypoint2);
    ransac.setInputCorrespondences(ransac_corr);
    ransac.getCorrespondences(*ransac_corr);
}

void detect_descriptors (pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points,
                                   pcl::PointCloud<pcl::PointXYZI>::Ptr &keypoints,
                                   pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr &descriptors_out){
   	pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_xyzrgb(new pcl::PointCloud<pcl::PointXYZRGB>);
    keypoints_xyzrgb->points.resize(keypoints->points.size());

    // We need to transfer the type
    pcl::copyPointCloud(*keypoints, *keypoints_xyzrgb);

    // Try to cast its type 
    pcl::FeatureFromNormals<pcl::PointXYZRGB, pcl::Normal, pcl::PFHRGBSignature250>::Ptr nm_ft = boost::dynamic_pointer_cast<pcl::FeatureFromNormals<pcl::PointXYZRGB, pcl::Normal, pcl::PFHRGBSignature250>> (ft_descriptor);

    // Setting main descriptor properties

    ft_descriptor->setSearchSurface(points);
    ft_descriptor->setInputCloud(keypoints_xyzrgb);

    if (nm_ft)
      {
        pcl::PointCloud<pcl::Normal>::Ptr normals (new  pcl::PointCloud<pcl::Normal>);
        pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> nm_est;
        nm_est.setSearchMethod (pcl::search::Search<pcl::PointXYZRGB>::Ptr (new pcl::search::KdTree<pcl::PointXYZRGB>));
        nm_est.setRadiusSearch (0.01);
        nm_est.setInputCloud (points);
        nm_est.compute (*normals);
        nm_ft->setInputNormals(normals);
    }

    ft_descriptor->compute(*descriptors_out);
}

void find_feature_correspondences (pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr &source_descriptors,
                              pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr &target_descriptors,
                              std::vector<int> &correspondences_out){

  correspondences_out.resize (source_descriptors->size ());
  pcl::KdTreeFLANN<pcl::PFHRGBSignature250> descriptor_kdtree;
  descriptor_kdtree.setInputCloud (target_descriptors);
  const int k = 1;
  std::vector<int> k_indices (k);
  std::vector<float> k_squared_distances (k);

  for (size_t i = 0; i < source_descriptors->size (); ++i){
    descriptor_kdtree.nearestKSearch (*source_descriptors, i, k, k_indices, k_squared_distances);
    correspondences_out[i] = k_indices[0];
  }
}


void detect_keypoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points, pcl::PointCloud<pcl::PointXYZI>::Ptr &keypoints){
    pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointXYZI> sift;
    sift.setSearchMethod(pcl::search::KdTree<pcl::PointXYZRGB>::Ptr (new pcl::search::KdTree<pcl::PointXYZRGB>));
    const float min_scale = 0.01f;
    const int nr_octaves = 3;
    const int nr_scales_per_octave = 2;
    const float min_contrast = 0.0;
    sift.setScales(min_scale,nr_octaves,nr_scales_per_octave);
    sift.setMinimumContrast(min_contrast);
    sift.setInputCloud(points);
    sift.setSearchSurface(points);
    sift.compute(*keypoints);
    cout << "Detected keypoints: " << keypoints->size() << "\n";
    //visualize_keypoints(points,keypoints);
}

Eigen::Matrix4f rigidTransformation(pcl::PointCloud<pcl::PointXYZI>::Ptr &nextKeypoints, pcl::PointCloud<pcl::PointXYZI>::Ptr &currKeypoints, boost::shared_ptr<pcl::Correspondences> &corresp){
  Eigen::Matrix4f transform; 
  pcl::registration::TransformationEstimationSVD<pcl::PointXYZI, pcl::PointXYZI> transformSVD;
  transformSVD.estimateRigidTransformation (*nextKeypoints, *currKeypoints, *corresp, transform); 
  return transform;
}

bool check_arguments(int argc, char** argv){
	string argumento = "";

    if (argc == 2){
        argumento = argv[1];
        if (argumento == "--o")
            PCD_FILE_PATH = "src/turtlebotto/get_pointclouds/src/pcd_files/original/pcd_";
        else if (argumento == "--s")
            PCD_FILE_PATH = "src/turtlebotto/get_pointclouds/src/pcd_files/sor/sor_pcd_";
        else if (argumento == "--sv")
            PCD_FILE_PATH = "src/turtlebotto/get_pointclouds/src/pcd_files/sor_filtered/sor_filtered_pcd_";
        else if (argumento == "--v")
            PCD_FILE_PATH = "src/turtlebotto/get_pointclouds/src/pcd_files/filtered/filtered_pcd_";
		return true;
    }
    else{
			cout << argc;
			string usage = "{--o (Original pcds) | --s (SOR pcds) | --v (VoxelGrid pcds)}";
			cout << "Incorrect program use! Usage must be: " << endl;
			cout << "rosrun *package_name* *executable_name* " << usage << endl;
			return false;
    }
}

void callback(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& msg){
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>(*msg));
	pcl::VoxelGrid<pcl::PointXYZRGB> vGrid;
	vGrid.setInputCloud (cloud);
	vGrid.setLeafSize (0.05f, 0.05f, 0.05f);
	vGrid.filter (*cloud);
	cout << "Voxel grid applied" << endl;
	std::vector<int> indices;
	cloud->is_dense = false;
	pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
	nextCloud = cloud;
}
void calculate_ICP(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &dstCloud,pcl::PointCloud<pcl::PointXYZRGB>::Ptr &final_cloud,pcl::PointCloud<pcl::PointXYZRGB>::Ptr currCloud){
	pcl::Registration<pcl::PointXYZRGB, pcl::PointXYZRGB>::Ptr registration (new pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB>);
	registration->setInputSource(dstCloud);
	registration->setInputTarget(currCloud);
	registration->setMaxCorrespondenceDistance(0.05);
	registration->setRANSACOutlierRejectionThreshold (0.05);
	registration->setTransformationEpsilon (0.000001);
	registration->setMaximumIterations (1000);
	registration->align(*final_cloud);
  
}
int main (int argc, char** argv){
    ros::init(argc, argv, "mapping_3d_node");
	boost::thread t(simpleVis);
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZRGB>>("/camera/depth/points",1,callback);
	   
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr currCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr currKeypoints (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PFHRGBSignature250>::Ptr currDescriptors (new pcl::PointCloud<pcl::PFHRGBSignature250>);
    

	pcl::PCDWriter writer;

	if(check_arguments(argc,argv)){
		ft_descriptor->setKSearch(50);
		while(ros::ok()){
			ros::spinOnce();

			if (nextCloud->size() == 0)
				continue;

			if(!INITIALIZED){ 
				currCloud = nextCloud;
			}

			cout << "CLOUD SIZE = " << currCloud->size() << endl;
			cout << "NEXT CLOUD SIZE = " << nextCloud->size() << endl;
			
			// Detect keypoints
			cout << "Detecting keypoints in " << "... " << endl;
			detect_keypoints(currCloud, currKeypoints);
			cout << "Detecting keypoints in " << "... " << endl;
			detect_keypoints(nextCloud, nextKeypoints);
			
			// Compute PFH features
			cout << "Computing PFH features " << "..." << endl;
			detect_descriptors (currCloud, currKeypoints, currDescriptors);
			cout << "Computing PFH features " << "..." << endl;
			detect_descriptors (nextCloud, nextKeypoints, nextDescriptors);
			
			// Find feature correspondences
			std::vector<int> ctnCorrespondences;
			std::vector<int> ntcCorrespondences;
			cout << "Find feature correspondences" << endl;
			find_feature_correspondences (currDescriptors, nextDescriptors, ctnCorrespondences);
			find_feature_correspondences (nextDescriptors, currDescriptors, ntcCorrespondences);
			cout << ctnCorrespondences.size();
			cout << "Filter feature correspondences" << endl;
			correspondences_filter(currKeypoints, nextKeypoints,ctnCorrespondences,ntcCorrespondences);

			pcl::PointCloud<pcl::PointXYZRGB>::Ptr dstCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
			Eigen::Matrix4f transf_matrix = rigidTransformation(currKeypoints, nextKeypoints, ransac_corr);
			pcl::transformPointCloud (*currCloud, *dstCloud, transf_matrix);

			// Applying ICP
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr transfCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
			calculate_ICP(dstCloud,transfCloud,currCloud);
			
			if(finalCloud->points.size()==0) {
				*finalCloud = *transfCloud;
				*currCloud = *transfCloud;	
			}
			else {
				*finalCloud += *transfCloud;
				*currCloud = *transfCloud;   
			}

			INITIALIZED = true;
			//Write final scene
			//writer.write<pcl::PointXYZRGB> (FINAL_PATH, *finalCloud, false);
		}
	}
    
  //***************************************************************///

  
  return (0);
}
