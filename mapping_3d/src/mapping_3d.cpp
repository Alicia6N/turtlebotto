#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include "pcl/keypoints/sift_keypoint.h"
#include "pcl/features/pfh.h"
#include "pcl/visualization/pcl_visualizer.h"
#include <string>
#include <sstream>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <Eigen/Core>
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
string PCD_FILE_PATH = "";
pcl::Feature<pcl::PointXYZRGB, pcl::PFHSignature125>::Ptr ft_descriptor (new pcl::PFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::PFHSignature125>);


void visualize_keypoints (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr points,
                          const pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints){
  cout << ">>>>> Keypoints detected: " << keypoints->size() << endl;
  // Add the points to the vizualizer
  pcl::visualization::PCLVisualizer viz;
  viz.addPointCloud (points, "points");
  // Draw each keypoint as a sphere 

  for (size_t i = 0; i < keypoints->size (); ++i){
    // Get the point data
    const pcl::PointWithScale & p = keypoints->points[i];
    // Pick the radius of the sphere *
    float r =  p.scale;
    // * Note: the scale is given as the standard deviation of a Gaussian blur, so a
    //   radius of 2*p.scale is a good illustration of the extent of the keypoint
    // Generate a unique string for each sphere
    stringstream ss ("keypoint");
    ss << i;
    // Add a sphere at the keypoint
    viz.addSphere (p, 0.1*p.scale, 1.0, 0.0, 0.0, ss.str ());
  }
  // Give control over to the visualizer
  viz.spin ();
}

void visualize_normals (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr points,
                        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr normal_points,
                        const pcl::PointCloud<pcl::Normal>::Ptr normals){
  // Add the points and normals to the vizualizer
  pcl::visualization::PCLVisualizer viz;
  viz.addPointCloud (points, "points");
  viz.addPointCloud (normal_points, "normal_points");
  viz.addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (normal_points, normals, 1, 0.01, "normals");
  // Give control over to the visualizer
  viz.spin ();
}

void visualize_correspondences (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr points1,
                                const pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints1,
                                const pcl::PointCloud<pcl::PointXYZRGB>::Ptr points2,
                                const pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints2,
                                const std::vector<int> &correspondences,
                                const std::vector<float> &correspondence_scores){
  // We want to visualize two clouds side-by-side, so do to this, we'll make copies of the clouds and transform them
  // by shifting one to the left and the other to the right.  Then we'll draw lines between the corresponding points
  // Create some new point clouds to hold our transformed data
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_left (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints_left (new pcl::PointCloud<pcl::PointWithScale>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr points_right (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointWithScale>::Ptr keypoints_right (new pcl::PointCloud<pcl::PointWithScale>);
  // Shift the first clouds' points to the left
  //const Eigen::Vector3f translate (0.0, 0.0, 0.3);
  const Eigen::Vector3f translate (0.4, 0.0, 0.0);
  const Eigen::Quaternionf no_rotation (0, 0, 0, 0);
  pcl::transformPointCloud (*points1, *points_left, -translate, no_rotation);
  pcl::transformPointCloud (*keypoints1, *keypoints_left, -translate, no_rotation);
  // Shift the second clouds' points to the right
  pcl::transformPointCloud (*points2, *points_right, translate, no_rotation);
  pcl::transformPointCloud (*keypoints2, *keypoints_right, translate, no_rotation);
  // Add the clouds to the vizualizer
  pcl::visualization::PCLVisualizer viz;
  viz.addPointCloud (points_left, "points_left");
  viz.addPointCloud (points_right, "points_right");
  // Compute the median correspondence score
  std::vector<float> temp (correspondence_scores);
  std::sort (temp.begin (), temp.end ());
  float median_score = temp[temp.size ()/2];
  // Draw lines between the best corresponding points
  int kpsize = 0;

  for (size_t i = 0; i < keypoints_left->size (); ++i){
    if (correspondence_scores[i] > median_score){
      continue; // Don't draw weak correspondences
    }
    kpsize++;
    // Get the pair of points
    const pcl::PointWithScale & p_left = keypoints_left->points[i];
    const pcl::PointWithScale & p_right = keypoints_right->points[correspondences[i]];
    // Generate a random (bright) color
    double r = (rand() % 100);
    double g = (rand() % 100);
    double b = (rand() % 100);
    double max_channel = std::max (r, std::max (g, b));
    r /= max_channel;
    g /= max_channel;
    b /= max_channel;
    // Generate a unique string for each line
    std::stringstream ss ("line");
    ss << i;
    // Draw the line
    viz.addLine (p_left, p_right, r, g, b, ss.str ());
  }

  cout << "Correspondences size: " << kpsize << endl;
  // Give control over to the visualizer
  viz.spin ();
}

pcl::CorrespondencesPtr correspondences_filter(pcl::PointCloud<pcl::PointWithScale>::Ptr keypoint1,
                    pcl::PointCloud<pcl::PointWithScale>::Ptr keypoint2, vector<int>& ctnCorrespondences, vector<int>& ntcCorrespondences){

	//boost::shared_ptr<pcl::Correspondences> cor_all (new pcl::Correspondences);
	//boost::shared_ptr<pcl::Correspondences> cor_inliers (new pcl::Correspondences);
	std::vector<std::pair<unsigned, unsigned>> corr_pair;
	//pcl::registration::CorrespondenceEstimation<pcl::PFHSignature125, pcl::PFHSignature125> corEst;
	//pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointWithScale> sac;
	pcl::CorrespondencesPtr ransac_corr(new pcl::Correspondences);
	/*corEst.setInputSource (desc_pfh_1);
	corEst.setInputTarget (desc_pfh_2);
	corEst.determineReciprocalCorrespondences (*cor_all);*/

	 for (unsigned i = 0; i < ntcCorrespondences.size (); ++i) {
        if (ctnCorrespondences[ntcCorrespondences[i]] == i) {
            corr_pair.push_back(std::make_pair(i, ntcCorrespondences[i]));
        }
    }

    ransac_corr->resize (corr_pair.size());
    for (unsigned i = 0; i < corr_pair.size(); ++i)
    {
        (*ransac_corr)[i].index_query = corr_pair[i].first;
        (*ransac_corr)[i].index_match = corr_pair[i].second;
    }

    
    pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointWithScale> ransac;
    ransac.setInputSource(keypoint1);
    ransac.setInputTarget(keypoint2);
    ransac.setInputCorrespondences(ransac_corr);
    ransac.getCorrespondences(*ransac_corr);
    return ransac_corr;
}

void detect_descriptors (pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points,
                                   pcl::PointCloud<pcl::PointWithScale>::Ptr &keypoints,
                                   pcl::PointCloud<pcl::PFHSignature125>::Ptr &descriptors_out){
  // Create a PFHEstimation object
  /*pcl::PFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::PFHSignature125> pfh_est;
  // Set it to use a FLANN-based KdTree to perform its neighborhood searches
  pfh_est.setSearchMethod (pcl::search::KdTree<pcl::PointXYZRGB>::Ptr (new pcl::search::KdTree<pcl::PointXYZRGB>));
  // Specify the radius of the PFH feature
  pfh_est.setRadiusSearch (FEATURE_RADIUS);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_xyzrgb (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::copyPointCloud (*keypoints, *keypoints_xyzrgb);
  pfh_est.setSearchSurface (points);
  pfh_est.setInputNormals (normals);
  pfh_est.setInputCloud (keypoints_xyzrgb);
  pfh_est.compute (*descriptors_out);*/
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_xyzrgb(new pcl::PointCloud<pcl::PointXYZRGB>);
    keypoints_xyzrgb->points.resize(keypoints->points.size());

    // We need to transfer the type
    pcl::copyPointCloud(*keypoints, *keypoints_xyzrgb);

    // Try to cast its type 
    pcl::FeatureFromNormals<pcl::PointXYZRGB, pcl::Normal, pcl::PFHSignature125>::Ptr nm_ft = boost::dynamic_pointer_cast<pcl::FeatureFromNormals<pcl::PointXYZRGB, pcl::Normal, pcl::PFHSignature125>> (ft_descriptor);

    // Setting main descriptor properties
	ft_descriptor->setKSearch(50);
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

void find_feature_correspondences (pcl::PointCloud<pcl::PFHSignature125>::Ptr &source_descriptors,
                              pcl::PointCloud<pcl::PFHSignature125>::Ptr &target_descriptors,
                              std::vector<int> &correspondences_out){

  correspondences_out.resize (source_descriptors->size ());
  pcl::search::KdTree<pcl::PFHSignature125> descriptor_kdtree;
  descriptor_kdtree.setInputCloud (target_descriptors);
  const int k = 1;
  pcl::PointCloud<pcl::PFHSignature125>::Ptr outputCloud;
  std::vector<int> k_indices (k);
  std::vector<float> k_squared_distances (k);

  for (size_t i = 0; i < source_descriptors->size (); ++i){
    descriptor_kdtree.nearestKSearch (*source_descriptors, i, k, k_indices, k_squared_distances);
    correspondences_out[i] = k_indices[0];
  }
}

void compute_surface_normals (pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points,
                         pcl::PointCloud<pcl::Normal>::Ptr &normals_out){

  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> norm_est;
  norm_est.setSearchMethod (pcl::search::KdTree<pcl::PointXYZRGB>::Ptr (new pcl::search::KdTree<pcl::PointXYZRGB>));
  norm_est.setRadiusSearch (NORMAL_RADIUS);
  norm_est.setInputCloud (points);
  norm_est.compute (*normals_out);
}

void detect_keypoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points, pcl::PointCloud<pcl::PointWithScale>::Ptr &keypoints){
    pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointWithScale> sift;
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
    //visualize_keypoints(points,keypoints);
}

Eigen::Matrix4f rigidTransformation(pcl::PointCloud<pcl::PointWithScale>::Ptr &nextKeypoints, pcl::PointCloud<pcl::PointWithScale>::Ptr &currKeypoints, boost::shared_ptr<pcl::Correspondences> &corresp){
  Eigen::Matrix4f transform; 
  pcl::registration::TransformationEstimationSVD<pcl::PointWithScale, pcl::PointWithScale> transformSVD;
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

int main (int argc, char** argv){
    ros::init(argc, argv, "mapping_3d_node");
	   
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr currCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointWithScale>::Ptr currKeypoints (new pcl::PointCloud<pcl::PointWithScale>);
    pcl::PointCloud<pcl::PFHSignature125>::Ptr currDescriptors (new pcl::PointCloud<pcl::PFHSignature125>);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr nextCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointWithScale>::Ptr nextKeypoints (new pcl::PointCloud<pcl::PointWithScale>);
    pcl::PointCloud<pcl::PFHSignature125>::Ptr nextDescriptors (new pcl::PointCloud<pcl::PFHSignature125>);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr finalCloud (new pcl::PointCloud<pcl::PointXYZRGB>);

    cout << "Created pointcloud variables" << endl;
    int id = 3;
	if(check_arguments(argc,argv)){
		cout << "PCD flags set" << endl;
		pcl::PCDReader reader; 

		reader.read<pcl::PointXYZRGB> (PCD_FILE_PATH + std::to_string(id) + ".pcd", *currCloud);
		while(ros::ok()){
			if(reader.read<pcl::PointXYZRGB> (PCD_FILE_PATH + std::to_string(id+1) + ".pcd", *nextCloud) != 0) {
				break;
			}
			// Compute surface normals
			std::vector<int> indices;
			pcl::removeNaNFromPointCloud(*currCloud, *currCloud, indices);
			pcl::removeNaNFromPointCloud(*nextCloud, *nextCloud, indices);
			// Detect keypoints
			cout << "Detecting keypoints in " << id << "... " << endl;
			detect_keypoints(currCloud, currKeypoints);
			cout << "Detecting keypoints in " << id+1 << "... " << endl;
			detect_keypoints(nextCloud, nextKeypoints);
			
			// Compute PFH features

			cout << "Computing PFH features " << id << "..." << endl;
			detect_descriptors (currCloud, currKeypoints, currDescriptors);
			cout << "Computing PFH features " << id+1 << "..." << endl;
			detect_descriptors (nextCloud, nextKeypoints, nextDescriptors);
			
			// Find feature correspondences
			std::vector<int> ctnCorrespondences;
			std::vector<int> ntcCorrespondences;
			cout << "Find feature correspondences" << endl;
			find_feature_correspondences (currDescriptors, nextDescriptors, ctnCorrespondences);
			find_feature_correspondences (nextDescriptors, currDescriptors, ntcCorrespondences);
			cout << ctnCorrespondences.size();
			cout << "Filter feature correspondences" << endl;
			pcl::CorrespondencesPtr corresp = correspondences_filter(nextKeypoints, currKeypoints,ctnCorrespondences,ntcCorrespondences);

			pcl::PointCloud<pcl::PointXYZRGB>::Ptr dstCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
			Eigen::Matrix4f transf_matrix = rigidTransformation(nextKeypoints, currKeypoints, corresp);
			pcl::transformPointCloud (*nextCloud, *dstCloud, transf_matrix);

			// Applying ICP
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
			pcl::Registration<pcl::PointXYZRGB, pcl::PointXYZRGB>::Ptr registration (new pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB>);
			registration->setInputSource(dstCloud);
			registration->setInputTarget(currCloud);
			registration->setMaxCorrespondenceDistance(0.05);
			registration->setRANSACOutlierRejectionThreshold (0.001);
			registration->setTransformationEpsilon (0.000001);
			registration->setMaximumIterations (1000);
			registration->align(*final_transformed_cloud);
			
			if(finalCloud == nullptr) {
			*finalCloud = *final_transformed_cloud;	
				continue;
			}
			*finalCloud += *final_transformed_cloud;
			*currCloud = *final_transformed_cloud;        
			id++;
		}
	}
    
  //***************************************************************///

  pcl::PCDWriter writer;
  //Write final scene
  writer.write<pcl::PointXYZRGB> (FINAL_PATH, *finalCloud, false);
  return (0);
}
