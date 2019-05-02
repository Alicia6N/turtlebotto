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
using namespace std;

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

boost::shared_ptr<pcl::Correspondences>  correspondences_filter(pcl::PointCloud<pcl::PFHSignature125>::Ptr &desc_pfh_1, 
                    pcl::PointCloud<pcl::PFHSignature125>::Ptr &desc_pfh_2,pcl::PointCloud<pcl::PointWithScale>::Ptr keypoint1,
                    pcl::PointCloud<pcl::PointWithScale>::Ptr keypoint2){

	boost::shared_ptr<pcl::Correspondences> cor_all (new pcl::Correspondences);
	boost::shared_ptr<pcl::Correspondences> cor_inliers (new pcl::Correspondences);

	pcl::registration::CorrespondenceEstimation<pcl::PFHSignature125, pcl::PFHSignature125> corEst;
	pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointWithScale> sac;

	corEst.setInputSource (desc_pfh_1);
	corEst.setInputTarget (desc_pfh_2);
	corEst.determineReciprocalCorrespondences (*cor_all);

	sac.setInputSource (keypoint1);
	sac.setInputTarget (keypoint2);
	sac.setInlierThreshold (0.1);
	sac.setMaximumIterations (1500);
	sac.setInputCorrespondences (cor_all);
	sac.getCorrespondences (*cor_inliers);

	return cor_inliers;
}

void compute_PFH_features_at_keypoints (pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points,
                                   pcl::PointCloud<pcl::Normal>::Ptr &normals,
                                   pcl::PointCloud<pcl::PointWithScale>::Ptr &keypoints, float feature_radius,
                                   pcl::PointCloud<pcl::PFHSignature125>::Ptr &descriptors_out){
  // Create a PFHEstimation object
  pcl::PFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::PFHSignature125> pfh_est;
  // Set it to use a FLANN-based KdTree to perform its neighborhood searches
  pfh_est.setSearchMethod (pcl::search::KdTree<pcl::PointXYZRGB>::Ptr (new pcl::search::KdTree<pcl::PointXYZRGB>));
  // Specify the radius of the PFH feature
  pfh_est.setRadiusSearch (feature_radius);
  /* This is a little bit messy: since our keypoint detection returns PointWithScale points, but we want to
   * use them as an input to our PFH estimation, which expects clouds of PointXYZRGB points.  To get around this,
   * we'll use copyPointCloud to convert "keypoints" (a cloud of type PointCloud<PointWithScale>) to
   * "keypoints_xyzrgb" (a cloud of type PointCloud<PointXYZRGB>).  Note that the original cloud doesn't have any RGB
   * values, so when we copy from PointWithScale to PointXYZRGB, the new r,g,b fields will all be zero.
   */

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints_xyzrgb (new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::copyPointCloud (*keypoints, *keypoints_xyzrgb);
  // Use all of the points for analyzing the local structure of the cloud
  pfh_est.setSearchSurface (points);
  pfh_est.setInputNormals (normals);
  // But only compute features at the keypoints
  pfh_est.setInputCloud (keypoints_xyzrgb);
  // Compute the features
  pfh_est.compute (*descriptors_out);
}

void find_feature_correspondences (pcl::PointCloud<pcl::PFHSignature125>::Ptr &source_descriptors,
                              pcl::PointCloud<pcl::PFHSignature125>::Ptr &target_descriptors,
                              std::vector<int> &correspondences_out, std::vector<float> &correspondence_scores_out){
  // Resize the output vector
  correspondences_out.resize (source_descriptors->size ());
  correspondence_scores_out.resize (source_descriptors->size ());
  // Use a KdTree to search for the nearest matches in feature space
  pcl::search::KdTree<pcl::PFHSignature125> descriptor_kdtree;
  descriptor_kdtree.setInputCloud (target_descriptors);
  // Find the index of the best match for each keypoint, and store it in "correspondences_out"
  const int k = 1;
  std::vector<int> k_indices (k);
  std::vector<float> k_squared_distances (k);
  for (size_t i = 0; i < source_descriptors->size (); ++i){
    descriptor_kdtree.nearestKSearch (*source_descriptors, i, k, k_indices, k_squared_distances);
    correspondences_out[i] = k_indices[0];
    correspondence_scores_out[i] = k_squared_distances[0];
  }
}

void compute_surface_normals (pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points, float normal_radius,
                         pcl::PointCloud<pcl::Normal>::Ptr &normals_out){
  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> norm_est;
  // Use a FLANN-based KdTree to perform neighborhood searches
  //norm_est.setSearchMethod (pcl::KdTreeFLANN<pcl::PointXYZRGB>::Ptr (new pcl::KdTreeFLANN<pcl::PointXYZRGB>));
  norm_est.setSearchMethod (pcl::search::KdTree<pcl::PointXYZRGB>::Ptr (new pcl::search::KdTree<pcl::PointXYZRGB>));
  // Specify the size of the local neighborhood to use when computing the surface normals
  norm_est.setRadiusSearch (normal_radius);
  // Set the input points
  norm_est.setInputCloud (points);
  // Estimate the surface normals and store the result in "normals_out"
  norm_est.compute (*normals_out);
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
    //visualize_keypoints(points,keypoints);
}

Eigen::Matrix4f rigidTransformation(pcl::PointCloud<pcl::PointWithScale>::Ptr &nextKeypoints, pcl::PointCloud<pcl::PointWithScale>::Ptr &currKeypoints, boost::shared_ptr<pcl::Correspondences> &corresp){

  Eigen::Matrix4f transform; 
  pcl::registration::TransformationEstimationSVD<pcl::PointWithScale, pcl::PointWithScale> transformSVD;
  transformSVD.estimateRigidTransformation (*nextKeypoints, *currKeypoints, *corresp, transform); 

  return transform;
}
int main (int argc, char** argv){
    ros::init(argc, argv, "mapping_3d_node");
    // Pevious, current and accumulated cloud
	   
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr currCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::Normal>::Ptr currNormals (new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::PointWithScale>::Ptr currKeypoints (new pcl::PointCloud<pcl::PointWithScale>);
    pcl::PointCloud<pcl::PFHSignature125>::Ptr currDescriptors (new pcl::PointCloud<pcl::PFHSignature125>);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr nextCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::Normal>::Ptr nextNormals (new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::PointWithScale>::Ptr nextKeypoints (new pcl::PointCloud<pcl::PointWithScale>);
    pcl::PointCloud<pcl::PFHSignature125>::Ptr nextDescriptors (new pcl::PointCloud<pcl::PFHSignature125>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr finalCloud (new pcl::PointCloud<pcl::PointXYZRGB>);

    cout << "Created pointcloud variables" << endl;
    int id = 0;
    string pcd_file_path = "";
	string argumento = "";
    string final_path = "src/turtlebotto/mapping_3d/src/final_cloud.pcd";
    if (argc == 2){
        argumento = argv[1];
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
			return 0;
    }

    cout << "PCD flags set" << endl;
    pcl::PCDReader reader; 

	reader.read<pcl::PointXYZRGB> (pcd_file_path + std::to_string(id) + ".pcd", *currCloud);
    while(ros::ok()){
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr dstCloud (new pcl::PointCloud<pcl::PointXYZRGB>);
        //Read two pcd files.
        //reader.read<pcl::PointXYZRGB> (pcd_file_path + std::to_string(id) + ".pcd", *currCloud);
		//cout << "Read:" << reader.read<pcl::PointXYZRGB> (pcd_file_path + std::to_string(id+1) + ".pcd", *nextCloud) << endl;
        if(reader.read<pcl::PointXYZRGB> (pcd_file_path + std::to_string(id+1) + ".pcd", *nextCloud) != 0) {
          cout << "Empty" << endl;
          break;
        }
        // Compute surface normals
        const float normal_radius = 0.03;
        cout << "Computing normals for " << id << "..." << endl;
        compute_surface_normals (currCloud, normal_radius, currNormals);
        cout << "Computing normals for " << id+1 << "..." << endl;
        compute_surface_normals (nextCloud, normal_radius, nextNormals);

        //visualize_normals (fullCloud, currCloud, currNormals);

        // Detect keypoints
        cout << "Detecting keypoints in " << id << "... " << endl;
        detect_keypoints(currCloud, currKeypoints);
        cout << "Detecting keypoints in " << id+1 << "... " << endl;
        detect_keypoints(nextCloud, nextKeypoints);
        
        // Compute PFH features
        const float feature_radius = 0.08;
        cout << "Computing PFH features " << id << "..." << endl;
        compute_PFH_features_at_keypoints (currCloud, currNormals, currKeypoints, feature_radius, currDescriptors);
        cout << "Computing PFH features " << id+1 << "..." << endl;
        compute_PFH_features_at_keypoints (nextCloud, nextNormals, nextKeypoints, feature_radius, nextDescriptors);
        
        // Find feature correspondences
        std::vector<int> correspondences;
        std::vector<float> correspondence_scores;
        cout << "Find feature correspondences" << endl;
        find_feature_correspondences (currDescriptors, nextDescriptors, correspondences, correspondence_scores);
        cout << "Filter feature correspondences" << endl;
        //visualize_correspondences (currCloud, currKeypoints, nextCloud, nextKeypoints, correspondences, correspondence_scores);
        boost::shared_ptr<pcl::Correspondences> corresp = correspondences_filter(currDescriptors, nextDescriptors, currKeypoints, nextKeypoints);

        Eigen::Matrix4f transf_matrix = rigidTransformation(nextKeypoints, currKeypoints, corresp);
        pcl::transformPointCloud (*nextCloud, *dstCloud, transf_matrix);

		if(id==0) {
			*finalCloud = *dstCloud;	
			*currCloud = *dstCloud;	
		}
		else {
			*finalCloud += *dstCloud;
			*currCloud = *dstCloud;        
		}

        id++;
    }
    //***************************************************************///

    pcl::PCDWriter writer;
    //Write final scene
    writer.write<pcl::PointXYZRGB> (final_path, *finalCloud, false);
    return (0);
}
