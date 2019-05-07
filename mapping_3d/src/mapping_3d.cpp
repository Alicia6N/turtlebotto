#pragma once

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <boost/foreach.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/pfhrgb.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/transformation_estimation_svd.h>
using namespace std;
using PointCloudXYZRGB = pcl::PointCloud<pcl::PointXYZRGB>;
using PointCloudXYZI = pcl::PointCloud<pcl::PointXYZI>;
using PFHRGBSign250 = pcl::PointCloud<pcl::PFHRGBSignature250>;

//Parametros globales
string ARGUMENTO_PCL;
string ARGUMENTO_KP;
bool INITIALIZED = false;

PointCloudXYZRGB::Ptr prevCloud (new PointCloudXYZRGB);
PointCloudXYZRGB::Ptr currCloud (new PointCloudXYZRGB);
PointCloudXYZRGB::Ptr finalCloud (new PointCloudXYZRGB);

pcl::Feature<pcl::PointXYZRGB, pcl::PFHRGBSignature250>::Ptr descriptor_detector (new pcl::PFHRGBEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::PFHRGBSignature250>);
boost::shared_ptr<pcl::Keypoint<pcl::PointXYZRGB, pcl::PointXYZI>> keypoint_detector;

pcl::CorrespondencesPtr ransac_corr(new pcl::Correspondences);

void simpleVis (){
  	pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
	while(!viewer.wasStopped()){
		viewer.showCloud(finalCloud);
		boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
	}
}

void compute_keypoints(PointCloudXYZRGB::ConstPtr source, PointCloudXYZI::Ptr keypoints){
	if (ARGUMENTO_KP == "--s3d"){
		//sift3d
		pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointXYZI>* sift = new pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointXYZI>;
		pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB> ());
		sift->setSearchMethod(tree);
		sift->setScales(0.01, 3, 2);
		sift->setMinimumContrast(0.0);
		keypoint_detector.reset(sift);
	}
	else if (ARGUMENTO_KP == "--h3d"){
		//harris3d
		pcl::HarrisKeypoint3D<pcl::PointXYZRGB,pcl::PointXYZI>* harris3D = new pcl::HarrisKeypoint3D<pcl::PointXYZRGB,pcl::PointXYZI> (pcl::HarrisKeypoint3D<pcl::PointXYZRGB,pcl::PointXYZI>::HARRIS);
		harris3D->setNonMaxSupression(true);
		harris3D->setRadius (0.01);
		harris3D->setRadiusSearch (0.01);
		keypoint_detector.reset(harris3D);
	}
	
	keypoint_detector->setInputCloud(source);
	keypoint_detector->setSearchSurface(source);
	keypoint_detector->compute(*keypoints);
}

void compute_descriptors(PointCloudXYZRGB::ConstPtr source, PointCloudXYZI::Ptr keypoints, PFHRGBSign250::Ptr features) {
	PointCloudXYZRGB::Ptr keypoints_xyzrgb(new PointCloudXYZRGB);
	keypoints_xyzrgb->points.resize(keypoints->points.size());

	pcl::copyPointCloud(*keypoints, *keypoints_xyzrgb);

	pcl::FeatureFromNormals<pcl::PointXYZRGB, pcl::Normal, pcl::PFHRGBSignature250>::Ptr nm_ft = boost::dynamic_pointer_cast<pcl::FeatureFromNormals<pcl::PointXYZRGB, pcl::Normal, pcl::PFHRGBSignature250>> (descriptor_detector);

	

	if (nm_ft){
		pcl::PointCloud<pcl::Normal>::Ptr normals (new  pcl::PointCloud<pcl::Normal>);
		pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> nm_est;
		nm_est.setSearchMethod (pcl::search::Search<pcl::PointXYZRGB>::Ptr (new pcl::search::KdTree<pcl::PointXYZRGB>));
		nm_est.setRadiusSearch (0.01);
		nm_est.setInputCloud (source);
		nm_est.compute (*normals);
		nm_ft->setInputNormals(normals);
	}

	descriptor_detector->setKSearch(50);
	descriptor_detector->setSearchSurface(source);
	descriptor_detector->setInputCloud(keypoints_xyzrgb);
	descriptor_detector->compute(*features);
}

void find_feature_correspondences(PFHRGBSign250::Ptr source, PFHRGBSign250::Ptr target, std::vector<int>& correspondences){
  	correspondences.resize(source->size());
	pcl::KdTreeFLANN<pcl::PFHRGBSignature250> descriptor_kdtree;
	descriptor_kdtree.setInputCloud(target);

	const int k = 1;
	std::vector<int> k_indices(k);
	std::vector<float> k_squared_distances(k);
	for (size_t i = 0; i < source->size (); ++i) {
		descriptor_kdtree.nearestKSearch(*source, i, k, k_indices, k_squared_distances);
		correspondences[i] = k_indices[0];
	}
}


void filter_correspondences(PointCloudXYZI::Ptr source_kp, PointCloudXYZI::Ptr target_kp,
							std::vector<int>& st_correspondences, std::vector<int>& ts_correspondences) {
	std::vector<std::pair<unsigned, unsigned>> corr_pair;
	
	for (unsigned i = 0; i < st_correspondences.size (); ++i) {
		if (ts_correspondences[st_correspondences[i]] == i) {
			corr_pair.push_back(std::make_pair(i, st_correspondences[i]));
		}
	}

	ransac_corr->resize (corr_pair.size());
	for (unsigned i = 0; i < corr_pair.size(); ++i){
		(*ransac_corr)[i].index_query = corr_pair[i].first;
		(*ransac_corr)[i].index_match = corr_pair[i].second;
	}

	pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZI> ransac;
	ransac.setInputSource(source_kp);
	ransac.setInputTarget(target_kp);
	ransac.setInputCorrespondences(ransac_corr);
	ransac.getCorrespondences(*ransac_corr);
}

void calculate_ICP(PointCloudXYZRGB::Ptr &dstCloud, PointCloudXYZRGB::Ptr prevCloud, PointCloudXYZRGB::Ptr &finalCloud){
	PointCloudXYZRGB::Ptr final_transformed_cloud(new PointCloudXYZRGB);
	pcl::Registration<pcl::PointXYZRGB, pcl::PointXYZRGB>::Ptr registration (new pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB>);
	registration->setInputSource(dstCloud);
	registration->setInputTarget(prevCloud);
	registration->setMaxCorrespondenceDistance(0.05);
	registration->setRANSACOutlierRejectionThreshold (0.05);
	registration->setTransformationEpsilon (0.000001);
	registration->setMaximumIterations (1000);
	registration->align(*finalCloud);
}

void rigidTransformation(PointCloudXYZRGB::Ptr &source,PointCloudXYZI::Ptr &currKp, PointCloudXYZI::Ptr &prevKp, PointCloudXYZRGB::Ptr &dstCloud){
	Eigen::Matrix4f transform;
	pcl::registration::TransformationEstimationSVD<pcl::PointXYZI, pcl::PointXYZI> transformSVD;
  	transformSVD.estimateRigidTransformation (*currKp, *prevKp, *ransac_corr, transform);
	pcl::transformPointCloud(*source, *dstCloud, transform);
}

void scene_reconstruction(PointCloudXYZRGB::Ptr source, PointCloudXYZRGB::Ptr target, int i){
    cout << "Cloud " << i << ". Captured points: " << currCloud->size() << endl;

	//!!!!!!!!!!!!!!!!!!
	//SOURCE = CURRCLOUD
	//TARGET = PREVCLOUD
	//!!!!!!!!!!!!!!!!!!

	// Keypoint detection
	PointCloudXYZI::Ptr source_kp(new PointCloudXYZI);
	PointCloudXYZI::Ptr target_kp(new PointCloudXYZI);
	compute_keypoints(source, source_kp);
	compute_keypoints(target, target_kp);

	// Computing descriptors
	PFHRGBSign250::Ptr source_ft(new PFHRGBSign250);
	PFHRGBSign250::Ptr target_ft(new PFHRGBSign250);
	compute_descriptors(source, source_kp, source_ft);
	compute_descriptors(target, target_kp, target_ft);

	// Deterministic function to find feature correspondences between both clouds
	std::vector<int> st_correspondences;
	std::vector<int> ts_correspondences;
	find_feature_correspondences(source_ft, target_ft, st_correspondences);
	find_feature_correspondences(target_ft, source_ft, ts_correspondences);

	// Discarding unlikely correspondences applying RANSAC
	filter_correspondences(source_kp, target_kp, st_correspondences, ts_correspondences);

	// First initial transform
	PointCloudXYZRGB::Ptr dstCloud(new PointCloudXYZRGB);
	rigidTransformation(source,source_kp,target_kp,dstCloud);
	// Applying ICP
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr transfCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	calculate_ICP(dstCloud, target, transfCloud);
	
	if(finalCloud->points.size() == 0) {
		*finalCloud = *transfCloud;  
		*prevCloud = *transfCloud;
	}
	else{
		*finalCloud += *transfCloud;
		*prevCloud = *transfCloud;
	}
}

bool check_arguments(int argc, char** argv){
	ARGUMENTO_PCL = "";
	ARGUMENTO_KP = "";

	bool ok = false;

    if (argc == 3){
        ARGUMENTO_PCL = argv[1];
		ARGUMENTO_KP = argv[2];

        if (ARGUMENTO_PCL == "--o")
            ok = true;
        else if (ARGUMENTO_PCL == "--s")
            ok = true;
        else if (ARGUMENTO_PCL == "--sv")
            ok = true;
        else if (ARGUMENTO_PCL == "--v")
            ok = true;

		if (ARGUMENTO_KP == "--h3d" && ok)
			return true;
		if (ARGUMENTO_KP == "--s3d" && ok)
			return true;
    }
    else{
		string usage = "{ [--o | --v | --s | --sv] [--h3d | --s3d]}";
		cout << "Incorrect program use! Usage must be: " << endl;
		cout << "rosrun *package_name* *executable_name* " << usage << endl;
		return false;
    }

	return false;
}

void apply_filters(PointCloudXYZRGB::Ptr &cloud){
	if (ARGUMENTO_PCL != "--o"){
		if (ARGUMENTO_PCL == "--v"){
			//voxelgrid point cloud
			pcl::VoxelGrid<pcl::PointXYZRGB> vGrid;
			vGrid.setInputCloud (cloud);
			vGrid.setLeafSize (0.05f, 0.05f, 0.05f);
			vGrid.filter (*cloud);
			return;
		}
		if (ARGUMENTO_PCL == "--s"){
			//sor point cloud
			pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
			sor.setInputCloud (cloud);
			sor.setMeanK (50);
			sor.setStddevMulThresh (1.0);
			sor.filter (*cloud);
			return;
		}
		if (ARGUMENTO_PCL == "--sv"){
			//sor voxelgrid point cloud
			pcl::VoxelGrid<pcl::PointXYZRGB> vGrid;
			pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
			sor.setInputCloud (cloud);
			sor.setMeanK (50);
			sor.setStddevMulThresh (1.0);
			sor.filter (*cloud);
			vGrid.setInputCloud (cloud);
			vGrid.setLeafSize (0.05f, 0.05f, 0.05f);
			vGrid.filter (*cloud);
			return;
		}
	}
}

void callback(const PointCloudXYZRGB::ConstPtr& msg){
	PointCloudXYZRGB::Ptr cloud (new PointCloudXYZRGB(*msg));
	currCloud = cloud;
	
	apply_filters(currCloud);
	
	std::vector<int> indices;
	currCloud->is_dense = false;
	pcl::removeNaNFromPointCloud(*currCloud, *currCloud, indices);	
}

int main(int argc, char** argv){
	if(check_arguments(argc, argv)){
		cout << "Arguments initialized with: " << ARGUMENTO_PCL << " and " << ARGUMENTO_KP << "." << endl;
		ros::init(argc, argv, "mapping_3d_node");
		ros::NodeHandle nh;
		ros::Subscriber sub = nh.subscribe<PointCloudXYZRGB>("/camera/depth/points", 1, callback);
		boost::thread t(simpleVis);

		int i = 0;
		
		while(ros::ok()){
			ros::spinOnce();			
			if (currCloud->size() == 0)
				continue;
			else if(!INITIALIZED)
				prevCloud = currCloud;

			scene_reconstruction(currCloud, prevCloud, i);
			
			INITIALIZED = true;
			i++;
		}
	}
}
