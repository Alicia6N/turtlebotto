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

#include <pcl/point_types.h>
#include <pcl/keypoints/impl/sift_keypoint.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <chrono>

using namespace std;
using PointCloudXYZRGB = pcl::PointCloud<pcl::PointXYZRGB>;
using PointCloudXYZI = pcl::PointCloud<pcl::PointXYZI>;
using PFHRGBSign250 = pcl::PointCloud<pcl::PFHRGBSignature250>;

//Parametros globales
string ARGUMENTO_PCL;
string ARGUMENTO_KP;
string ARGUMENTO_HARRIS;
bool INITIALIZED = false;

PointCloudXYZRGB::Ptr prevCloud (new PointCloudXYZRGB);
PointCloudXYZRGB::Ptr currCloud (new PointCloudXYZRGB);
PointCloudXYZRGB::Ptr finalCloud (new PointCloudXYZRGB);
pcl::Feature<pcl::PointXYZRGB, pcl::PFHRGBSignature250>::Ptr descriptor_detector (new pcl::PFHRGBEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::PFHRGBSignature250>);
boost::shared_ptr<pcl::Keypoint<pcl::PointXYZRGB, pcl::PointXYZI>> keypoint_detector;

pcl::CorrespondencesPtr inlinerCorrespondences(new pcl::Correspondences);
//pcl::visualization::PCLVisualizer visualizer_;
//pcl::visualization::PCLVisualizer viewer("PCL Viewer");
/*void visualize_keypoints (const PointCloudXYZRGB::Ptr points, const PointCloudXYZI::Ptr keypoints){
	// Visualization of keypoints along with the original cloud

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> keypoints_color_handler (keypoints, 0, 255, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> cloud_color_handler (points, 255, 255, 0);
	viewer.setBackgroundColor( 0.0, 0.0, 0.0 );
	viewer.addPointCloud(points, "cloud");
	viewer.addPointCloud(keypoints, keypoints_color_handler, "keypoints");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "keypoints");
	
	while(!viewer.wasStopped ()){
		viewer.spinOnce ();
	}
}*/
/*void visualize_normals (const pcl::PointCloud<pcl::PointXYZRGB>::Ptr points,
                        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr normal_points,
                        const pcl::PointCloud<pcl::Normal>::Ptr normals)
{
  // Add the points and normals to the vizualizer
  pcl::visualization::PCLVisualizer viz;
  viz.addPointCloud (points, "points");
  viz.addPointCloud (normal_points, "normal_points");
  viz.addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (normal_points, normals, 1, 0.01, "normals");
  // Give control over to the visualizer
  viz.spin ();
}*/

void simpleVis (){
  	pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
	while(!viewer.wasStopped()){
		viewer.showCloud(finalCloud);
		boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
	}
}

void compute_keypoints(PointCloudXYZRGB::ConstPtr source, PointCloudXYZI::Ptr keypoints){
	cout << ">> Computing keypoints...\n";
	if (ARGUMENTO_KP == "--sift3d"){
		//sift3d
		//KDtree to search for nearest matches.
		pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointXYZI>* sift = new pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointXYZI>;
		pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB> ());
		sift->setSearchMethod(tree);
		sift->setScales(0.01, 3, 2);
		sift->setMinimumContrast(0.0);
		keypoint_detector.reset(sift);
	}
	else if (ARGUMENTO_KP == "--harris3d"){
		//harris3d
		pcl::HarrisKeypoint3D<pcl::PointXYZRGB,pcl::PointXYZI>* harris3D = new pcl::HarrisKeypoint3D<pcl::PointXYZRGB,pcl::PointXYZI> (pcl::HarrisKeypoint3D<pcl::PointXYZRGB,pcl::PointXYZI>::HARRIS);
		
		harris3D->setNonMaxSupression(true);
		harris3D->setRadius (0.01);
		harris3D->setRadiusSearch (0.01);
		keypoint_detector.reset(harris3D);
		if (ARGUMENTO_HARRIS != ""){
			if (ARGUMENTO_HARRIS == "--TOMASI"){
				harris3D->setMethod(pcl::HarrisKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZI>::TOMASI);
			}
			else if (ARGUMENTO_HARRIS == "--NOBLE"){
				harris3D->setMethod(pcl::HarrisKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZI>::NOBLE);
			}
			else if (ARGUMENTO_HARRIS == "--LOWE"){
				harris3D->setMethod(pcl::HarrisKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZI>::LOWE);
			}
			else if (ARGUMENTO_HARRIS == "--CURVATURE"){
				harris3D->setMethod(pcl::HarrisKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZI>::CURVATURE);
			}
			else{
				cout << "Error argumentos de harris method." << "\n";
			}
		}
		else{
			harris3D->setMethod(pcl::HarrisKeypoint3D<pcl::PointXYZRGB, pcl::PointXYZI>::HARRIS);
		}
	}
	else{
		cout << "Error argumentos en keypoint detector." << "\n";
	}
	
	keypoint_detector->setInputCloud(source);
	keypoint_detector->setSearchSurface(source);
	keypoint_detector->compute(*keypoints);
	cout << "Keypoints detected: " << keypoints->points.size() << "\n";
}

void compute_descriptors(PointCloudXYZRGB::ConstPtr source, PointCloudXYZI::Ptr keypoints, PFHRGBSign250::Ptr features) {
	cout << ">> Computing descriptors...\n";
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
	cout << ">> Computing correspondences...\n";
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


void filter_correspondences(PointCloudXYZI::Ptr currKeypoints, PointCloudXYZI::Ptr prevKeypoints,
							std::vector<int>& currCorrespondences, std::vector<int>& prevCorrespondences) {
	std::vector<std::pair<unsigned, unsigned>> correspondences;
	cout << ">> Filtering correspondences...\n";
	for (unsigned i = 0; i < currCorrespondences.size (); ++i) {
		if (prevCorrespondences[currCorrespondences[i]] == i) {
			correspondences.push_back(std::make_pair(i, currCorrespondences[i]));
		}
	}

	inlinerCorrespondences->resize (correspondences.size());
	for (unsigned i = 0; i < correspondences.size(); ++i){
		(*inlinerCorrespondences)[i].index_query = correspondences[i].first;
		(*inlinerCorrespondences)[i].index_match = correspondences[i].second;
	}

	pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZI> rejector;
	rejector.setInputSource(currKeypoints);
	rejector.setInputTarget(prevKeypoints);
	rejector.setInputCorrespondences(inlinerCorrespondences);
	rejector.getCorrespondences(*inlinerCorrespondences);
}

void calculate_ICP(PointCloudXYZRGB::Ptr &dstCloud, PointCloudXYZRGB::Ptr prevCloud, PointCloudXYZRGB::Ptr &finalCloud){
	cout << ">>  Calculating ICP...\n";
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
  	transformSVD.estimateRigidTransformation (*currKp, *prevKp, *inlinerCorrespondences, transform);
	pcl::transformPointCloud(*source, *dstCloud, transform);
}

void scene_reconstruction(PointCloudXYZRGB::Ptr source, PointCloudXYZRGB::Ptr target, int i){
    cout << "------------------------------------\n";
	cout << ">> Cloud Nº" << i << "\n";
	const auto before = std::chrono::system_clock::now();

	// Keypoint detection
	PointCloudXYZI::Ptr currKeypoints(new PointCloudXYZI);
	PointCloudXYZI::Ptr prevKeypoints(new PointCloudXYZI);
	compute_keypoints(source, currKeypoints);
	//visualize_keypoints(source, currKeypoints);
	compute_keypoints(target, prevKeypoints);
	//visualize_keypoints(target, prevKeypoints);

	// Compute PFH features
	PFHRGBSign250::Ptr currDescriptors(new PFHRGBSign250);
	PFHRGBSign250::Ptr prevDescriptors(new PFHRGBSign250);
	compute_descriptors(source, currKeypoints, currDescriptors);
	compute_descriptors(target, prevKeypoints, prevDescriptors);

	// Find feature correspondences between two pointclouds
	std::vector<int> currCorrespondences;
	std::vector<int> prevCorrespondences;
	find_feature_correspondences(currDescriptors, prevDescriptors, currCorrespondences);
	cout << ">> Correspondences (Source to target): " << currCorrespondences.size() << "\n";
	find_feature_correspondences(prevDescriptors, currDescriptors, prevCorrespondences);
	cout << ">> Correspondences (Target to source): " << prevCorrespondences.size() << "\n";
	// Applying RANSAC we reject useless correspondences.
	filter_correspondences(currKeypoints, prevKeypoints, currCorrespondences, prevCorrespondences);
	cout << ">> Inlier Correspondences: " << inlinerCorrespondences->size() << "\n"; 
	// First initial transform
	PointCloudXYZRGB::Ptr dstCloud(new PointCloudXYZRGB);
	rigidTransformation(source,currKeypoints,prevKeypoints,dstCloud);
	// Applying ICP
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr transfCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	calculate_ICP(dstCloud, target, transfCloud);

	const auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now()-before);
	cout << ">> Duration: " << duration.count()/1000.0 << "ms" << "\n";
	
	cout << ">> Reconstructing scene...\n";
	if(finalCloud->points.size() == 0) {
		//*finalCloud = *dstCloud;  
		//*prevCloud = *dstCloud;
		*finalCloud = *transfCloud;  
		*prevCloud = *transfCloud;
	}
	else{
		//*finalCloud += *dstCloud;
		//*prevCloud = *dstCloud;
		*finalCloud += *transfCloud;
		*prevCloud = *transfCloud;
	}
	cout << "-------------------------------------\n";
}

bool check_arguments(int argc, char** argv){
	ARGUMENTO_PCL = "";
	ARGUMENTO_KP = "";

	bool ok = false;

    if (argc >= 3){
        ARGUMENTO_PCL = argv[1];
		ARGUMENTO_KP = argv[2];

		if (argc == 4){
			ARGUMENTO_HARRIS = argv[3];
			cout << ARGUMENTO_PCL << " " << ARGUMENTO_KP << " " << ARGUMENTO_HARRIS << "\n";
		}

        if (ARGUMENTO_PCL == "--o")
            ok = true;
        else if (ARGUMENTO_PCL == "--s")
            ok = true;
        else if (ARGUMENTO_PCL == "--sv")
            ok = true;
        else if (ARGUMENTO_PCL == "--v")
            ok = true;

		if (ARGUMENTO_KP == "--harris3d" && ok)
			return true;
		if (ARGUMENTO_KP == "--sift3d" && ok)
			return true;
    }
    else{
		string usage = "{ [--o | --v | --s | --sv] [--harris3d | --sift3d]}";
		cout << "Incorrect program use! Usage must be: " << "\n";
		cout << "rosrun *package_name* *executable_name* " << usage << "\n";
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
			vGrid.setLeafSize (0.025, 0.025, 0.025);
			vGrid.setDownsampleAllData(true);
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
			vGrid.setInputCloud (cloud);
			vGrid.setLeafSize (0.025, 0.025, 0.025);
			vGrid.filter (*cloud);
			sor.setInputCloud (cloud);
			sor.setMeanK (50);
			sor.setStddevMulThresh (1.0);
			sor.filter (*cloud);
			return;
		}
	}
	
}

void callback(const PointCloudXYZRGB::ConstPtr& msg){
	PointCloudXYZRGB::Ptr cloud (new PointCloudXYZRGB(*msg));
	currCloud = cloud;
	cout << ">> Captured points before applying filters: " << currCloud->points.size() << "\n";
	apply_filters(currCloud);
	cout << ">> Captured points after applying filters: " << currCloud->points.size() << "\n";
	std::vector<int> indices;
	currCloud->is_dense = false;
	pcl::removeNaNFromPointCloud(*currCloud, *currCloud, indices);	
}

int main(int argc, char** argv){
	if(check_arguments(argc, argv)){
		cout << "@@@ Arguments initialized with: " << ARGUMENTO_PCL << " and " << ARGUMENTO_KP << ". @@@" << "\n";
		ros::init(argc, argv, "mapping_3d_node");
		ros::NodeHandle nh;
		ros::Subscriber sub = nh.subscribe<PointCloudXYZRGB>("/camera/depth/points", 1, callback);
		boost::thread t(simpleVis);

		int i = 0;
		
		while(ros::ok()){
			ros::spinOnce();			
			if (currCloud->size() == 0)
				continue;
			else if(!INITIALIZED) //Apply first cloud as prev.
				prevCloud = currCloud;
			//Algorithm
			scene_reconstruction(currCloud, prevCloud, i);
			
			INITIALIZED = true;
			i++;
		}
	}
}
