//For greedy projection
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl/common/transforms.h>
#include <pcl/io/obj_io.h>

//For plane filtering
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>

//For outlier filtering
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/point_types.h>

//For Clustering
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/segmentation/extract_clusters.h>

//Poisson
#include <pcl/filters/filter.h>
#include <pcl/common/common.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>

#include <math.h>

using namespace std;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

PointCloudT ConcatenateClouds (PointCloudT cloud_a, PointCloudT cloud_b)
{
	cout << "Concating" << endl;
	PointCloudT cloud_c;
	cloud_c  = cloud_a;
	cloud_c += cloud_b;
	cout << "finsihed Concating" << endl;
	return cloud_c;
}

PointCloudT::Ptr FilterPlane (PointCloudT::Ptr cloud)
{

	PointCloudT::Ptr cloud_inliers (new PointCloudT),
						cloud_outliers (new PointCloudT);

	// Segment the ground
	pcl::ModelCoefficients::Ptr plane (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr 		  inliers_plane (new pcl::PointIndices);
	PointCloudT::Ptr 			      cloud_plane (new PointCloudT);

	// Make room for a plane equation (ax+by+cz+d=0)
	plane->values.resize (4);

	pcl::SACSegmentation<PointT> seg;				// Create the segmentation object
	seg.setOptimizeCoefficients (true);				// Optional
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setDistanceThreshold (0.005f);      // Controls how strict plane match is
	seg.setInputCloud (cloud);
	seg.segment (*inliers_plane, *plane);

	if (inliers_plane->indices.size () == 0) {
		PCL_ERROR ("Could not estimate a planar model for the given dataset.\n");
		return (cloud);
	}

	// Extract inliers
	pcl::ExtractIndices<PointT> extract;
	extract.setInputCloud (cloud);
	extract.setIndices (inliers_plane);
	extract.setNegative (false);			// Extract the inliers
	extract.filter (*cloud_inliers);		// cloud_inliers contains the plane

	// Extract outliers
	//extract.setInputCloud (cloud);		// Already done line 52
	//extract.setIndices (inliers);			// Already done line 53
	extract.setNegative (true);				// Extract the outliers
	extract.filter (*cloud_outliers);		// cloud_outliers contains everything but the plane

	printf ("Plane segmentation equation [ax+by+cz+d]=0: [%3.4f | %3.4f | %3.4f | %3.4f]     \t\n",
			plane->values[0], plane->values[1], plane->values[2] , plane->values[3]);

	return (cloud_outliers);
}

PointCloudT::Ptr RemoveOutliers (PointCloudT::Ptr cloud)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

	// Create the filtering object
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud (cloud);
	sor.setMeanK (50);
	sor.setStddevMulThresh (0.8);
	sor.filter (*cloud_filtered);
	cout << "Removed outliers" << endl;
	return cloud_filtered;
}

PointCloudT::Ptr ExtractLargestCluster (PointCloudT::Ptr cloud)
{

	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud (cloud);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance (0.005); // 2cm
	ec.setMinClusterSize (100);
	ec.setMaxClusterSize (25000);
	ec.setSearchMethod (tree);
	ec.setInputCloud (cloud);
	ec.extract (cluster_indices);

  pcl::PCDWriter writer;
	int j = 0;
	int largest_idx = 0;
	int largest_size = 0;
	pcl::PointCloud<pcl::PointXYZ>::Ptr largest_cluster;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	{
		int cur_size = (*it).indices.size();
		if (cur_size> largest_size){
			largest_idx = j;
			largest_size = cur_size;
		}
		j++;
	}

		// For each cluster, add the indexed points from the original cloud to an output cloud
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);

		pcl::PointIndices largest_cluster_indicies = cluster_indices[largest_idx];

		for (int i = 0; i < largest_cluster_indicies.indices.size(); i++)
			cloud_cluster->points.push_back (cloud->points[largest_cluster_indicies.indices[i]]); //*
		cloud_cluster->width = cloud_cluster->points.size ();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		// Write the cloud to a PCD file
		std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
		std::stringstream ss;
		ss << "cloud_cluster_" << j << ".pcd";
		writer.write<pcl::PointXYZ> (ss.str (), *cloud_cluster, false); //*

		return cloud_cluster;
}

PointCloudT::Ptr MLSSmooth (PointCloudT::Ptr cloud)
{
	using namespace pcl;
	MovingLeastSquares<PointXYZ, PointXYZ> mls;
	mls.setInputCloud (cloud);
	mls.setSearchRadius (0.01);
	mls.setPolynomialFit ( true );
	mls.setUpsamplingMethod (MovingLeastSquares<PointXYZ, PointXYZ>::VOXEL_GRID_DILATION);
  mls.setDilationVoxelSize(0.002);
	mls.setPolynomialOrder (4);

	// mls.setUpsamplingRadius (0.005);
	// mls.setUpsamplingStepSize (0.003);

	PointCloud<PointXYZ>::Ptr cloud_smoothed ( new PointCloud<PointXYZ> ());
	std::cout << "MLS processing..." << std::endl;
	mls.process (*cloud_smoothed);
	std::cout << "MLS processing finished" << std::endl;
	return cloud_smoothed;
}

void Poisson (PointCloudT::Ptr cloud, string mesh_path)
{
	using namespace pcl;
	NormalEstimationOMP<PointXYZ, Normal> ne;
	ne.setNumberOfThreads (8);
	ne.setInputCloud (cloud);
	ne.setRadiusSearch (0.01);
	Eigen::Vector4f centroid;
	compute3DCentroid (*cloud, centroid);
	ne.setViewPoint (centroid[0], centroid[1], centroid[2]);

	PointCloud<Normal>::Ptr cloud_normals ( new PointCloud<Normal> ());
	std::cout << "Surface normals processing..." << std::endl;
	ne.compute (*cloud_normals);
	std::cout << "Surface normals finished" << std::endl;

	PointCloud<PointNormal>::Ptr cloud_smoothed_normals ( new PointCloud<PointNormal> ());
	concatenateFields (*cloud, *cloud_normals, *cloud_smoothed_normals);

	std::cout << "Poisson starting..." << std::endl;
	pcl::Poisson<PointNormal> poisson;
	poisson.setDepth (7);
	poisson.setScale(1.0);
	poisson.setInputCloud(cloud_smoothed_normals);
	PolygonMesh mesh;
	poisson.reconstruct (mesh);
	std::cout << "Poisson finished..." << std::endl;

  pcl::io::saveOBJFile (mesh_path + ".obj", mesh);
  pcl::io::saveVTKFile (mesh_path + ".vtk", mesh);
	return;
}

PointCloudT::Ptr GenerateSecondCloud(PointCloudT::Ptr cloud, bool single_cloud){
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_back (new pcl::PointCloud<pcl::PointXYZ>);
	if(single_cloud)
		*cloud_back = *cloud;
	else{
		// Load input file into a PointCloud<T> with an appropriate type
		pcl::PCLPointCloud2 cloud_blob;
		//pcl::io::loadPCDFile ("point_clouds/pc_back1.pcd", cloud_blob);
		cout << "getting from pcd" << endl;
		pcl::io::loadPCDFile ("point_clouds/glasses_case_back.pcd", cloud_blob);
		cout << "got from pcd" << endl;
		pcl::fromPCLPointCloud2 (cloud_blob, *cloud_back);
		cout << "loaded from cloud" << endl;

		// Eliminate a plane from the cloud
		cloud_back = FilterPlane(cloud_back);
		// Remove any remaining outliers
		cloud_back = RemoveOutliers(cloud_back);
		// Cluster extraction to find biggest object
		cloud_back = ExtractLargestCluster(cloud_back);
	}

	float front_max_z = 0;
	float back_min_z = 0;
	float avg = 0;

	int num_points = cloud_back->points.size();

	for(size_t i = 0; i < cloud->points.size(); i++){
		float cur_z = cloud->points[i].z;
		if(cur_z > front_max_z){
			front_max_z = cur_z;
		}
	}

	for(size_t i = 0; i < num_points; i++){
		float cur_z = cloud_back->points[i].z;
		// Invert the z
		cloud_back->points[i].z = -cur_z;
		// Gather info for average
		avg += cur_z;
		// Gather info for max
		if(-cur_z < back_min_z){
			back_min_z = -cur_z;
		}
	}
	avg /= num_points;

	float std_dev = 0;
	for(size_t i = 0; i < num_points; i++){
		float cur_z = pow((-cloud_back->points[i].z - avg), 2);
		std_dev += cur_z;
	}
	std_dev /= num_points;
	std_dev = sqrt(std_dev);

	float distance = front_max_z - back_min_z;
	cout << "Front max z is: " << front_max_z << endl;
	cout << "Back min z is: " << back_min_z << endl;
	// Translate the points back up to match the original cloud
	for(size_t i = 0; i < num_points; i++){
		cloud_back->points[i].z += (2 * avg) + (3.5 * std_dev);
	}

	return cloud_back;
}


void ProcessCloud(string cloud_path, string mesh_path, bool poisson, bool single_cloud)
{
		// Load input file into a PointCloud<T> with an appropriate type
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PCLPointCloud2 cloud_blob;
		//pcl::io::loadPCDFile ("point_clouds/pc_back1.pcd", cloud_blob);
		pcl::io::loadPCDFile (cloud_path, cloud_blob);
		pcl::fromPCLPointCloud2 (cloud_blob, *cloud);
		//* the data should be available in cloud

		// Eliminate a plane from the cloud
		cloud = FilterPlane(cloud);
		// Remove any remaining outliers
		cloud = RemoveOutliers(cloud);
		// Cluster extraction to find biggest object
		cloud = ExtractLargestCluster(cloud);

		//Smooth the cloud
		cloud = MLSSmooth(cloud);
		/*
			FOR BACK CLOUD ONLY
			Take the front cloud, invert its coordinates, and reposition it to align
			with the original cloud
		*/
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_back (new pcl::PointCloud<pcl::PointXYZ>);

		cloud_back = GenerateSecondCloud(cloud, single_cloud);

		// Combine the front and back clouds in one cloud
		*cloud = ConcatenateClouds(*cloud, *cloud_back);

		// Use poisson meshing if applicable
		if(poisson){
			Poisson(cloud, mesh_path);
			return;
		}

		cloud = RemoveOutliers(cloud);

		// Rotate the cloud a number of degrees about the x axis to account for varying camera angle
		// Dont transform, only rotate (use identity transformation matrix)
		// Eigen::Affine3f transform (Eigen::Affine3f::Identity());
		// // Rotate 20 degrees for test
		// Eigen::Matrix3f rotation (Eigen::AngleAxisf((20.0*M_PI) / 180, Eigen::Vector3f::UnitX()));
		// transform.rotate(rotation);
		// pcl::transformPointCloud(*cloud, *cloud, transform);
		// std::cout << transform.matrix() << std::endl << std::endl;


		// Normal estimation*
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
		pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
		tree->setInputCloud (cloud);
		n.setInputCloud (cloud);
		n.setSearchMethod (tree);
		n.setKSearch (20);
		n.compute (*normals);
		//* normals should not contain the point normals + surface curvatures
		// Concatenate the XYZ and normal fields*
		pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
		pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
		//* cloud_with_normals = cloud + normals

		// Create search tree*
		pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
		tree2->setInputCloud (cloud_with_normals);

		// Initialize objects
		pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
		pcl::PolygonMesh triangles;

		// Set the maximum distance between connected points (maximum edge length)
		gp3.setSearchRadius (1.0); // Increased from .025

		// Set typical values for the parameters
		gp3.setMu (2.5);
		gp3.setMaximumNearestNeighbors (100); // Decreased from 500
		gp3.setMaximumSurfaceAngle(M_PI/2); // 90 degrees increased from default 45
		gp3.setMinimumAngle(M_PI/18); // 10 degrees
		gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
		gp3.setNormalConsistency(false);

		// Get result
		gp3.setInputCloud (cloud_with_normals);
		gp3.setSearchMethod (tree2);
		gp3.reconstruct (triangles);

		// Additional vertex information
		std::vector<int> parts = gp3.getPartIDs();
		std::vector<int> states = gp3.getPointStates();

    //pcl::io::saveVTKFile ("meshes/flat_mesh_back1.vtk", triangles);
    pcl::io::saveOBJFile (mesh_path + ".obj", triangles);
    pcl::io::saveVTKFile (mesh_path + ".vtk", triangles);
		// Finish
		return;
}

int main(int argc, char* argv[]){
	if(argc != 5){
		cout << "Please specify an input cloud path and an output vtk path" << endl;
		return 1;
	}
	bool poisson = (strcmp(argv[3], "0") == 0 ? false : true);
	bool single_cloud = (strcmp(argv[3], "0") == 0 ? false : true);
	cout << "Input cloud path is " << argv[1] << endl;
	cout << "Output vtk path is " << argv[2] << endl;
	cout << "Constructing mesh with: " << (poisson ? "Poisson" : "greedy") << endl;
	ProcessCloud(argv[1], argv[2], poisson, single_cloud);
	return 0;
}
