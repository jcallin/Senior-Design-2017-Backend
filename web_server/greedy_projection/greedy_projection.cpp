//For greedy projection
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl/common/transforms.h>

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

using namespace std;

//#include <Eigen/Dense>

/*
   This function takes a point cloud and eliminates points on a plane
 */
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

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

PointCloudT::Ptr ExtractLargestCluster (PointCloudT::Ptr cloud)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

	// Create the filtering object
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud (cloud);
	sor.setMeanK (50);
	sor.setStddevMulThresh (1.0);
	sor.filter (*cloud_filtered);
	cout << "Removed outliers" << endl;
	return cloud_filtered;
}

bool enforceCurvatureOrIntensitySimilarity (const pcl::PointNormal& point_a, const pcl::PointNormal& point_b, float squared_distance)
{
	//const Eigen::Vector3f point_a_normal = point_a.normal, point_b_normal = point_b.normal;
	Eigen::Map<const Eigen::Vector3f> point_a_normal(point_a.normal);
	Eigen::Map<const Eigen::Vector3f> point_b_normal(point_b.normal);
	if (fabs (point_a_normal.dot (point_b_normal)) < 0.05)
		return (true);
	return (false);
}

bool customRegionGrowing (const pcl::PointNormal& point_a, const pcl::PointNormal& point_b, float squared_distance)
{
	Eigen::Map<const Eigen::Vector3f> point_a_normal(point_a.normal);
	Eigen::Map<const Eigen::Vector3f> point_b_normal(point_b.normal);
	if (squared_distance < 10000)
	{
		if (fabs (point_a_normal.dot (point_b_normal)) < 0.06)
			return (true);
	}
	return (false);
}


PointCloudT::Ptr RemoveOutliers (PointCloudT::Ptr cloud)
{

	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud (cloud);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance (0.02); // 2cm
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


void ProcessCloud(string cloud_path, string mesh_path)
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

	// Rotate the cloud a number of degrees about the x axis to account for varying camera angle
	// Dont transform, only rotate (use identity transformation matrix)
	// Eigen::Affine3f transform (Eigen::Affine3f::Identity());
	// // Rotate 20 degrees for test
	// Eigen::Matrix3f rotation (Eigen::AngleAxisf((20.0*M_PI) / 180, Eigen::Vector3f::UnitX()));
	// transform.rotate(rotation);
	// pcl::transformPointCloud(*cloud, *cloud, transform);
	// std::cout << transform.matrix() << std::endl << std::endl;

	/*
	   FOR BACK CLOUD ONLY
	 */
	// Invert the y coordinates of the cloud
	// for(size_t i = 0; i < cloud->points.size(); i++){
	// 	cloud->points[i].y = -cloud->points[i].y;
	// }

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
	gp3.setSearchRadius (0.025);

	// Set typical values for the parameters
	gp3.setMu (2.5);
	gp3.setMaximumNearestNeighbors (500);
	gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
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
	pcl::io::saveVTKFile (mesh_path, triangles);
	// Finish
	return;
}

int main(int argc, char* argv[]){
	if(argc != 3){
		cout << "Please specify an input cloud path and an output vtk path" << endl;
	}
	cout << "Input cloud path is " << argv[1] << endl;
	cout << "Output vtk path is " << argv[2] << endl;
	ProcessCloud(argv[1], argv[2]);
	return 0;
}
>>>>>>> master
