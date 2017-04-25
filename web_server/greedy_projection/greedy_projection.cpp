#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl/common/transforms.h>

void ProcessCloud()
{
		// Load input file into a PointCloud<T> with an appropriate type
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PCLPointCloud2 cloud_blob;
		//pcl::io::loadPCDFile ("point_clouds/pc_back1.pcd", cloud_blob);
		pcl::io::loadPCDFile ("point_clouds/pc_front1.pcd", cloud_blob);
		pcl::fromPCLPointCloud2 (cloud_blob, *cloud);
		//* the data should be available in cloud

		// Rotate the cloud a number of degrees about the x axis to account for varying camera angle
		// Dont transform, only rotate (use identity transformation matrix)
		Eigen::Affine3f transform (Eigen::Affine3f::Identity());
		// Rotate 20 degrees for test
		Eigen::Matrix3f rotation (Eigen::AngleAxisf((20.0*M_PI) / 180, Eigen::Vector3f::UnitX()));
		transform.rotate(rotation);
		pcl::transformPointCloud(*cloud, *cloud, transform);
		std::cout << transform.matrix() << std::endl << std::endl;

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
    pcl::io::saveVTKFile ("meshes/flat_mesh_front1.vtk", triangles);
		// Finish
		return;
}

int main(){
	ProcessCloud();
	return 0;
}
