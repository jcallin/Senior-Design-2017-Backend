#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
#include <pcl/common/transforms.h>

#include <pcl/filters/filter.h>
#include <pcl/common/common.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/poisson.h>
using namespace pcl;
void ProcessCloud()
{
		// Load input file into a PointCloud<T> with an appropriate type
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
		cloud->is_dense = false;
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

		std::vector<int> indices;
		pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);
		/*
			FOR BACK CLOUD ONLY
		*/
		// Invert the y coordinates of the cloud
		// for(size_t i = 0; i < cloud->points.size(); i++){
		// 	cloud->points[i].y = -cloud->points[i].y;
		// }

		MovingLeastSquares<PointXYZ, PointXYZ> mls;
		mls.setInputCloud (cloud);
		mls.setSearchRadius (0.01);
		mls.setPolynomialFit ( true );
		mls.setPolynomialOrder (2);
		mls.setUpsamplingMethod (MovingLeastSquares<PointXYZ, PointXYZ>::SAMPLE_LOCAL_PLANE);
		mls.setUpsamplingRadius (0.005);
		mls.setUpsamplingStepSize (0.003);

		PointCloud<PointXYZ>::Ptr cloud_smoothed ( new PointCloud<PointXYZ> ());
		std::cout << "MLS processing..." << std::endl;
		mls.process (*cloud_smoothed);
		std::cout << "MLS processing finished" << std::endl;

		NormalEstimationOMP<PointXYZ, Normal> ne;
		ne.setNumberOfThreads (4);
		ne.setInputCloud (cloud_smoothed);
		ne.setRadiusSearch (0.01);
		Eigen::Vector4f centroid;
		compute3DCentroid (*cloud_smoothed, centroid);
		ne.setViewPoint (centroid[0], centroid[1], centroid[2]);

		PointCloud<Normal>::Ptr cloud_normals ( new PointCloud<Normal> ());
		std::cout << "Surface normals processing..." << std::endl;
		ne.compute (*cloud_normals);
		std::cout << "Surface normals finished" << std::endl;

		for ( size_t i = 0; i < cloud_normals->size (); ++i)
		{
			cloud_normals->points[i].normal_x *= -1;
			cloud_normals->points[i].normal_y *= -1;
			cloud_normals->points[i].normal_z *= -1;
		}

		PointCloud<PointNormal>::Ptr cloud_smoothed_normals ( new PointCloud<PointNormal> ());
		concatenateFields (*cloud_smoothed, *cloud_normals, *cloud_smoothed_normals);

		Poisson<PointNormal> poisson;
		poisson.setDepth (9);
		poisson.setInputCloud(cloud_smoothed_normals);
		PolygonMesh mesh;
		poisson.reconstruct (mesh);

    pcl::io::saveVTKFile ("meshes/flat_mesh_front1_i.vtk", mesh);
		// Finish
		return;
}

int main(){
	ProcessCloud();
	return 0;
}
