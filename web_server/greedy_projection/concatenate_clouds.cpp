#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <math.h>

int main (int argc, char** argv)
{

	pcl::PointCloud<pcl::PointXYZ> cloud_a, cloud_b, cloud_c;
	pcl::PointCloud<pcl::Normal> n_cloud_b;
	pcl::PointCloud<pcl::PointNormal> p_n_cloud_c;

	// Fill in the cloud data
	cloud_a.width  = 5;
	cloud_a.height = cloud_b.height = n_cloud_b.height = 1;
	cloud_a.points.resize (cloud_a.width * cloud_a.height);

	cloud_b.width  = 3;
	cloud_b.points.resize (cloud_b.width * cloud_b.height);

	for (size_t i = 0; i < cloud_a.points.size (); ++i)
	{
		// Code for random points in a cloud
		// {
		// 	cloud_a.points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
		// 	cloud_a.points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
		// 	cloud_a.points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
		// }

		// Code for a circle
		// Generate 3 random numbers
		float x = 1024 * rand () / (RAND_MAX + 1.0f);
		float y = 1024 * rand () / (RAND_MAX + 1.0f);
		float z = 1024 * rand () / (RAND_MAX + 1.0f);
		// Normalize each number to (0-1)
		// If all number are not 0, normalize, else skip to assignment
		if(!(x == 0 && y == 0 && z == 0)){
			float norm_coef;
			norm_coef = 1 / sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2))
			x *= norm_coef * SPHERE_RADIUS;
			y *= norm_coef * SPHERE_RADIUS;
			z *= norm_coef * SPHERE_RADIUS;
		}
		cloud_a.points[i].x = x;
		cloud_a.points[i].y = y;
		cloud_a.points[i].z = z;
	}

	for (size_t i = 0; i < cloud_b.points.size (); ++i)
	{
		cloud_b.points[i].x = 0;
		cloud_b.points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
		cloud_b.points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
	}

	std::cerr << "Cloud A: " << std::endl;
	for (size_t i = 0; i < cloud_a.points.size (); ++i)
		std::cerr << "    " << cloud_a.points[i].x << " " << cloud_a.points[i].y << " " << cloud_a.points[i].z << std::endl;

	std::cerr << "Cloud B: " << std::endl;
	for (size_t i = 0; i < cloud_b.points.size (); ++i)
		std::cerr << "    " << cloud_b.points[i].x << " " << cloud_b.points[i].y << " " << cloud_b.points[i].z << std::endl;

	// Copy the point cloud data
	cloud_c  = cloud_a;
	cloud_c += cloud_b;
	std::cerr << "Cloud C: " << std::endl;
	for (size_t i = 0; i < cloud_c.points.size (); ++i)
		std::cerr << "    " << cloud_c.points[i].x << " " << cloud_c.points[i].y << " " << cloud_c.points[i].z << " " << std::endl;

	return (0);
}
