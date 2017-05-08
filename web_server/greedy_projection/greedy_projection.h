#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/io/vtk_io.h>
//#include "pcd_gen.h"
#ifndef GREEDY_PROJECTION_H
#define GREEDY_PROJECTION_H

void ProcessCloud();
PointCloudT::Ptr FilterPlane (PointCloudt::Ptr cloud);

#endif /* GREEDY_PROJECTION_H */
