#pragma once

#include<iostream>
#include "string.h"
#include <fstream>
#include <cstdio>
#include <omp.h>
#include <ctime>
#include <windows.h>
#include "Freeimage.h"
#include <direct.h>  
#include <math.h>
#include <string>


#include<pcl/visualization/cloud_viewer.h>
#include<pcl/io/io.h>
#include<pcl/io/pcd_io.h>//pcd 读写类相关的头文件。
#include<pcl/io/ply_io.h>
#include<pcl/point_types.h> //PCL中支持的点类型头文件。
#include<pcl/octree/octree.h>
#include <vtkOutputWindow.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/common/common.h>
#include <pcl/filters/crop_box.h>
#include <pcl/point_cloud.h>
#include <pcl/common/colors.h>

#include "Eigen/Dense"

using namespace std;
using namespace Eigen;





struct angle_extent {
	float azimuth_min;
	float azimuth_max;
	float zenith_min;
	float zenith_max;
};


struct face_inf
{
	int face_rows;
	int start_row;
	int end_row;

	int face_cols;
	int start_col;
	int end_col;
};