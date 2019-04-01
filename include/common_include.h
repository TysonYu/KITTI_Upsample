//
// Created by Tyson YU on 3/25/2019
//

#ifndef COMMON_INCLUDE_H
#define COMMON_INCLUDE_H

// define the commonly included file to avoid a long include list
// for Eigen
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <Eigen/IterativeLinearSolvers>
using Eigen::Vector2d;
using Eigen::Vector3d;

// for OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
using cv::Mat;

//for PCL
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/transforms.h>//allows us to use pcl::transformPointCloud function
#include <pcl/filters/passthrough.h>//allows us to use pcl::PassThrough

// std 
#include <vector>
#include <list>
#include <memory>
#include <string>
#include <iostream>
#include <set>
#include <unordered_map>
#include <map>
#include <fstream>
#include <sstream>
#include <boost/timer.hpp>
#include <cmath>
using namespace std;
#endif  //COMMON_INCLUDE_H