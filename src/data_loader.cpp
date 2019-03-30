//
//  Created by Tyson YU on 3/29/2019
//

#include "data_loader.h"


void PointCloudLoader::readKittiPclBinData(int cloud_id_)
{
    std::stringstream ss;
    ss << std::setw(10) << std::setfill('0') << cloud_id_ << ".bin";
    std::string in_file = data_dir_ + "/velodyne_points/data/" + ss.str();
    std::fstream input(in_file.c_str(), std::ios::in | std::ios::binary);
    if(!input.good())
    {
        std::cerr << "Could not read file: " << in_file << std::endl;
        exit(EXIT_FAILURE);
    }
    input.seekg(0, std::ios::beg);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    for (int i=0; input.good() && !input.eof(); i++) {
        pcl::PointXYZ point_xyz;
        float temp;
        input.read((char *) &point_xyz.x, 3*sizeof(float));
        input.read((char *) &temp, sizeof(float));
        cloud->points.push_back(point_xyz);
    }
    input.close();
    raw_cloud_ = cloud;
}

void ImageLoader::readKittiImage(int image_id_)
{
    std::stringstream ss;
    ss << std::setw(10) << std::setfill('0') << image_id_ << ".png";
    std::string in_file = data_dir_ + "/image_02/data/" + ss.str();
    image_ = cv::imread(in_file, cv::IMREAD_COLOR);
}