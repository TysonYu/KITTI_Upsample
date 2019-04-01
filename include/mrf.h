//
//  Created by Tyson YU on 3/29/2019
//

#ifndef MRF_H
#define MRF_H

#include "common_include.h"
#include "data_loader.h"

class MRF
{
public:
    typedef std::shared_ptr<MRF> Ptr;
    Calibration::Ptr calibration_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud_;
    cv::Mat raw_image_;
    cv::Mat raw_image_gray_;
    double max_depth_ = 0;
    cv::Mat_<cv::Vec3b> _small_RGB_image_;
    cv::Mat depth_image_;
    cv::Mat result_;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr result_cloud_;
public:
    MRF(){};
    MRF(Calibration::Ptr Calibration, pcl::PointCloud<pcl::PointXYZ>::Ptr Cloud, cv::Mat Image):calibration_(Calibration), raw_cloud_(Cloud), raw_image_(Image){}
    void MRFProcess();
};






#endif //MRF_H