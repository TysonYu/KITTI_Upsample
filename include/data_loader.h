//
//  Created by Tyson YU on 3/29/2019
//

#ifndef DATA_LOADER_H
#define DATA_LOADER_H

#include "common_include.h"

class PointCloudLoader
{
public:
    typedef std::shared_ptr<PointCloudLoader> Ptr;
    string data_dir_;
    int cloud_id_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud_;

public:
    //  functions
    PointCloudLoader();
    PointCloudLoader(string & data_dir): data_dir_(data_dir){}
    void readKittiPclBinData(int cloud_id_);

};

class ImageLoader
{
public:
    typedef std::shared_ptr<ImageLoader> Ptr;
    string data_dir_;
    int image_id_;
    cv::Mat image_;

public:
    //  function
    ImageLoader();
    ImageLoader(string &data_dir): data_dir_(data_dir){}
    void readKittiImage(int image_id_);
};

class Calibration
{
public:
    typedef std::shared_ptr<Calibration> Ptr;
    Eigen::Matrix4f rt1_;//激光雷达到相机cam0的RT矩阵
    Eigen::Matrix4f rt2_;//cam0-to-cam2;
    Eigen::Matrix4f Rt_;//激光雷达到Cam2
    Eigen::Matrix4f intrisic_;//相机内参
    Calibration()
    {
    rt1_ << 7.533745e-03,-9.999714e-01,-6.166020e-04,-4.069766e-03,   
            1.480249e-02,7.280733e-04,-9.998902e-01,-7.631618e-02,  
            9.998621e-01,7.523790e-03,1.480755e-02,-2.717806e-01, 
            0,0,0,1;  
    // rt2_ << 9.999758e-01, -5.267463e-03,-4.552439e-03,5.956621e-02,    
    //         5.251945e-03,9.999804e-01,-3.413835e-03,2.900141e-04, 
    //         4.570332e-03,3.389843e-03,9.999838e-01,2.577209e-03,
    //         0,0,0,1;
    rt2_ << 9.999239e-01, 9.837760e-03, -7.445048e-03, 0,
            -9.869795e-03, 9.999421e-01,-4.278459e-03, 0,
            7.402527e-03, 4.351614e-03, 9.999631e-01, 0,
            0, 0, 0, 1;
    Rt_ = rt2_*rt1_;
    intrisic_ << 7.215377e+02, 0.000000e+00, 6.095593e+02, 4.485728e+01, 
                0.000000e+00, 7.215377e+02, 1.728540e+02, 2.163791e-01, 
                0.000000e+00, 0.000000e+00, 1.000000e+00, 2.745884e-03,    
                0,0,0,1;
    }

};




#endif //DATA_LOADER_H