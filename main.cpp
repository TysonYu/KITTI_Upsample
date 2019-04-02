//
//  create by tiezheng yu on 2019-2-21
//----- useage: ./main /home/icey/Desktop/project/KITTI/2011_09_26_drive_0005_sync/velodyne_points/data/0000000100.bin /home/icey/Desktop/project/KITTI/2011_09_26_drive_0005_sync/image_02/data/0000000100.png
//


#include "common_include.h"
#include "data_loader.h"
#include "mrf.h"

int main(int argc, char **argv)
{
    string data_file = "/home/icey/Desktop/project/KITTI/2011_09_26_drive_0005_sync";
    PointCloudLoader::Ptr point_cloud_loader (new PointCloudLoader(data_file));
    ImageLoader::Ptr image_loader (new ImageLoader(data_file));
    Calibration::Ptr calibration (new Calibration);

    // pcl::visualization::PCLVisualizer viewer("result");//pcl viewer

    for(int i = 130; i < 153; i++)
    {
        cout << "========== fram number :" << i << "=============================" << endl;
        boost::timer timer;
        
        point_cloud_loader->readKittiPclBinData(i);
        image_loader->readKittiImage(i);
        MRF::Ptr mrf (new MRF);
        mrf->calibration_ = calibration;
        mrf->raw_cloud_ = point_cloud_loader->raw_cloud_;
        mrf->raw_image_ = image_loader->image_;
        mrf->MRFProcess();

        cout<<"total cost time: "<<timer.elapsed() <<endl;
        // cv::waitKey(10);
        usleep(10000);

        // cv::imshow("image", mrf->raw_image_);
        // cv::waitKey(0);
        // cv::destroyWindow("image");
        // viewer.addPointCloud(mrf->result_cloud_,to_string(i));
        // viewer.setBackgroundColor(0,0,0);
        // viewer.addCoordinateSystem();
        // viewer.spin();
        // viewer.removeCoordinateSystem();
        // viewer.removeAllPointClouds(); 
    }

    return 0;
}
