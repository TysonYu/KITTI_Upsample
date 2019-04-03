//
//  create by tiezheng yu on 2019-2-21
//----- useage: ./main /home/icey/Desktop/project/KITTI/2011_09_26_drive_0005_sync/velodyne_points/data/0000000100.bin /home/icey/Desktop/project/KITTI/2011_09_26_drive_0005_sync/image_02/data/0000000100.png
//


#include "common_include.h"
#include "data_loader.h"
#include "mrf.h"

#include <condition_variable>

string data_file = "/home/icey/Desktop/project/KITTI/2011_09_26_drive_0005_sync";
// pcl::visualization::PCLVisualizer viewer("result");//pcl viewer
std::mutex mtx_1; // 全局互斥锁
std::condition_variable cv_1;
std::mutex mtx_2; // 全局互斥锁
std::condition_variable cv_2;
std::mutex mtx_3; // 全局互斥锁
std::condition_variable cv_3;
PointCloudLoader::Ptr point_cloud_loader (new PointCloudLoader(data_file));
ImageLoader::Ptr image_loader (new ImageLoader(data_file));
MRF::Ptr mrf_1 (new MRF);
MRF::Ptr mrf_2 (new MRF);
MRF::Ptr mrf_3 (new MRF);

void worker_thread_1()
{
    while(1)
    {
        std::unique_lock <std::mutex> lck(mtx_1);
        cv_1.wait(lck);
        mrf_1->m.lock();
        mrf_1->flag = false;
        cout << "worker_thread_1 processing" << endl;
        mrf_1->raw_image_ = image_loader->image_;
        mrf_1->raw_cloud_ = point_cloud_loader->raw_cloud_;
        mrf_1->MRFProcess();
        // viewer.addPointCloud(mrf_1->result_cloud_,"cloud");
        // viewer.setBackgroundColor(0,0,0);
        // viewer.addCoordinateSystem();
        // viewer.spin();
        // viewer.removeCoordinateSystem();
        // viewer.removeAllPointClouds(); 
        usleep(1000);
        mrf_1->flag = true;
        mrf_1->m.unlock();
    }
}

void worker_thread_2()
{
    while(1)
    {
        std::unique_lock <std::mutex> lck(mtx_2);
        cv_2.wait(lck);
        mrf_2->m.lock();
        mrf_2->flag = false;
        cout << "worker_thread_2 processing" << endl;
        mrf_2->raw_image_ = image_loader->image_;
        mrf_2->raw_cloud_ = point_cloud_loader->raw_cloud_;
        mrf_2->MRFProcess();
        // viewer.addPointCloud(mrf_2->result_cloud_,"cloud");
        // viewer.setBackgroundColor(0,0,0);
        // viewer.addCoordinateSystem();
        // viewer.spin();
        // viewer.removeCoordinateSystem();
        // viewer.removeAllPointClouds(); 
        usleep(1000);
        mrf_2->flag = true;
        mrf_2->m.unlock();
    }
}

void worker_thread_3()
{
    while(1)
    {
        std::unique_lock <std::mutex> lck(mtx_3);
        cv_3.wait(lck);
        mrf_3->m.lock();
        mrf_3->flag = false;
        cout << "worker_thread_3 processing" << endl;
        mrf_3->raw_image_ = image_loader->image_;
        mrf_3->raw_cloud_ = point_cloud_loader->raw_cloud_;
        mrf_3->MRFProcess();
        // viewer.addPointCloud(mrf_2->result_cloud_,"cloud");
        // viewer.setBackgroundColor(0,0,0);
        // viewer.addCoordinateSystem();
        // viewer.spin();
        // viewer.removeCoordinateSystem();
        // viewer.removeAllPointClouds(); 
        usleep(1000);
        mrf_3->flag = true;
        mrf_3->m.unlock();
    }
}

int main(int argc, char **argv)
{
    clock_t start = clock();
    // boost::timer timer_out;
    Calibration::Ptr calibration (new Calibration);    
    mrf_1->calibration_ = calibration;
    mrf_2->calibration_ = calibration;
    mrf_3->calibration_ = calibration;
    std::thread worker_1(worker_thread_1);
    std::thread worker_2(worker_thread_2);
    std::thread worker_3(worker_thread_3);
    for(int i = 0; i < 100; i++)
    {
        cout << "========== fram number :" << i << "=============================" << endl;
        
        point_cloud_loader->readKittiPclBinData(i);
        image_loader->readKittiImage(i);
        while(1)
        {
            if(mrf_1->flag == true)
            {
                cv_1.notify_one();
                break;
            }
            if(mrf_2->flag == true)
            {
                cv_2.notify_one();
                break;
            }
            if(mrf_3->flag == true)
            {
                cv_3.notify_one();
                break;
            }
            usleep(1000);
        }
        
        // usleep(10000);

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
    worker_1.detach();
    worker_2.detach();
    worker_3.detach();

    // cout<<"total cost time: "<<timer_out.elapsed() <<endl;
    clock_t end = clock();
    cout << "cost time = " << (double)(end-start)/CLOCKS_PER_SEC <<  " s"<< endl;
    return 0;
}
