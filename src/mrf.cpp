//
//  Created by Tyson YU on 3/29/2019
//

#include "mrf.h"
#define TOTAL 1000*200
#define TOTAL_16 1000*200*16


void MRF::MRFProcess()
{
    pcl::transformPointCloud (*raw_cloud_, *raw_cloud_, calibration_->Rt_);//相机坐标系
    //  过滤掉点云中后面的点
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud (raw_cloud_);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0, 200);//delete all the point that z<0 && z>200
    pass.filter (*raw_cloud_);
    pcl::transformPointCloud (*raw_cloud_, *raw_cloud_, calibration_->intrisic_);//image，Z上未归一化的像素坐标系
    for(int i = 0; i < raw_cloud_->points.size(); i++)
    {
        raw_cloud_->points[i].x = raw_cloud_->points[i].x / raw_cloud_->points[i].z;
        raw_cloud_->points[i].y = raw_cloud_->points[i].y / raw_cloud_->points[i].z;
        if (raw_cloud_->points[i].z > max_depth_)
                max_depth_ = raw_cloud_->points[i].z;
    }
    // 把点云投影到图像上
    // boost::timer timer_interpolation;
    cv::Mat M(raw_image_.rows, raw_image_.cols, CV_32F);//把点投影到M上
    cv::Mat P(raw_image_.rows, raw_image_.cols, CV_32F);//扩展投影点
    cv::MatIterator_<float>Mbegin,Mend;//遍历所有像素，初始化像素值
	for (Mbegin=M.begin<float>(),Mend=M.end<float>();Mbegin!=Mend;++Mbegin)
		*Mbegin=max_depth_;
    for(int i = 0; i < raw_cloud_->points.size(); i++)//把深度值投影到图像M上
    {
        if(raw_cloud_->points[i].x>=0  && raw_cloud_->points[i].x<raw_image_.cols && raw_cloud_->points[i].y>=0 && raw_cloud_->points[i].y<raw_image_.rows)
        {
            if( raw_cloud_->points[i].z < M.at<float>(raw_cloud_->points[i].y,raw_cloud_->points[i].x))
                M.at<float>(raw_cloud_->points[i].y,raw_cloud_->points[i].x) = raw_cloud_->points[i].z;
        }
    }

    //  结合深度图和彩色图像的颜色进行上采样．
    cv::cvtColor(raw_image_, raw_image_gray_, CV_RGB2GRAY);//把图片转化为灰度图
    for(int count = 0; count < 9; count ++)
    {
        if (count%2 == 0) 
        {
            for(int i = 1; i < raw_image_.rows-1; i++)
            {
                for(int j = 1; j < raw_image_.cols-1; j++)
                {
                    if(M.at<float>(i,j) == max_depth_)
                    {
                        int flag[4] = {0,0,0,0};//  上下左右
                        int gray_diference[4] = {0,0,0,0};
                        float neighbor_depth[4] = {0,0,0,0};
                        float sum = 0;
                        int cnt = 0;
                        int count = 0;
                        neighbor_depth[0] = M.at<float>(i-1,j);
                        neighbor_depth[1] = M.at<float>(i+1,j);
                        neighbor_depth[2] = M.at<float>(i,j-1);
                        neighbor_depth[3] = M.at<float>(i,j+1);
                        gray_diference[0] = abs(raw_image_gray_.at<char>(i-1,j) - raw_image_gray_.at<char>(i,j));
                        gray_diference[1] = abs(raw_image_gray_.at<char>(i+1,j) - raw_image_gray_.at<char>(i,j));
                        gray_diference[2] = abs(raw_image_gray_.at<char>(i,j-1) - raw_image_gray_.at<char>(i,j));
                        gray_diference[3] = abs(raw_image_gray_.at<char>(i,j+1) - raw_image_gray_.at<char>(i,j));
                        if(neighbor_depth[0] < max_depth_ && gray_diference[0] < 10) flag[0] =1;
                        if(neighbor_depth[1] < max_depth_ && gray_diference[1] < 10) flag[1] =1;
                        if(neighbor_depth[2] < max_depth_ && gray_diference[2] < 10) flag[2] =1;
                        if(neighbor_depth[3] < max_depth_ && gray_diference[3] < 10) flag[3] =1;
                        for(int k = 0; k < 4; k++)
                        {
                            if(flag[k] == 1 && max_depth_ > neighbor_depth[k])
                            {
                                sum = sum + neighbor_depth[k] * (10 - gray_diference[k]);
                                cnt = cnt + (10 - gray_diference[k]);
                                count ++;
                            }
                        }
                        if(count > 0)
                        {
                            P.at<float>(i,j) = sum/cnt;
                            // cout << cnt << endl;
                        }
                            
                        else
                            P.at<float>(i,j) = M.at<float>(i,j);
                    }
                    else
                        P.at<float>(i,j) = M.at<float>(i,j);
                }
            }
        }
        else
        {
            for(int i = 1; i < raw_image_.rows-1; i++)
            {
                for(int j = 1; j < raw_image_.cols-1; j++)
                {
                    if(P.at<float>(i,j) == max_depth_)
                    {
                        int flag[4] = {0,0,0,0};//  上下左右
                        int gray_diference[4] = {0,0,0,0};
                        float neighbor_depth[4] = {0,0,0,0};
                        float sum = 0;
                        int cnt = 0;
                        int count = 0;
                        neighbor_depth[0] = P.at<float>(i-1,j);
                        neighbor_depth[1] = P.at<float>(i+1,j);
                        neighbor_depth[2] = P.at<float>(i,j-1);
                        neighbor_depth[3] = P.at<float>(i,j+1);
                        gray_diference[0] = abs(raw_image_gray_.at<char>(i-1,j) - raw_image_gray_.at<char>(i,j));
                        gray_diference[1] = abs(raw_image_gray_.at<char>(i+1,j) - raw_image_gray_.at<char>(i,j));
                        gray_diference[2] = abs(raw_image_gray_.at<char>(i,j-1) - raw_image_gray_.at<char>(i,j));
                        gray_diference[3] = abs(raw_image_gray_.at<char>(i,j+1) - raw_image_gray_.at<char>(i,j));
                        if(neighbor_depth[0] < max_depth_ && gray_diference[0] < 10) flag[0] =1;
                        if(neighbor_depth[1] < max_depth_ && gray_diference[1] < 10) flag[1] =1;
                        if(neighbor_depth[2] < max_depth_ && gray_diference[2] < 10) flag[2] =1;
                        if(neighbor_depth[3] < max_depth_ && gray_diference[3] < 10) flag[3] =1;
                        for(int k = 0; k < 4; k++)
                        {
                            if(flag[k] == 1 && max_depth_ > neighbor_depth[k])
                            {
                                sum = sum + neighbor_depth[k] * (10 - gray_diference[k]);
                                cnt = cnt + (10 - gray_diference[k]);
                                count ++;
                            }
                        }
                        if(count > 0)
                            M.at<float>(i,j) = sum/cnt;
                        else
                            M.at<float>(i,j) = P.at<float>(i,j);
                    }
                    else
                        M.at<float>(i,j) = P.at<float>(i,j);
                }
            }
        }
    }

    //  只使用稀疏深度点进行差值采样的简单方法．    
    for(int count = 0; count < 5; count ++)
    {
        if (count%2 == 0) 
        {
            for (int i=1;i<M.rows-1;i++)
	        {
		        for (int j=1;j<M.cols-1;j++)
		        {
                    if(M.at<float>(i,j) == max_depth_)
                    {
                        float temp = max_depth_;
                        float sum = 0;
                        int cnt = 0;
                        for(int n = i-1; n < i+2; n++)
                        {
                            for(int m = j-1; m < j+2; m++)
                            {
                                if(M.at<float>(n,m) < temp )
                                {
                                    sum = sum + M.at<float>(n,m);
                                    cnt ++;
                                    temp = M.at<float>(n,m);
                                }   
                            }
                        }
                        if (cnt > 0) 
                            temp = sum / cnt;
                        P.at<float>(i,j) = temp;
                    }
                    else
                        P.at<float>(i,j)  = M.at<float>(i,j);
		        }
            }
        }
        else
        {
            for (int i=1;i<M.rows-1;i++)
	        {
		        for (int j=1;j<M.cols-1;j++)
		        {
                    if(P.at<float>(i,j) == max_depth_)
                    {
                        float temp = max_depth_;
                        float sum = 0;
                        int cnt = 0;
                        for(int n = i-1; n < i+2; n++)
                        {
                            for(int m = j-1; m < j+2; m++)
                            {
                                if(P.at<float>(n,m) < temp)
                                {
                                    sum = sum + P.at<float>(n,m);
                                    cnt ++;
                                    temp = P.at<float>(n,m);
                                }
                            }
                        }
                        if (cnt > 0) 
                            temp = sum / cnt;
                        M.at<float>(i,j) = temp;
                    }
                    else
                        M.at<float>(i,j)  = P.at<float>(i,j);
		        }
            }
        }
    }
    // cout<<"Interpolation cost time: "<<timer_interpolation.elapsed() <<endl;
    depth_image_ = M;
    // cv::Mat temp_1(raw_image_.rows, raw_image_.cols, CV_8U);//把点投影到M上
    // for(int i = 0; i < M.rows; i++)
    //     for(int j = 0; j < M.cols; j++)
    //     {
    //         temp_1.at<char>(i,j) =  M.at<float>(i,j)/max_depth_ *255;
    //     }

    // cv::imshow("depthmap_1", temp_1);
    // cv::waitKey(0);
    // cv::destroyWindow("depthmap_1");

    //  取出要用的部分图像
    for(int i = 0; i < raw_image_.rows; i++)
        for(int j = 0; j < raw_image_.cols; j++)
        {
            depth_image_.at<float>(i,j) =  depth_image_.at<float>(i,j)/max_depth_ *255;
        }
    
    cv::Rect rect(0, 150, 1000, 200);
    cv::Mat small_RGB_image = raw_image_(rect);//RGB图像中的一小块
    cv::Mat_<cv::Vec3b> _small_RGB_image = small_RGB_image;
    cv::Mat small_depth_image = depth_image_(rect);//用临近深度填充的深度图中的一小块

    //  开始正式Rarkov Random field
    
    Eigen::SparseMatrix < double > A_1 (TOTAL , TOTAL) ;//创建一个稀疏矩阵A_1
    Eigen::SparseMatrix < double > A_2 (TOTAL , TOTAL) ;//创建一个稀疏矩阵A_2
    Eigen::SparseMatrix < double > A (TOTAL , TOTAL) ;//创建一个稀疏矩阵A
    Eigen::SparseMatrix < double > b (TOTAL , 1) ;//创建一个稀疏矩阵b
    Eigen::VectorXd x (TOTAL) ;//创建向量储存结果
    std::vector < Eigen::Triplet < double > > triplets_A_1 ;//创建一个用于初始化稀疏矩阵的向量A_1
    std::vector < Eigen::Triplet < double > > triplets_b ;//创建一个用于初始化稀疏矩阵的向量b
    std::vector < Eigen::Triplet < double > > triplets_A_2 ;//创建一个用于初始化稀疏矩阵的向量A_2
    std::vector < Eigen::Triplet < double > > triplets_A ;//创建一个用于初始化稀疏矩阵的向量A_2
    int total_pix = TOTAL;
    int *r = new int[TOTAL_16];
    int *c = new int[TOTAL_16];
    double *val = new double[TOTAL_16];
    for(int i = 0; i < total_pix; i++)
    {
        r[i] = i;
        c[i] = i;
        val[i] = 1;
    }
    for ( int i = 0 ; i < total_pix ; i++ )
         triplets_A_1.push_back ( Eigen::Triplet < double >(r[i] , c[i] , val[i]) );
    A_1.setFromTriplets ( triplets_A_1.begin ( ) , triplets_A_1.end ( ) );// 初始化A_1
    for(int  i = 0; i < total_pix; i++)
    {
        val[i] = 0;
        c[i] = 0;
            val[i] =  small_depth_image.at<float>(i/small_depth_image.size().width,i%small_depth_image.size().width);
    }
    for ( int i = 0 ; i < total_pix ; i++ )
         triplets_b.push_back ( Eigen::Triplet < double >(r[i] , c[i] , val[i]) );
    b.setFromTriplets ( triplets_b.begin ( ) , triplets_b.end ( ) );// 初始化b
    long flag = 0;
    double C = 0.01;
    for(int i = 0; i < total_pix; i++ )
    {
        if (i == 0)//左上角
        {
            int j_1 = i + 1;
            int j_2 = i + small_depth_image.size().width;
            int u = i / small_depth_image.size().width;//y坐标
            int v = i % small_depth_image.size().width;//x坐标
            int v_1 = v + 1;//右边邻居
            int u_2 = u + 1;//下边邻居
            double w_1 = exp(-C*sqrt(pow(_small_RGB_image(u,v)[2] - _small_RGB_image(u,v_1)[2],2) + pow(_small_RGB_image(u,v)[1] - _small_RGB_image(u,v_1)[1],2) + pow(_small_RGB_image(u,v)[0] - _small_RGB_image(u,v_1)[0],2)));
            r[flag] = i;
            c[flag] = j_1;
            val[flag] = -w_1;
            flag ++;
            r[flag] = j_1;
            c[flag] = i;
            val[flag] = -w_1;
            flag ++;
            r[flag] = j_1;
            c[flag] = j_1;
            val[flag] = w_1;
            flag ++;
            double w_2 = exp(-C*sqrt(pow(_small_RGB_image(u,v)[2] - _small_RGB_image(u_2,v)[2],2) + pow(_small_RGB_image(u,v)[1] - _small_RGB_image(u_2,v)[1],2) + pow(_small_RGB_image(u,v)[0] - _small_RGB_image(u_2,v)[0],2)));
            r[flag] = i;
            c[flag] = j_2;
            val[flag] = -w_2;
            flag ++;
            r[flag] = j_2;
            c[flag] = i;
            val[flag] = -w_2;
            flag ++;
            r[flag] = j_2;
            c[flag] = j_2;
            val[flag] = w_2;
            flag ++;
            r[flag] = i;
            c[flag] = i;
            val[flag] = w_1 + w_2;
            flag ++;
        }
        else if (i == small_depth_image.size().width)//右上角
        {
            int j_1 = i - 1;
            int j_2 = i + small_depth_image.size().width;
            int u = i / small_depth_image.size().width;//y坐标
            int v = i % small_depth_image.size().width;//x坐标
            int v_1 = v - 1;//左邻居
            int u_2 = u + 1;//下邻居
            double w_1 = exp(-C*sqrt(pow(_small_RGB_image(u,v)[2] - _small_RGB_image(u,v_1)[2],2) + pow(_small_RGB_image(u,v)[1] - _small_RGB_image(u,v_1)[1],2) + pow(_small_RGB_image(u,v)[0] - _small_RGB_image(u,v_1)[0],2)));
            r[flag] = i;
            c[flag] = j_1;
            val[flag] = -w_1;
            flag ++;
            r[flag] = j_1;
            c[flag] = i;
            val[flag] = -w_1;
            flag ++;
            r[flag] = j_1;
            c[flag] = j_1;
            val[flag] = w_1;
            flag ++;
            double w_2 = exp(-C*sqrt(pow(_small_RGB_image(u,v)[2] - _small_RGB_image(u_2,v)[2],2) + pow(_small_RGB_image(u,v)[1] - _small_RGB_image(u_2,v)[1],2) + pow(_small_RGB_image(u,v)[0] - _small_RGB_image(u_2,v)[0],2)));
            r[flag] = i;
            c[flag] = j_2;
            val[flag] = -w_2;
            flag ++;
            r[flag] = j_2;
            c[flag] = i;
            val[flag] = -w_2;
            flag ++;
            r[flag] = j_2;
            c[flag] = j_2;
            val[flag] = w_2;
            flag ++;
            r[flag] = i;
            c[flag] = i;
            val[flag] = w_1 + w_2;
            flag ++;
        }
        else if(i == total_pix - small_depth_image.size().width)//左下角
        {
            int j_1 = i - small_depth_image.size().width;//
            int j_2 = i + 1;
            int u = i / small_depth_image.size().width;//y坐标
            int v = i % small_depth_image.size().width;//x坐标
            int v_1 = v + 1;//右邻居
            int u_2 = u - 1;//上邻居
            double w_1 = exp(-C*sqrt(pow(_small_RGB_image(u,v)[2] - _small_RGB_image(u,v_1)[2],2) + pow(_small_RGB_image(u,v)[1] - _small_RGB_image(u,v_1)[1],2) + pow(_small_RGB_image(u,v)[0] - _small_RGB_image(u,v_1)[0],2)));
            r[flag] = i;
            c[flag] = j_1;
            val[flag] = -w_1;
            flag ++;
            r[flag] = j_1;
            c[flag] = i;
            val[flag] = -w_1;
            flag ++;
            r[flag] = j_1;
            c[flag] = j_1;
            val[flag] = w_1;
            flag ++;
            double w_2 = exp(-C*sqrt(pow(_small_RGB_image(u,v)[2] - _small_RGB_image(u_2,v)[2],2) + pow(_small_RGB_image(u,v)[1] - _small_RGB_image(u_2,v)[1],2) + pow(_small_RGB_image(u,v)[0] - _small_RGB_image(u_2,v)[0],2)));
            r[flag] = i;
            c[flag] = j_2;
            val[flag] = -w_2;
            flag ++;
            r[flag] = j_2;
            c[flag] = i;
            val[flag] = -w_2;
            flag ++;
            r[flag] = j_2;
            c[flag] = j_2;
            val[flag] = w_2;
            flag ++;
            r[flag] = i;
            c[flag] = i;
            val[flag] = w_1 + w_2;
            flag ++;
        }
        else if (i == -1)//右下角
        {
            int j_1 = i - 1;
            int j_2 = i - small_depth_image.size().width;
            int u = i / small_depth_image.size().width;//y坐标
            int v = i % small_depth_image.size().width;//x坐标
            int v_1 = v - 1;//左邻居
            int u_2 = u - 1;//上邻居
            double w_1 = exp(-C*sqrt(pow(_small_RGB_image(u,v)[2] - _small_RGB_image(u,v_1)[2],2) + pow(_small_RGB_image(u,v)[1] - _small_RGB_image(u,v_1)[1],2) + pow(_small_RGB_image(u,v)[0] - _small_RGB_image(u,v_1)[0],2)));
            r[flag] = i;
            c[flag] = j_1;
            val[flag] = -w_1;
            flag ++;
            r[flag] = j_1;
            c[flag] = i;
            val[flag] = -w_1;
            flag ++;
            r[flag] = j_1;
            c[flag] = j_1;
            val[flag] = w_1;
            flag ++;
            double w_2 = exp(-C*sqrt(pow(_small_RGB_image(u,v)[2] - _small_RGB_image(u_2,v)[2],2) + pow(_small_RGB_image(u,v)[1] - _small_RGB_image(u_2,v)[1],2) + pow(_small_RGB_image(u,v)[0] - _small_RGB_image(u_2,v)[0],2)));
            r[flag] = i;
            c[flag] = j_2;
            val[flag] = -w_2;
            flag ++;
            r[flag] = j_2;
            c[flag] = i;
            val[flag] = -w_2;
            flag ++;
            r[flag] = j_2;
            c[flag] = j_2;
            val[flag] = w_2;
            flag ++;
            r[flag] = i;
            c[flag] = i;
            val[flag] = w_1 + w_2;
            flag ++;
        }
        else if (i >= 1 && i <= small_depth_image.size().width-2)//上排
        {
            int j = i + small_depth_image.size().width;
            int u = i / small_depth_image.size().width;//y坐标
            int v = i % small_depth_image.size().width;//x坐标
            int u_1 = u + 1;//下邻居
            double w_1 = exp(-C*sqrt(pow(_small_RGB_image(u,v)[2] - _small_RGB_image(u_1,v)[2],2) + pow(_small_RGB_image(u,v)[1] - _small_RGB_image(u_1,v)[1],2) + pow(_small_RGB_image(u,v)[0] - _small_RGB_image(u_1,v)[0],2)));
            r[flag] = i;
            c[flag] = j;
            val[flag] = -w_1;
            flag ++;
            r[flag] = j;
            c[flag] = i;
            val[flag] = -w_1;
            flag ++;
            r[flag] = j;
            c[flag] = j;
            val[flag] = w_1;
            flag ++;
            r[flag] = i;
            c[flag] = i;
            val[flag] = w_1;
            flag ++;
        }
        else if (i>0 && i%small_depth_image.size().width == 0)//左排
        {
            int j = i + 1;
            int u = i / small_depth_image.size().width;//y坐标
            int v = i % small_depth_image.size().width;//x坐标
            int v_1 = v + 1;//右邻居
            double w_1 = exp(-C*sqrt(pow(_small_RGB_image(u,v)[2] - _small_RGB_image(u,v_1)[2],2) + pow(_small_RGB_image(u,v)[1] - _small_RGB_image(u,v_1)[1],2) + pow(_small_RGB_image(u,v)[0] - _small_RGB_image(u,v_1)[0],2)));
            r[flag] = i;
            c[flag] = j;
            val[flag] = -w_1;
            flag ++;
            r[flag] = j;
            c[flag] = i;
            val[flag] = -w_1;
            flag ++;
            r[flag] = j;
            c[flag] = j;
            val[flag] = w_1;
            flag ++;
            r[flag] = i;
            c[flag] = i;
            val[flag] = w_1;
            flag ++;
        }
        else if (i>0 && i%small_depth_image.size().width == small_depth_image.size().width-1)//右排
        {
            int j = i - 1;
            int u = i / small_depth_image.size().width;//y坐标
            int v = i % small_depth_image.size().width;//x坐标
            int v_1 = v - 1;//左邻居
            double w_1 = exp(-C*sqrt(pow(_small_RGB_image(u,v)[2] - _small_RGB_image(u,v_1)[2],2) + pow(_small_RGB_image(u,v)[1] - _small_RGB_image(u,v_1)[1],2) + pow(_small_RGB_image(u,v)[0] - _small_RGB_image(u,v_1)[0],2)));
            r[flag] = i;
            c[flag] = j;
            val[flag] = -w_1;
            flag ++;
            r[flag] = j;
            c[flag] = i;
            val[flag] = -w_1;
            flag ++;
            r[flag] = j;
            c[flag] = j;
            val[flag] = w_1;
            flag ++;
            r[flag] = i;
            c[flag] = i;
            val[flag] = w_1;
            flag ++;
        }
        else if (i > total_pix - small_depth_image.size().width && i < total_pix-1)//下排
        {
            int j = i - small_depth_image.size().width;
            int u = i / small_depth_image.size().width;//y坐标
            int v = i % small_depth_image.size().width;//x坐标
            int u_1 = u - 1;//上邻居
            double w_1 = exp(-C*sqrt(pow(_small_RGB_image(u,v)[2] - _small_RGB_image(u_1,v)[2],2) + pow(_small_RGB_image(u,v)[1] - _small_RGB_image(u_1,v)[1],2) + pow(_small_RGB_image(u,v)[0] - _small_RGB_image(u_1,v)[0],2)));
            r[flag] = i;
            c[flag] = j;
            val[flag] = -w_1;
            flag ++;
            r[flag] = j;
            c[flag] = i;
            val[flag] = -w_1;
            flag ++;
            r[flag] = j;
            c[flag] = j;
            val[flag] = w_1;
            flag ++;
            r[flag] = i;
            c[flag] = i;
            val[flag] = w_1;
            flag ++;
        }
        else//中间
        {
            int j_1 = i - 1;
            int j_2 = i + 1;
            int j_3 = i - small_depth_image.size().width;
            int j_4 = i + small_depth_image.size().width;
            int u = i / small_depth_image.size().width;//y坐标
            int v = i % small_depth_image.size().width;//x坐标
            int v_1 = v - 1;//左邻居
            int v_2 = v + 1;//右邻居
            int u_3 = u - 1;//上邻居
            int u_4 = u + 1;//下邻居
            double w_1 = exp(-C*sqrt(pow(_small_RGB_image(u,v)[2] - _small_RGB_image(u,v_1)[2],2) + pow(_small_RGB_image(u,v)[1] - _small_RGB_image(u,v_1)[1],2) + pow(_small_RGB_image(u,v)[0] - _small_RGB_image(u,v_1)[0],2)));
            r[flag] = i;
            c[flag] = j_1;
            val[flag] = -w_1;
            flag ++;
            r[flag] = j_1;
            c[flag] = i;
            val[flag] = -w_1;
            flag ++;
            r[flag] = j_1;
            c[flag] = j_1;
            val[flag] = w_1;
            flag ++;
            double w_2 = exp(-C*sqrt(pow(_small_RGB_image(u,v)[2] - _small_RGB_image(u,v_2)[2],2) + pow(_small_RGB_image(u,v)[1] - _small_RGB_image(u,v_2)[1],2) + pow(_small_RGB_image(u,v)[0] - _small_RGB_image(u,v_2)[0],2)));
            r[flag] = i;
            c[flag] = j_2;
            val[flag] = -w_2;
            flag ++;
            r[flag] = j_2;
            c[flag] = i;
            val[flag] = -w_2;
            flag ++;
            r[flag] = j_2;
            c[flag] = j_2;
            val[flag] = w_2;
            flag ++;
            double w_3 = exp(-C*sqrt(pow(_small_RGB_image(u,v)[2] - _small_RGB_image(u_3,v)[2],2) + pow(_small_RGB_image(u,v)[1] - _small_RGB_image(u_3,v)[1],2) + pow(_small_RGB_image(u,v)[0] - _small_RGB_image(u_3,v)[0],2)));
            r[flag] = i;
            c[flag] = j_3;
            val[flag] = -w_3;
            flag ++;
            r[flag] = j_3;
            c[flag] = i;
            val[flag] = -w_3;
            flag ++;
            r[flag] = j_3;
            c[flag] = j_3;
            val[flag] = w_3;
            flag ++;
            double w_4 = exp(-C*sqrt(pow(_small_RGB_image(u,v)[2] - _small_RGB_image(u_4,v)[2],2) + pow(_small_RGB_image(u,v)[1] - _small_RGB_image(u_4,v)[1],2) + pow(_small_RGB_image(u,v)[0] - _small_RGB_image(u_4,v)[0],2)));
            r[flag] = i;
            c[flag] = j_4;
            val[flag] = -w_4;
            flag ++;
            r[flag] = j_4;
            c[flag] = i;
            val[flag] = -w_4;
            flag ++;
            r[flag] = j_4;
            c[flag] = j_4;
            val[flag] = w_4;
            flag ++;
            r[flag] = i;
            c[flag] = i;
            val[flag] = w_1 + w_2 + w_3 + w_4;
            flag ++;
        }
    }
    for ( int i = 0 ; i < flag ; i++ )
         triplets_A_2.push_back ( Eigen::Triplet < double >(r[i] , c[i] , val[i]) );
    A_2.setFromTriplets ( triplets_A_2.begin ( ) , triplets_A_2.end ( ) );// 初始化A_2
    A = A_1 + A_2;  //  得到A
    cout << "starting cg calculation ........." << endl;
    boost::timer timer_MRF;
    Eigen::ConjugateGradient<Eigen::SparseMatrix<double>, Eigen::Lower|Eigen::Upper> cg;
    cg.setMaxIterations(100);
    cg.compute(A);
    x = cg.solve(b);
    std::cout << "#iterations:     " << cg.iterations() << std::endl;
    std::cout << "estimated error: " << cg.error()      << std::endl;
    cout<<"CG porcess cost time: "<<timer_MRF.elapsed() <<endl;
    Mat result(small_depth_image.size().height,small_depth_image.size().width,CV_32F);//result 用于储存结果
    for(int i = 0; i < TOTAL; i++)
    {
        result.at<float>(i/result.size().width,i%result.size().width) = x(i);
    }
    // 锐化深度图边缘
    // cv::Mat kernel(3,3,CV_32F,cv::Scalar(0));
    // kernel.at<float>(0,0) = 0.0;
    // kernel.at<float>(0,1) = -1.0;
    // kernel.at<float>(0,2) = 0.0;
    // kernel.at<float>(1,0) = -1.0;
    // kernel.at<float>(1,1) = 5.0;
    // kernel.at<float>(1,2) = -1.0;
    // kernel.at<float>(2.0) = 0.0;
    // kernel.at<float>(2,1) = -1.0;
    // kernel.at<float>(2.2) = 0.0;
    // cv::Mat mask(small_depth_image.rows, small_depth_image.cols, CV_32F);
    // cv::filter2D(result,result,result.depth(),kernel);
    //  show temp result (depth map)
    // cv::Mat temp(small_depth_image.rows, small_depth_image.cols, CV_8U);//把点投影到M上
    // for(int i = 0; i < small_depth_image.rows; i++)
    //     for(int j = 0; j < small_depth_image.cols; j++)
    //     {
    //         temp.at<char>(i,j) =  result.at<float>(i,j);
    //     }

    // cv::imshow("depthmap", temp);
    // cv::waitKey(0);
    // cv::destroyWindow("depthmap");
    
    // 深度图返回点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr result_cloud_xyz (new pcl::PointCloud<pcl::PointXYZ>);
    for(int i = 0; i < TOTAL; i++)
    {
        pcl::PointXYZ point;
        point.z = x(i);
        point.x = i%result.size().width * point.z;
        point.y = i/result.size().width * point.z;
        result_cloud_xyz->points.push_back(point);
    }
    pcl::transformPointCloud (*result_cloud_xyz, *result_cloud_xyz, calibration_->intrisic_.inverse());//image，Z上未归一化的像素坐标系
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr result_cloud_rgb (new pcl::PointCloud<pcl::PointXYZRGB>);
    for(int i = 0; i < TOTAL; i++)
    {
        pcl::PointXYZRGB point;
        point.z = result_cloud_xyz->points[i].z;
        point.x = result_cloud_xyz->points[i].x;
        point.y = result_cloud_xyz->points[i].y;
        point.r = _small_RGB_image(round(i/result.size().width),round(i%result.size().width))[2];//round() most closed number
        point.g = _small_RGB_image(round(i/result.size().width),round(i%result.size().width))[1];
        point.b = _small_RGB_image(round(i/result.size().width),round(i%result.size().width))[0];
        result_cloud_rgb->points.push_back(point);
    }
    result_cloud_ = result_cloud_rgb;
    delete [] r;
    delete [] c;
    delete [] val;
    // pcl::visualization::PCLVisualizer result_viewer("result");
    // result_viewer.addPointCloud(result_cloud_rgb, "sample cloud");
    // result_viewer.setBackgroundColor(0,0,0);
    // result_viewer.addCoordinateSystem();
    // while(!result_viewer.wasStopped())
    //     result_viewer.spinOnce();

}
