#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include<pcl/filters/statistical_outlier_removal.h>

#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/common/common.h>
#include <opencv2/opencv.hpp>

using namespace std;

void DrawAndMoveImg(int event, int x, int y, int flags, void *param);
void DrawLine(cv::Mat &tempimage);
void RemoveCode(cv::Mat &tempimage);
void CropRioImage(cv::Mat &rio_image);


struct Two_Points{
    cv::Point2i frist_point;
    cv::Point2i last_point;
};
Two_Points two_points;

bool ctrl_s = false, ctrl_z = false, ctrl_m = false, ctrl_d = false;//退出，撤销，删除，画线
// bool last_is_
    // cv::Mat mat_map = cv::Mat(hight, width, CV_8UC1, cv::Scalar(255));
cv::Mat mat_temp;//临时图片
cv::Mat rect_rio_image;

cv::Point2i left_up_point;
int main()
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZI>);

    if (pcl::io::loadPCDFile<pcl::PointXYZI>("/home/cidi/Loam_livox_pcd/surround10.pcd", *cloud) != 0)
    {
        return -1;
    }
    std::cout<<"cloud size1 "<<cloud->points.size()<<std::endl;
    // pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    // sor.setInputCloud(cloud); 
    // sor.setMeanK(6);
    // sor.setStddevMulThresh(0.8);
    // sor.filter(*cloud);

    // pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
    // outrem.setInputCloud(cloud);
    // outrem.setRadiusSearch(0.1);
    // outrem.setMinNeighborsInRadius(4);
    // outrem.filter(*cloud);
    // pcl::PointCloud <pcl::PointXYZ>::Ptr colored_cloud = reg.getColoredCloud ();
	
    pcl::PointXYZI min, max;
    pcl::getMinMax3D(*cloud,min,max);
    cout<<"min.x = "<<min.x<<"\n"<<endl;
    cout<<"max.x = "<<max.x<<"\n"<<endl;
    cout<<"min.y = "<<min.y<<"\n"<<endl;
    cout<<"max.y = "<<max.y<<"\n"<<endl;
    cv::Point2f point_center;
    point_center.x = (min.x + max.x)/2.0;
    point_center.y = (min.y + max.y)/2.0;
    int width, hight;
    hight = (max.x - min.x)/0.05;//5cm分辨率
    width = (max.y - min.y)/0.05;//5cm分辨率
    cv::Mat mat_map = cv::Mat(hight, width, CV_8UC1, cv::Scalar(255));

    for(size_t i = 0; i < cloud->points.size(); i++)
    {
        int color = cloud->points[i].intensity * 3.0;
        if(color > 255) color = 255;
        // std::cout<<"color  "<<color<<std::endl;
        int m = hight/2 - (cloud->points[i].x - point_center.x)/0.05;
        int n = width/2 - (cloud->points[i].y - point_center.y)/0.05;
        if(color<100) continue;
        mat_map.at<uchar>(m, n) = 0;
    }
    // for(int i = 0; i < hight; i++)
    // for(int j=0; j < width; j++)
    // {
    //     if(mat_map.at<uchar>(i, j) <254) mat_map.at<uchar>(i, j)=0;

    // }
    mat_map.copyTo(mat_temp);
    cv::namedWindow("mat_map", CV_WINDOW_NORMAL);
    cv::namedWindow("rect rio image", CV_WINDOW_NORMAL);
    cv::setMouseCallback("mat_map", DrawAndMoveImg, (void*)&mat_map);

    std::cout<<"x, y "<<mat_temp.cols<<"   "<<mat_temp.rows<<std::endl;
    while(1)
    {
        int key;
        key = cv::waitKey(1);
        if(key == 27)break;
        switch(key)
        {
            case 'm':
            {
                ctrl_s = false; 
                ctrl_m = false;
                ctrl_d = false;
                ctrl_z = false;                
                ctrl_m = true;
                std::cout<<"remove code"<<std::endl;
            }break;
            case 'd':
            {
                ctrl_s = false; 
                ctrl_m = false;
                ctrl_d = false;
                ctrl_z = false; 
                ctrl_d = true;
                std::cout<<"draw line"<<std::endl;
            }break;
            case 'z':
            {
                ctrl_s = false; 
                ctrl_m = false;
                ctrl_d = false;
                ctrl_z = false; 
                ctrl_z = true;
                std::cout<<"reset"<<std::endl;
            }break;
            case 's':
            {
                ctrl_s = false; 
                ctrl_m = false;
                ctrl_d = false;
                ctrl_z = false; 
                ctrl_s = true;
                std::cout<<"Crop RIO image"<<std::endl;
            }break;
        }
        if(ctrl_z)
            mat_temp.copyTo(mat_map);
        imshow("mat_map", mat_map);
        if(rect_rio_image.rows > 10)
        imshow("rect rio image", rect_rio_image);
    }
    std::cout<<"rio the left&&up x: "<<left_up_point.x<<std::endl;
    std::cout<<"rio the left&&up y: "<<left_up_point.y<<std::endl;
    cv::imwrite("map.png", mat_map);
    // cv::waitKey(1);
    // cv::imshow("mat_map", mat_map);
    // cv::waitKey(0);

    // pcl::visualization::CloudViewer viewer ("滤波");
	// viewer.showCloud(cloud);
	// while (!viewer.wasStopped ())
	// {
	// }//进行可视化

}



void DrawAndMoveImg(int event, int x, int y, int flags, void *param)
{

    cv::Mat &image = *(cv::Mat*)param;
    //记录点
    switch (event)
    {
        case cv::EVENT_MOUSEMOVE:
        {
            
        }break;
        case CV_EVENT_MBUTTONDOWN:
        {
            two_points.frist_point.x = x;
            two_points.frist_point.y = y;
            std::cout<<"frist x, y  "<<two_points.frist_point.x<<" "<<two_points.frist_point.y<<std::endl;
        }break;
        case CV_EVENT_MBUTTONUP:
        {
            two_points.last_point.x = x;
            two_points.last_point.y = y;
            std::cout<<"last x, y  "<<two_points.last_point.x<<" "<<two_points.last_point.y<<std::endl;
            if(ctrl_m || ctrl_d || ctrl_s)
            image.copyTo(mat_temp);
            if(ctrl_s) CropRioImage(rect_rio_image);
            if(ctrl_m) RemoveCode(image);
            if(ctrl_d) DrawLine(image);
        }break;
    }
}



void DrawLine(cv::Mat &tempimage)
{
    cv::line(tempimage, two_points.frist_point, two_points.last_point, cv::Scalar(0), 2);
}

void RemoveCode(cv::Mat &tempimage)
{
    int max_x, max_y, min_x, min_y;
    max_x = std::max<int>(two_points.frist_point.x, two_points.last_point.x);
    min_x = std::min<int>(two_points.frist_point.x, two_points.last_point.x);
    max_y = std::max<int>(two_points.frist_point.y, two_points.last_point.y);
    min_y = std::min<int>(two_points.frist_point.y, two_points.last_point.y);
    if(max_x > mat_temp.cols - 1) max_x = mat_temp.cols - 1;
    if(max_y > mat_temp.rows - 1) max_y = mat_temp.rows - 1;
    if(min_x < 0) min_x = 0;
    if(min_y < 0) min_y = 0;
    for(size_t i = min_y; i <= max_y; i++)
    for(size_t j = min_x; j <= max_x; j++)
    {
        tempimage.at<uchar>(i, j) = 255;
    }
}


void CropRioImage(cv::Mat &rio_image)
{
    int max_x, max_y, min_x, min_y;
    max_x = std::max<int>(two_points.frist_point.x, two_points.last_point.x);
    min_x = std::min<int>(two_points.frist_point.x, two_points.last_point.x);
    max_y = std::max<int>(two_points.frist_point.y, two_points.last_point.y);
    min_y = std::min<int>(two_points.frist_point.y, two_points.last_point.y);
    // cv::Mat image_rio = cv::Mat(max_y - min_y, max_x - min_x, CV_8UC1);
    // for(size_t i = min_y; i <= max_y; i++)
    // for(size_t j = min_x; j <= max_x; j++)
    // {
    //     image_rio.at<uchar>(i, j) = mat_map.at<uchar>(i, j);
    // }
    // image_rio.copyTo(rio_image);
    // rio_image

    std::cout<<"max x ,y  "<<max_x<<"  "<<max_y<<std::endl;
    std::cout<<"min x ,y  "<<min_x<<"  "<<min_y<<std::endl;

    if(max_x > mat_temp.cols - 1) max_x = mat_temp.cols - 1;
    if(max_y > mat_temp.rows - 1) max_y = mat_temp.rows - 1;
    if(min_x < 0) min_x = 0;
    if(min_y < 0) min_y = 0;

    left_up_point.x = min_x;
    left_up_point.y = min_y;
    // if(max_x - min_x <= 0 || max_y - min_y <= 0 || min_x < 0 ||) return;
    cv::Rect rect(min_x, min_y, max_x - min_x, max_y - min_y);
    rio_image = mat_temp(rect);
}
