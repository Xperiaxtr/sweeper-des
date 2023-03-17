#include <ros/ros.h>
#include <pcl/point_cloud.h>
//msgs type and conversion
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
//pcd io
#include <pcl/io/pcd_io.h>
//point types
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <time.h>

// class Livox_Scan
// {
// public:
// }
using namespace std;
using namespace cv;

//点云曲率, 40000为一帧点云中点的最大数量
float cloudCurvature[40000];
//曲率点对应的序号
int cloudSortInd[40000];
//点是否筛选过标志：0-未筛选过，1-筛选过
int cloudNeighborPicked[40000];
//点分类标号:2-代表曲率很大，1-代表曲率比较大,-1-代表曲率很小，0-曲率比较小(其中1包含了2,0包含了1,0和1构成了点云全部的点)
int cloudLabel[40000];


void Get_Scan(pcl::PointCloud<pcl::PointXYZ> &laserCloudIn)
{
    std::cout<<"size  "<<laserCloudIn.points.size()<<std::endl;
    vector<vector<int>> scans;
    vector<int> scan;
    std::vector<int> indices;
    //移除空点
    //pcl::removeNaNFromPointCloud(laserCloudIn, laserCloudIn, indices);
    std::cout<<"size  "<<laserCloudIn.points.size()<<std::endl;
    int cloudSize = laserCloudIn.points.size();

    double t1=clock();
    double angle_last,dis_incre_last,dis_incre_now,angle=0.0;
    for (size_t i = 0; i < cloudSize; i++)
    {
        angle = atan(laserCloudIn.points[i].z / (sqrt(laserCloudIn.points[i].x * laserCloudIn.points[i].x + laserCloudIn.points[i].y * laserCloudIn.points[i].y)));
        // std::cout<<"angle "<<angle<<std::endl;
        if (i ==0)
        {
            scan.push_back(i);
            angle_last = angle;
            continue;
        }
        dis_incre_now = angle - angle_last;
        if (i > 1 && (dis_incre_now > 0 && dis_incre_last < 0 || dis_incre_now < 0 && dis_incre_last > 0))
        {
            if(scan.size()>30)
            scans.push_back(scan);
            scan.clear();
            scan.push_back(i);
        }
        else scan.push_back(i);

        // double angle=atan(laserCloudIn.points[i].z/sqrt(laserCloudIn.points[i].x*laserCloudIn.points[i].x+laserCloudIn.points[i].y*laserCloudIn.points[i].y));

        angle_last = angle;
        dis_incre_last = dis_incre_now;
    }

    std::cout<<"71"<<std::endl;

    //开始计算曲率
    for(size_t i=0;i<scans.size();i++)
    for(size_t j=5;j<scans[i].size()-5;j++)
    {
        float diffX = laserCloudIn.points[scans[i][j]-5].x + laserCloudIn.points[scans[i][j] - 4].x 
                    + laserCloudIn.points[scans[i][j] - 3].x + laserCloudIn.points[scans[i][j] - 2].x 
                    + laserCloudIn.points[scans[i][j] - 1].x - 10 * laserCloudIn.points[scans[i][j]].x 
                    + laserCloudIn.points[scans[i][j] + 1].x + laserCloudIn.points[scans[i][j] + 2].x
                    + laserCloudIn.points[scans[i][j] + 3].x + laserCloudIn.points[scans[i][j] + 4].x
                    + laserCloudIn.points[scans[i][j] + 5].x;
        float diffY = laserCloudIn.points[scans[i][j] - 5].y + laserCloudIn.points[scans[i][j] - 4].y 
                    + laserCloudIn.points[scans[i][j] - 3].y + laserCloudIn.points[scans[i][j] - 2].y 
                    + laserCloudIn.points[scans[i][j] - 1].y - 10 * laserCloudIn.points[scans[i][j]].y 
                    + laserCloudIn.points[scans[i][j] + 1].y + laserCloudIn.points[scans[i][j] + 2].y
                    + laserCloudIn.points[scans[i][j] + 3].y + laserCloudIn.points[scans[i][j] + 4].y
                    + laserCloudIn.points[scans[i][j] + 5].y;
        float diffZ = laserCloudIn.points[scans[i][j] - 5].z + laserCloudIn.points[scans[i][j] - 4].z 
                    + laserCloudIn.points[scans[i][j] - 3].z + laserCloudIn.points[scans[i][j] - 2].z 
                    + laserCloudIn.points[scans[i][j] - 1].z - 10 * laserCloudIn.points[scans[i][j]].z 
                    + laserCloudIn.points[scans[i][j] + 1].z + laserCloudIn.points[scans[i][j] + 2].z
                    + laserCloudIn.points[scans[i][j] + 3].z + laserCloudIn.points[scans[i][j] + 4].z
                    + laserCloudIn.points[scans[i][j] + 5].z;
    
        //曲率计算
        cloudCurvature[scans[i][j]] = diffX * diffX + diffY * diffY + diffZ * diffZ;    
        cloudSortInd[scans[i][j]] = scans[i][j];
        // std::cout<<cloudCurvature[scans[i][j]]<<"  " <<cloudSortInd[i]<<std::endl;

    }
// std::cout<<"101"<<std::endl;

    //挑选点，排除容易被斜面挡住的点以及离群点
    for(int i = 5; i<laserCloudIn.points.size()-6;i++)
    {
        float diffX = laserCloudIn.points[i + 1].x - laserCloudIn.points[i].x;
        float diffY = laserCloudIn.points[i + 1].y - laserCloudIn.points[i].y;
        float diffZ = laserCloudIn.points[i + 1].z - laserCloudIn.points[i].z;
        //计算有效曲率点与后一个点之间的距离平方和
        float diff = diffX * diffX + diffY * diffY + diffZ * diffZ;

        if (diff > 0.1) {//前提:两个点之间距离要大于0.1

            //点的深度
        float depth1 = sqrt(laserCloudIn.points[i].x * laserCloudIn.points[i].x + 
                        laserCloudIn.points[i].y * laserCloudIn.points[i].y +
                        laserCloudIn.points[i].z * laserCloudIn.points[i].z);

        //后一个点的深度
        float depth2 = sqrt(laserCloudIn.points[i + 1].x * laserCloudIn.points[i + 1].x + 
                        laserCloudIn.points[i + 1].y * laserCloudIn.points[i + 1].y +
                        laserCloudIn.points[i + 1].z * laserCloudIn.points[i + 1].z);

        //按照两点的深度的比例，将深度较大的点拉回后计算距离
        if (depth1 > depth2) {
            diffX = laserCloudIn.points[i + 1].x - laserCloudIn.points[i].x * depth2 / depth1;
            diffY = laserCloudIn.points[i + 1].y - laserCloudIn.points[i].y * depth2 / depth1;
            diffZ = laserCloudIn.points[i + 1].z - laserCloudIn.points[i].z * depth2 / depth1;

            //边长比也即是弧度值，若小于0.1，说明夹角比较小，斜面比较陡峭,点深度变化比较剧烈,点处在近似与激光束平行的斜面上
            if (sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) / depth2 < 0.1) {//排除容易被斜面挡住的点
                //该点及前面五个点（大致都在斜面上）全部置为筛选过
            cloudNeighborPicked[i - 5] = 1;
            cloudNeighborPicked[i - 4] = 1;
            cloudNeighborPicked[i - 3] = 1;
            cloudNeighborPicked[i - 2] = 1;
            cloudNeighborPicked[i - 1] = 1;
            cloudNeighborPicked[i] = 1;
            }
        } else {
            diffX = laserCloudIn.points[i + 1].x * depth1 / depth2 - laserCloudIn.points[i].x;
            diffY = laserCloudIn.points[i + 1].y * depth1 / depth2 - laserCloudIn.points[i].y;
            diffZ = laserCloudIn.points[i + 1].z * depth1 / depth2 - laserCloudIn.points[i].z;

            if (sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) / depth1 < 0.1) {
            cloudNeighborPicked[i + 1] = 1;
            cloudNeighborPicked[i + 2] = 1;
            cloudNeighborPicked[i + 3] = 1;
            cloudNeighborPicked[i + 4] = 1;
            cloudNeighborPicked[i + 5] = 1;
            cloudNeighborPicked[i + 6] = 1;
            }
        }
        }

        float diffX2 = laserCloudIn.points[i].x - laserCloudIn.points[i - 1].x;
        float diffY2 = laserCloudIn.points[i].y - laserCloudIn.points[i - 1].y;
        float diffZ2 = laserCloudIn.points[i].z - laserCloudIn.points[i - 1].z;
        //与前一个点的距离平方和
        float diff2 = diffX2 * diffX2 + diffY2 * diffY2 + diffZ2 * diffZ2;

        //点深度的平方和
        float dis = laserCloudIn.points[i].x * laserCloudIn.points[i].x
                + laserCloudIn.points[i].y * laserCloudIn.points[i].y
                + laserCloudIn.points[i].z * laserCloudIn.points[i].z;

        //与前后点的平方和都大于深度平方和的万分之二，这些点视为离群点，包括陡斜面上的点，强烈凸凹点和空旷区域中的某些点，置为筛选过，弃用
        if (diff > 0.0002 * dis && diff2 > 0.0002 * dis) {
        cloudNeighborPicked[i] = 1;
        }
    }

std::cout<<"173"<<std::endl;
  pcl::PointCloud<pcl::PointXYZ> cornerPointsSharp;
  pcl::PointCloud<pcl::PointXYZ> cornerPointsLessSharp;
  pcl::PointCloud<pcl::PointXYZ> surfPointsFlat;
  pcl::PointCloud<pcl::PointXYZ> surfPointsLessFlat;

std::cout<<"scans.size() "<<scans.size()<<endl;
    for(size_t i=0;i<scans.size();i++)
    {
        //冒泡法排序
        for(size_t j=scans[i][0]+5;j<scans[i][0]+scans[i].size()-1-5;j++)
        {
            for(int k=j;k>=scans[i][0]+5;k--)
            {
                if(cloudCurvature[k]>cloudCurvature[k-1])
                {
                    int temp = cloudSortInd[k - 1];
                    cloudSortInd[k - 1] = cloudSortInd[k];
                    cloudSortInd[k] = temp;
                }
            }
        }
        // for(int k=scans[i][0];k<scans[i][0]+scans[i].size()-1;k++)
        // {
        //     std::cout<<k<<"  "<<cloudSortInd[k]<<std::endl;
        // }

        // std::cout<<"size  "<<scans[i].size()<<endl;
        // std::cout<<"195"<<std::endl;
        //挑选每个分段的曲率很大和比较大的点
        int largestPickedNum = 0;
        for(int k = scans[i][0]+scans[i].size()-5; k >= scans[i][0]+6; k--)
        {
            int ind = cloudSortInd[k];
            
            //如果曲率大的点，曲率的确比较大，并且未被筛选过滤掉
            if(cloudNeighborPicked[ind] == 0 &&cloudCurvature[ind] > 0.01)
            {
                largestPickedNum++;
                if (largestPickedNum <= 3) {//挑选曲率最大的前20个点放入less sharp点集合
                    cloudLabel[ind] = 1;//1代表点曲率比较尖锐
                    cornerPointsLessSharp.push_back(laserCloudIn.points[ind]);
                } else {
                    break;
                }

                cloudNeighborPicked[ind] = 1;//筛选标志置位

                          //将曲率比较大的点的前后各5个连续距离比较近的点筛选出去，防止特征点聚集，使得特征点在每个方向上尽量分布均匀
                for (int l = 1; l <= 5; l++) {
                    float diffX = laserCloudIn.points[ind + l].x 
                                - laserCloudIn.points[ind + l - 1].x;
                    float diffY = laserCloudIn.points[ind + l].y 
                                - laserCloudIn.points[ind + l - 1].y;
                    float diffZ = laserCloudIn.points[ind + l].z 
                                - laserCloudIn.points[ind + l - 1].z;
                    if (diffX * diffX + diffY * diffY + diffZ * diffZ < 0.05) {
                    break;
                    }

                    cloudNeighborPicked[ind + l] = 1;
                }
                for (int l = -1; l >= -5; l--) {
                    float diffX = laserCloudIn.points[ind + l].x 
                                - laserCloudIn.points[ind + l + 1].x;
                    float diffY = laserCloudIn.points[ind + l].y 
                                - laserCloudIn.points[ind + l + 1].y;
                    float diffZ = laserCloudIn.points[ind + l].z 
                                - laserCloudIn.points[ind + l + 1].z;
                    if (diffX * diffX + diffY * diffY + diffZ * diffZ < 0.05) {
                    break;
                    }

                    cloudNeighborPicked[ind + l] = 1;
                }

            }

        }
// std::cout<<"247"<<std::endl;
        //挑选每个分段的曲率很小比较小的点
        int smallestPickedNum = 0;
        for (int k = scans[i][0]+5; k <= scans[i][0]+scans[i].size()-5; k++) {
            int ind = cloudSortInd[k];
            if(ind<6)continue;
            // std::cout<<"ind  "<<ind<<endl;
            //如果曲率的确比较小，并且未被筛选出
            if (cloudNeighborPicked[ind] == 0 &&
                cloudCurvature[ind] < 0.001/* &&laserCloudIn.points[ind].z<-0.5*/) {

            cloudLabel[ind] = -1;//-1代表曲率很小的点

            surfPointsFlat.push_back(laserCloudIn.points[ind]);

            smallestPickedNum++;
            if (smallestPickedNum >= 20) {//只选最小的四个，剩下的Label==0,就都是曲率比较小的
                break;
            }

            cloudNeighborPicked[ind] = 1;
            for (int l = 1; l <= 5; l++) {//同样防止特征点聚集
                float diffX = laserCloudIn.points[ind + l].x 
                            - laserCloudIn.points[ind + l - 1].x;
                float diffY = laserCloudIn.points[ind + l].y 
                            - laserCloudIn.points[ind + l - 1].y;
                float diffZ = laserCloudIn.points[ind + l].z 
                            - laserCloudIn.points[ind + l - 1].z;
                if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
                break;
                }

                cloudNeighborPicked[ind + l] = 1;
            }
            for (int l = -1; l >= -5; l--) {
                float diffX = laserCloudIn.points[ind + l].x 
                            - laserCloudIn.points[ind + l + 1].x;
                float diffY = laserCloudIn.points[ind + l].y 
                            - laserCloudIn.points[ind + l + 1].y;
                float diffZ = laserCloudIn.points[ind + l].z 
                            - laserCloudIn.points[ind + l + 1].z;
                if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
                break;
                }

                cloudNeighborPicked[ind + l] = 1;
            }
            }
            // std::cout<<"num  "<<smallestPickedNum<<endl;
        }
        // std::cout<<"295"<<std::endl;

    }

std::cout<<"296"<<std::endl;
std::cout<<"cornerPointsLessSharp"<<cornerPointsLessSharp.size()<<std::endl;
std::cout<<"surfPointsFlat"<<surfPointsFlat.size()<<std::endl;
cv::Mat color_mat = cv::Mat(1000,1000,CV_8UC3,cv::Scalar(0,0,0));
for(int i=0;i<cornerPointsLessSharp.size();i++)
{

    int rand_x=1000-cornerPointsLessSharp.points[i].x/0.01;
    int rand_y=-cornerPointsLessSharp.points[i].y/0.01+500;
    circle(color_mat, cv::Point(rand_y,rand_x),2,cv::Scalar(0,0,255),-1,8);
}

// cv::Mat color_mat = cv::Mat(1000,1000,CV_8UC3,cv::Scalar(0,0,0));
for(int i=0;i<surfPointsFlat.size();i++)
{

    int rand_x=1000-surfPointsFlat.points[i].x/0.01;
    int rand_y=-surfPointsFlat.points[i].y/0.01+500;
    circle(color_mat, cv::Point(rand_y,rand_x),2,cv::Scalar(255,0,0),-1,8);
}

    // cout << "\nicp time" << (double)(clock() - t1) / CLOCKS_PER_SEC << endl;
    // cv::Mat color_mat = cv::Mat(1000,1000,CV_8UC3,cv::Scalar(0,0,0));
    //     for(size_t i=0;i<scans.size();i++)
    //     {
    //         for(size_t j=0;j<scans[i].size();j++)
    //         {
    //             if(laserCloudIn.points[scans[i][j]].x>0&&laserCloudIn.points[scans[i][j]].x<9.9&&laserCloudIn.points[scans[i][j]].y>-4.9
    //                 &&laserCloudIn.points[scans[i][j]].y<4.9)
    //             {    
                    
    //                 int rand_x=1000-laserCloudIn.points[scans[i][j]].x/0.01;
    //                 int rand_y=-laserCloudIn.points[scans[i][j]].y/0.01+500;
    //                 color_mat.at<cv::Vec3b>(rand_x,rand_y)[0]=((i+1)%3)*80;
    //                 color_mat.at<cv::Vec3b>(rand_x,rand_y)[1]=i%5*50;
    //                 color_mat.at<cv::Vec3b>(rand_x,rand_y)[2]=((i+1)%2)*100;
    //             }
    //         }
    //     }
        cv::imshow("color", color_mat);
        cv::waitKey(0);

}

int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>); //声明cloud
    pcl::io::loadPCDFile("/home/cidi/catkin_ws/pcl_2_pcd1.pcd", *cloud);
    Get_Scan(*cloud);
}