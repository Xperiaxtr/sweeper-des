#include <ros/ros.h>
#include <algorithm>
#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

#include <nav_msgs/Odometry.h>


class GetMapPng
{
    public:

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_input;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_noplane;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_map;
    bool new_cloud_receive;
    bool new_odom_receive;
    double time_new_odom_receive;
    double time_new_cloud_receive;
    Eigen::Quaterniond gp_odom_q_curr;
    Eigen::Vector3d gp_odom_t_curr;
    Eigen::Matrix3d rotationMatrix;//矫正的变换矩阵
    std::vector<cv::Point2i > map_points_;
    ros::Subscriber gp_sub_cloud;
    ros::Subscriber gp_sub_odom;
    ros::Publisher gp_pub_cloud;
    int counts = 0;


    double livox_yaw_ = 0.0;
    double livox_pitch_ = 0.261;
    double livox_roll_ = 0.0002;
    double livox_x_ = -0.5;
    double livox_y_ = 0.0;
    double livox_z_ = 0.6721+1.32;
    // int width = 0, hight = 0;
    int x_max = -100, x_min = 1000000, y_max = -100, y_min = 1000000;
    pcl::VoxelGrid<pcl::PointXYZI> m_down_sample_filter_points;


    void LaserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &msg);
    void GetRioCloud();
    void LaserOdometryHandler(nav_msgs::Odometry laserOdometry);
    int Get_Threshold_New(cv::Mat src);
    int Get_Threshold_Img(cv::Mat src, cv::Mat dstImage);
    void PointAssociateToBeMapped(pcl::PointXYZI &pi);
    void PointCloudAssociateToMap(pcl::PointCloud<pcl::PointXYZI> &pc_cloud);
    Eigen::Matrix4f CreateRotateMatrix(Eigen::Vector3f before, Eigen::Vector3f after);
    void Correction_Cloud(pcl::PointCloud<pcl::PointXYZI>::Ptr &point_cloud);
    void MapToPng(int width, int hight);
    void AddPointCloudToMap(int & width, int & hight);
    void RunGetMapPng();
    GetMapPng();
    ~GetMapPng(){};
};

GetMapPng::GetMapPng(): new_cloud_receive(false), new_odom_receive(false), time_new_odom_receive(0.0), 
                        time_new_cloud_receive(0.0)
{
    ros::NodeHandle nh;
    gp_sub_cloud = nh.subscribe("/laser_cloud_all_real", 1000, &GetMapPng::LaserCloudHandler, this);
    gp_sub_odom = nh.subscribe("/aft_mapped_to_init", 1000, &GetMapPng::LaserOdometryHandler, this);
    gp_pub_cloud = nh.advertise<sensor_msgs::PointCloud2>("/pub_noplane_cloud", 1);
    cloud_input = pcl::PointCloud<pcl::PointXYZI>::Ptr( new pcl::PointCloud<pcl::PointXYZI>() );
    cloud_noplane = pcl::PointCloud<pcl::PointXYZI>::Ptr( new pcl::PointCloud<pcl::PointXYZI>() );
    cloud_map = pcl::PointCloud<pcl::PointXYZI>::Ptr( new pcl::PointCloud<pcl::PointXYZI>() );

    m_down_sample_filter_points.setLeafSize(0.01, 0.01, 0.01);//0.04
}



void GetMapPng::LaserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &msg)
{
  // std::cout<<"66"<<std::endl;
    time_new_cloud_receive = msg->header.stamp.toSec();
    // cloud_input->clear();
    pcl::fromROSMsg(*msg, *cloud_input);
    // Correction_Cloud(cloud_input);
    new_cloud_receive = true;
}


void GetMapPng::GetRioCloud()
{
    int cloudsize = cloud_input->points.size();
    cv::Mat mat_gray = cv::Mat(1000, 2000, CV_8UC1, cv::Scalar(255));
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_roi(new pcl::PointCloud<pcl::PointXYZI>);
    for(size_t i = 0; i < cloudsize; i++)
    {
        pcl::PointXYZI point = cloud_input->points[i];
        if(point.x < 10.0 && point.x > 0.0 && point.y > -10.0 && point.y < 10.0)
        {
            cloud_roi->points.push_back(point);
            // if(point.y <= -5.0 || point.y >= 5.0) continue;
            int m = 1000 - round(point.x / 0.01);
            int n = 1000 - round(point.y / 0.01);
            int color = round((point.z + 0.2) / 0.002);

            if (color > 255)
            {
              color = 255;
            }
            else if (color < 0)
            {
              color = 0;
            }
            mat_gray.at<uchar>(m, n) = 255 - color;
        }
    }
    int threshold1;
    threshold1 = Get_Threshold_Img(mat_gray, mat_gray);
    cv::imshow("mat_gray", mat_gray);
    cv::waitKey(1);

    cv::Mat mat_brinary = cv::Mat(1000, 2000, CV_8UC1, cv::Scalar(255));
    // int threshold1, threshold2;
    // threshold1 = Get_Threshold_New(mat_down);
    // threshold2 = Get_Threshold_New(mat_up);
    // std::cout << "threshold1  " << threshold1 << "    threshold2   " << threshold2 << std::endl;
    // if (threshold2 - threshold1 > 10)
    // threshold1 = threshold2;

    // double threshold = threshold1 * 0.002 - 0.2;
    // std::cout<<"hight "<<threshold<<std::endl;
    for(size_t i = 0; i < cloud_roi->points.size(); i++)
    {
        pcl::PointXYZI point = cloud_roi->points[i];
        int m = 1000 - point.x/0.01;
        int n = 1000 - point.y/0.01;
        // if(point.z > threshold)  
        if(mat_gray.at<uchar>(m, n) == 255)
        {
          cloud_noplane->points.push_back(point);
          mat_brinary.at<uchar>(m, n) = 0;
        }
    }
    // cv::imshow("mat_brinary", mat_brinary);
    // cv::waitKey(1);
    // std::cout<<"140"<<std::endl;
}

void GetMapPng::LaserOdometryHandler(nav_msgs::Odometry laserOdometry)
{
    time_new_odom_receive = laserOdometry.header.stamp.toSec();
    // gp_odom_q_curr = laserOdometry.pose.pose.orientation;
    // gp_odom_q_curr(laserOdometry.pose.pose.orientation.w, laserOdometry.pose.pose.orientation.x,
    //               laserOdometry.pose.pose.orientation.y, laserOdometry.pose.pose.orientation.z);
    gp_odom_q_curr.x() = laserOdometry.pose.pose.orientation.x;
    gp_odom_q_curr.y() = laserOdometry.pose.pose.orientation.y;
    gp_odom_q_curr.z() = laserOdometry.pose.pose.orientation.z;
    gp_odom_q_curr.w() = laserOdometry.pose.pose.orientation.w;

    gp_odom_t_curr.x() = laserOdometry.pose.pose.position.x;
    gp_odom_t_curr.y() = laserOdometry.pose.pose.position.y;
    gp_odom_t_curr.z() = laserOdometry.pose.pose.position.z;

    // gp_odom_t_curr(laserOdometry.pose.pose.position.x, laserOdometry.pose.pose.position.y,
    //                 laserOdometry.pose.pose.position.z);
    new_odom_receive = true;
}


int GetMapPng::Get_Threshold_New(cv::Mat src)
{
  const int step = 20;
  int threshold = 0;
  int means_object = 0;
  int object_num = 0;
  int means_back = 0;
  int back_num = 0;
  int threshold_last = 105; //115
  //计算直方图
  int height = src.rows;
  int width = src.cols;
  int pixelCount[256] = {0};
  int pixelCount_now[256] = {0};

  //统计灰度级中每个像素在整幅图像中的个数
  for (size_t i = 0; i < height; i++)
  {
    for (size_t j = 0; j < width; j++)
    {
      if (src.at<uchar>(i, j) != 255 && src.at<uchar>(i, j) != 0)
      {
        pixelCount[src.at<uchar>(i, j)]++; //将像素值作为计数数组的下标
      }
    }
  }

  int start = step - step / 2;
  int end = 256 - step / 2;
  double temp;
  //直方图优化
  bool frist = true;
  for (int i = start; i < end; i++)
  {

    if (frist)
    {
      for (int j = 0 - step / 2; j < step / 2; j++)
      {
        temp += pixelCount[(int)(i + j)];
      }
      frist = false;
    }
    else
      temp = temp - pixelCount[(int)(i - step / 2 - 1)] + pixelCount[(int)(i + step / 2)];
    pixelCount_now[i] = (int)(temp / step);
  }

  if(1)
  {
    //显示直方图，确认无误
    cv::Mat dstHist = cv::Mat(801, 256, CV_8UC1, cv::Scalar(255));
    for (size_t i = 0; i < 256; i++)
    {
      cv::line(dstHist, cv::Point(i, 800), cv::Point(i, 800 - pixelCount_now[i] / 7), cv::Scalar(0), 1, 1);
    }
    cv::imshow("dstHist", dstHist);
    cv::waitKey(1);
  }

  int max_threshold = 255;
  std::vector<int> mount;
  for (size_t i = 30; i < 220; i++)
  {
    if (pixelCount_now[i] > 1000)
    {
      if (pixelCount_now[max_threshold] < pixelCount_now[i])
        max_threshold = i;
      if (pixelCount_now[i] >= pixelCount_now[i - 1] && pixelCount_now[i] >= pixelCount_now[i + 1] &&
          pixelCount_now[i] >= pixelCount_now[i - 2] && pixelCount_now[i] >= pixelCount_now[i + 2])
      {
        mount.push_back(i);
        std::cout << "mount" << i << "  " << pixelCount_now[i] << std::endl;
      }
    }
  }

  if (mount.size() > 1)
  {
    for (size_t i = 0; i < mount.size() - 1; i++)
    {
      if (mount[i + 1] - mount[i] < 20 || (mount[i + 1] - mount[i] < 50 && abs(pixelCount_now[mount[i + 1]] - pixelCount_now[mount[i]]) < 500))
      {

        mount[i] = (mount[i + 1] + mount[i]) / 2;
        mount.erase(mount.begin() + i + 1);
        i--;
      }
    }
    if (mount.size() == 2 && (mount[0] + mount[1]) / 2 < max_threshold)
    {
      // ADEBUG<<"using two mount ways";
      std::cout << "size" << mount.size() << "  threshold" << (mount[0] + mount[1]) / 2 << std::endl;
      return (mount[0] + mount[1]) / 2;
    }
    else
    {
      // ADEBUG<<"don't using two mount ways ."<<" size = "<<mount.size();
      for (size_t i = 0; i < mount.size(); i++)
      {
        std::cout << "mount" << mount[i] << std::endl;
      }
    }
  }
  // else ADEBUG<<"don't using two mount ways, size<2.";

  while (abs(threshold - threshold_last) > 2)
  {
    means_back = 0;
    means_object = 0;
    back_num = 0;
    object_num = 0;
    threshold = threshold_last;
    for (size_t i = 1; i < 255; i++)
    {
      if (i < threshold)
      {
        means_back += pixelCount[i] * i;
        back_num += pixelCount[i];
      }
      else
      {
        means_object += pixelCount[i] * i;
        object_num += pixelCount[i];
      }
    }
    threshold_last = (means_back / (back_num + 0.001) + means_object / (object_num + 0.001)) / 2;
  }
  return threshold;
}



int GetMapPng::Get_Threshold_Img(cv::Mat src, cv::Mat dstImage)
{
  cv::Mat src1 = cv::Mat(src.rows * 4 / 10, src.cols, CV_8UC1, cv::Scalar(0));
  cv::Mat src2 = cv::Mat(src.rows * 6 / 10, src.cols, CV_8UC1, cv::Scalar(0));

  for (int i = 0; i < src.rows; i++)
  {
    for (int j = 0; j < src.cols; j++)
    {
      if (i < src.rows * 4 / 10)
      {
        src1.at<uchar>(i, j) = src.at<uchar>(i, j);
      }
      else
      {
        src2.at<uchar>(i - src.rows * 4 / 10, j) = src.at<uchar>(i, j);
      }
    }
  }

  int threshold1, threshold2;
  threshold1 = Get_Threshold_New(src1);
  threshold2 = Get_Threshold_New(src2);
  std::cout << "threshold1  " << threshold1 << "    threshold2   " << threshold2 << std::endl;
  if (threshold1 - threshold2 > 10)
    threshold1 = threshold2;
  if (threshold2 - threshold1 > 10)
    threshold2 = threshold1;
  cv::threshold(src1, src1, threshold1, 255, cv::THRESH_BINARY_INV);
  cv::threshold(src2, src2, threshold1, 255, cv::THRESH_BINARY_INV);

  for (int i = 0; i < src.rows; i++)
  {
    for (int j = 0; j < src.cols; j++)
    {
      if (i < src.rows * 4 / 10)
      {
        dstImage.at<uchar>(i, j) = src1.at<uchar>(i, j);
      }
      else
      {
        dstImage.at<uchar>(i, j) = src2.at<uchar>(i - src.rows * 4 / 10, j);
      }
    }
  }
  // return std::max<int>(threshold1, threshold2);
  return threshold1;
}



//将点云转到实际位置
void GetMapPng::PointAssociateToBeMapped(pcl::PointXYZI &pi )
{
    // std::cout<<"rotationMatrix.inverse()  "<<rotationMatrix.inverse()<<std::endl;
    // std::cout<<"rotationMatrix  "<<rotationMatrix<<std::endl;
    Eigen::Vector3d point_w( pi.x, pi.y, pi.z );
    Eigen::Vector3d point_curr = /* rotationMatrix.inverse() * */point_w ;//将点云转换到未矫正时

    // if(!std::isnan(rotationMatrix.inverse()(0,0)))
    //   point_curr = rotationMatrix.inverse() * point_curr;
    //将点云转换到实际车身位置
    point_curr = gp_odom_q_curr * point_curr;//将点云姿态转到实际
    point_curr = gp_odom_t_curr + point_curr;//将点云位置转到实际

    pi.x = point_curr.x();
    pi.y = point_curr.y();
    pi.z = point_curr.z();
    // po->intensity = pi->intensity;
}

void GetMapPng::PointCloudAssociateToMap(pcl::PointCloud<pcl::PointXYZI> &pc_cloud)
{
    int points_size = pc_cloud.points.size();
    
    for(size_t i = 0; i < points_size; i++)
    {
        PointAssociateToBeMapped(pc_cloud.points[i]);
    }
}

/*
函数功能：  得到平面系数 */
Eigen::Matrix4f GetMapPng::CreateRotateMatrix(Eigen::Vector3f before, Eigen::Vector3f after)
{

  before.normalize();
  after.normalize();

  float angle = acos(before.dot(after));
  Eigen::Vector3f p_rotate = before.cross(after);
  p_rotate.normalize();

  // Eigen::Matrix3f rotationMatrix;
  rotationMatrix(0, 0) = cos(angle) + p_rotate[0] * p_rotate[0] * (1 - cos(angle));
  rotationMatrix(0, 1) = p_rotate[0] * p_rotate[1] * (1 - cos(angle) - p_rotate[2] * sin(angle)); //这里跟公式比多了一个括号，但是看实验结果它是对的。
  rotationMatrix(0, 2) = p_rotate[1] * sin(angle) + p_rotate[0] * p_rotate[2] * (1 - cos(angle));

  rotationMatrix(1, 0) = p_rotate[2] * sin(angle) + p_rotate[0] * p_rotate[1] * (1 - cos(angle));
  rotationMatrix(1, 1) = cos(angle) + p_rotate[1] * p_rotate[1] * (1 - cos(angle));
  rotationMatrix(1, 2) = -p_rotate[0] * sin(angle) + p_rotate[1] * p_rotate[2] * (1 - cos(angle));

  rotationMatrix(2, 0) = -p_rotate[1] * sin(angle) + p_rotate[0] * p_rotate[2] * (1 - cos(angle));
  rotationMatrix(2, 1) = p_rotate[0] * sin(angle) + p_rotate[1] * p_rotate[2] * (1 - cos(angle));
  rotationMatrix(2, 2) = cos(angle) + p_rotate[2] * p_rotate[2] * (1 - cos(angle));

  Eigen::Vector3d ea = rotationMatrix.eulerAngles(2, 1, 0);
  // ea[0]=3.1415;
  // ea[2]=-3.1415;

  // std::cout<<"ea[0]  "<<ea[0]<<"  "<<"ea[1]  "<<ea[1]<<"  "<<"ea[2]  "<<ea[2]<<std::endl;
  Eigen::Matrix3d rotation_matrix3;
  rotation_matrix3 = Eigen::AngleAxisd(ea[0], Eigen::Vector3d::UnitZ()) *
                     Eigen::AngleAxisd(ea[1], Eigen::Vector3d::UnitY()) *
                     Eigen::AngleAxisd(ea[2], Eigen::Vector3d::UnitX());

  Eigen::Matrix4f rotationMatrix1;
  for (size_t i = 0; i < 3; i++)
    for (size_t j = 0; j < 3; j++)
    {
      rotationMatrix1(i, j) = rotation_matrix3(i, j);
    }
  return rotationMatrix1;
}

/*
函数功能：  矫正点云
输入：  代矫正点云
输出:  矫正后点云
 */
void GetMapPng::Correction_Cloud(pcl::PointCloud<pcl::PointXYZI>::Ptr &point_cloud)
{
  // Eigen::Vector3d euler_angle(livox_yaw_, livox_pitch_, livox_roll_);
  // Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(euler_angle(2), Eigen::Vector3d::UnitX()));
  // Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(euler_angle(1), Eigen::Vector3d::UnitY()));
  // Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(euler_angle(0), Eigen::Vector3d::UnitZ()));

  // Eigen::Quaterniond quaternion;
  // quaternion = yawAngle * pitchAngle * rollAngle;
  // Eigen::Translation3d translation(livox_x_, livox_y_, livox_z_);
  // Eigen::Affine3d affine = translation * quaternion;
  // Eigen::Matrix4d matrix = affine.matrix();

  // pcl::transformPointCloud(*point_cloud, *point_cloud, matrix);

  pcl::PointCloud<pcl::PointXYZI>::Ptr point_seg(new pcl::PointCloud<pcl::PointXYZI>);
  for (size_t i = 0; i < point_cloud->points.size(); i++)
  {
    if (point_cloud->points[i].x < 10.0 && point_cloud->points[i].y < 3.0 && point_cloud->points[i].y > -0.5 && point_cloud->points[i].z < -1.0)
      point_seg->points.push_back(point_cloud->points[i]);
  }
  std::cout<<"point_seg->points.size()  "<<point_seg->points.size()<<std::endl;
  if (point_seg->points.size() > 100)
  {
    pcl::SACSegmentation<pcl::PointXYZI> plane_seg;
    pcl::PointIndices::Ptr plane_inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr plane_coefficients(new pcl::ModelCoefficients);
    plane_seg.setOptimizeCoefficients(true);
    plane_seg.setModelType(pcl::SACMODEL_PLANE);
    plane_seg.setMethodType(pcl::SAC_RANSAC);
    plane_seg.setDistanceThreshold(0.03);
    plane_seg.setInputCloud(point_seg);
    plane_seg.segment(*plane_inliers, *plane_coefficients); //得到平面系数，进而得到平面法向量

    Eigen::Vector3f before, after;
    before[0] = plane_coefficients->values[0];
    before[1] = plane_coefficients->values[1];
    // before[0]=0.0;
    // before[1]=0.0;
    before[2] = plane_coefficients->values[2];

    after[0] = 0.0;
    after[1] = 0.0;
    after[2] = 1.0;
    pcl::transformPointCloud(*point_cloud, *point_cloud, CreateRotateMatrix(before, after));
  }
}

void GetMapPng::MapToPng(int width, int hight)
{
    int center_x = (x_min + x_max)/2;
    int center_y = (y_min + y_max)/2;
    std::cout<<"center_x  center_y"<<center_x<<"  "<<center_y<<std::endl;
    std::cout<<"hight  width"<<hight<<"  "<<width<<std::endl;
    std::cout<<"hight  width"<<hight<<"  "<<width<<std::endl;
    cv::Mat map_png = cv::Mat(hight, width, CV_8UC1, cv::Scalar(100));
    int x_ = 0, y_ = 0; 
    for(size_t i = 0; i < map_points_.size(); i++)
    {
        // std::cout<<"point_x point_y"<<map_points_[i].x<<"  "<<map_points_[i].y<<std::endl;
        int x = hight - (map_points_[i].x - x_min);
        int y = width/2 -  (map_points_[i].y - center_y);

        // std::cout<<"x y"<<x<<"  "<<y<<std::endl;

        // std::max
        if(x > width-1 || y> hight-1) std::cout<<"x y"<<x<<"  "<<y<<std::endl;
        if(x > 1 && x < width - 1 && y > 1 && y < hight-1)
        map_png.at<uchar>(x, y) = 255;
    }
    cv::imshow("map_png", map_png);
    cv::waitKey(1);
    // cv::imwrite("~/path/map.png", map_png);

    std::cout<<"506"<<std::endl;
}

void GetMapPng::AddPointCloudToMap(int & width, int & hight)
{
    // int x_max = -100, x_min = 1000000, y_max = -100, y_min = 1000000;
    // std::vector<cv::Point2i > map_points_;
    cv::Point2i point_xy;
    pcl::PointXYZI point_xyz;
    for(size_t i = 0; i < cloud_noplane->points.size(); i++)
    {
        point_xyz = cloud_noplane->points[i];
        point_xy.x = point_xyz.x / 0.02;//分辨率1cm
        point_xy.y = point_xyz.y / 0.02;
        map_points_.push_back(point_xy);
        x_max = std::max<int>(x_max, point_xy.x);
        x_min = std::min<int>(x_min, point_xy.x);
        y_max = std::max<int>(y_max, point_xy.y);
        y_min = std::min<int>(y_min, point_xy.y);
    } 
    hight = x_max - x_min;
    width = y_max - y_min;

    std::cout<<"x_max  "<<x_max<<"  x_min"<<x_min<<std::endl;
}

void GetMapPng::RunGetMapPng()
{
  // counts++;
  // if(counts % 5 != 1)  return;
    // 如果有新数据进来则执行，否则不执行任何操作
    if(new_cloud_receive && new_odom_receive &&
       std::abs(time_new_odom_receive - time_new_cloud_receive) < 0.05)
    {
      counts++;
      new_odom_receive = false;
      new_cloud_receive = false;
    }else{
      return;
    }
    if(counts % 5 != 1)  return;
    //对点云矫正
    Correction_Cloud(cloud_input);
    std::cout<<"535"<<std::endl;
    //得到想要的点云部分，即去除地面的点云
    GetRioCloud();
    //对点云变换回到实际的位置
    // *cloud_noplane = *cloud_input;
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud_noplane, *cloud_noplane, indices);
    PointCloudAssociateToMap(*cloud_noplane);
    //将点云添加到全局地图
    // *cloud_map +=  *cloud_noplane;
    for(size_t i = 0; i < cloud_noplane->points.size(); i++)
    {
      cloud_map->points.push_back(cloud_noplane->points[i]);
    }

     pcl::PointCloud<pcl::PointXYZI> cloud_map_ds;

    m_down_sample_filter_points.setInputCloud(cloud_map);
    m_down_sample_filter_points.filter(cloud_map_ds);
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(cloud_map_ds, output);
    output.header.frame_id = "odom";
    gp_pub_cloud.publish(output);//发布数据;

    if(counts % 20 == 1)
    {
      cloud_map_ds.width = 1;
      cloud_map_ds.height = cloud_map_ds.points.size();
      std::cout<<"size  "<<cloud_map_ds.points.size()<<std::endl;
      std::cout<<"save pcd"<<std::endl;
      pcl::io::savePCDFile("/home/cidi/Loam_livox_pcd/surround10.pcd", cloud_map_ds);//目标点云
    }

    cloud_input->clear();
    cloud_noplane->clear();
    // counts++;

}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "GetMapPng");
    GetMapPng gp;

    ros::Rate rate(200);
    while (ros::ok())
    {
        ros::spinOnce();

        gp.RunGetMapPng();

        rate.sleep();
    }
    ros::spin();
    return 0;
}