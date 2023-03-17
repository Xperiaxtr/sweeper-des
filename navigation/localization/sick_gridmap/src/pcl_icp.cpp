#include<ros/ros.h>
#include<sensor_msgs/PointCloud2.h>
#include<sensor_msgs/PointCloud2.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include<pcl/registration/gicp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>//去除NAN点的头文件
#include<pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>//除去离散点的头文件
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/registration/gicp.h>

#include <pcl/registration/ndt.h> //ndt头文件

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>

#include <Kalman.h>

#include<fstream>  //ifstream
#include<iostream>
#include<sstream>
#include<string>     //包含getline()
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>


using namespace std;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_fristsource(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_fristtarget(new pcl::PointCloud<pcl::PointXYZ>);
ros::Publisher pub,pub_odom,pub_odom_path,rviz_path,car_path,debug_odom,icp_is_ok;
int color=0;
double angle, location[3];//记录叠加数据结果
std_msgs::Bool icp_ok;

Eigen::Matrix4f transformation_last;
nav_msgs::Path m_laser_after_mapped_path;
double score;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tgt(new pcl::PointCloud<pcl::PointXYZ>);


struct Kalman
{
  double X_last_last[7];//a,b,c
  double X_now_last[7];
  double X_now_now[7];
  double P_last_last[7];
  double P_now_last[7];
  double P_now_now[7];
  double Z_now[7];
  double Kg_now[7];
  double R[7];
  double Q[7];

  
};
Kalman kalman;


//轨迹集合
vector<geometry_msgs::PoseStamped> all_pose;

void GetPose(vector<geometry_msgs::PoseStamped> &all__pose, std::string file)
{
    std::string line, str;
    ifstream inf;
    int i=0;
    inf.open("/home/cidi/path/path.txt",ios::in);
    geometry_msgs::PoseStamped pose_file;
    // vector<geometry_msgs::PoseStamped> all_pose;
    vector<std::string > two_string;
    while (getline(inf,line))
    {      
        
        istringstream stream(line);
        if(i==1) 
        {
            while(stream>>str)
            {
                two_string.push_back(str);
            }
            pose_file.pose.position.x = atof(two_string[1].c_str());
            two_string.clear();
        }
        else if(i==2)
        {
            while(stream>>str)
            {
                two_string.push_back(str);
            }
            pose_file.pose.position.y = atof(two_string[1].c_str());
            two_string.clear();
        }
        else if(i==3)
        {
            while(stream>>str)
            {
                two_string.push_back(str);
            }
            pose_file.pose.position.z = atof(two_string[1].c_str());
            two_string.clear();
        }
        else if(i==5)
        {
            while(stream>>str)
            {
                two_string.push_back(str);
            }
            pose_file.pose.orientation.x = atof(two_string[1].c_str());
            two_string.clear();
        }
        else if(i==6)
        {
            while(stream>>str)
            {
                two_string.push_back(str);
            }
            pose_file.pose.orientation.y = atof(two_string[1].c_str());
            two_string.clear();
        }
        else if(i==7)
        {
            while(stream>>str)
            {
                two_string.push_back(str);
            }
            pose_file.pose.orientation.z = atof(two_string[1].c_str());
            two_string.clear();
        }
        else if(i==8)
        {
            while(stream>>str)
            {
                two_string.push_back(str);
            }
            pose_file.pose.orientation.w = atof(two_string[1].c_str());
            two_string.clear();
        }
        all__pose.push_back(pose_file);
        i=i%10;
        i++;

    }
    if(all__pose.size()>10)
    std::cout<<"file opened successfully!"<<std::endl;
    else std::cout<<"file opened failure !"<<std::endl;
    inf.close();
}



KalmanFilter kf;
double now_timestamp,last_timestamp;
// Eigen::VectorXd 
void StartKalman()
{
    double m_x,m_y,m_z,m_yaw,m_roll,m_pitch,m_vx,m_vy;
    m_x = transformation_last(0, 3);
    m_y = transformation_last(1, 3);
    m_z = transformation_last(2, 3);

    Eigen::Matrix3d R1;
    R1(0,0)=transformation_last(0,0);
    R1(0,1)=transformation_last(0,1);
    R1(0,2)=transformation_last(0,2);
    R1(1,0)=transformation_last(1,0);
    R1(2,0)=transformation_last(2,0);
    R1(1,1)=transformation_last(1,1);
    R1(1,2)=transformation_last(1,2);
    R1(2,1)=transformation_last(2,1);
    R1(2,2)=transformation_last(2,2);
    Eigen::Vector3d euler_angles = R1.eulerAngles(2, 1, 0);
    m_yaw = euler_angles[0];
    m_pitch = euler_angles[1];
    m_roll = euler_angles[2];

    if(!kf.IsInitialized()){
        Eigen::VectorXd x_in(8,1);
        x_in <<m_x,m_y,m_z,m_yaw,m_pitch,m_roll,1.0,0.0;
        kf.Initialization(x_in);

        Eigen::MatrixXd P_in(8,8);
        P_in<< 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10.0;
        kf.SetP(P_in);

        Eigen::MatrixXd Q_in(8,8);
        Q_in<< 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
        kf.SetQ(Q_in);

        Eigen::MatrixXd H_in(6,8);
        H_in<< 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0;

        kf.SetH(H_in);

        Eigen::MatrixXd R_in(6,6);

        R_in<< 0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.05, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.005, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.001, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.001, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 0.001; 
        kf.SetR(R_in);

        return;       
        
    }

    //double delta_t = now_timestamp - last_timestamp;
    double delta_t =0.1;
    Eigen::MatrixXd F_in(8,8);
    F_in <<    1.0, 0.0, 0.0, 0.0, 0.0, 0.0, delta_t, 0.0,
               0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, delta_t,
               0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
               0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
    kf.SetF(F_in);
    kf.Prediction();

    Eigen::VectorXd z(6,1);
    z<< m_x,m_y,m_z,m_yaw,m_roll,m_pitch;
    kf.MeasurementUpdate(z);

    Eigen::VectorXd x_out=kf.GetX();

    transformation_last(0, 3)=x_out(0);
    transformation_last(1, 3)=x_out(1);
    transformation_last(2, 3)=x_out(2);

    euler_angles(0)=x_out(3);
    euler_angles(1)=x_out(4);
    euler_angles(2)=x_out(5);

    // std::cout<<"vx"<<x_out(6)<<std::endl;
    // std::cout<<"vy"<<x_out(7)<<std::endl;

    R1 = Eigen::AngleAxisd(euler_angles[0], Eigen::Vector3d::UnitZ()) * 
                       Eigen::AngleAxisd(euler_angles[1], Eigen::Vector3d::UnitY()) * 
                       Eigen::AngleAxisd(euler_angles[2], Eigen::Vector3d::UnitX());

    // transformation_last(0,0)=R1(0,0);
    // transformation_last(0,1)=R1(0,1);
    // transformation_last(0,2)=R1(0,2);
    // transformation_last(1,0)=R1(1,0);
    // transformation_last(2,0)=R1(2,0);
    // transformation_last(1,1)=R1(1,1);
    // transformation_last(1,2)=R1(1,2);
    // transformation_last(2,1)=R1(2,1);
    // transformation_last(2,2)=R1(2,2);


        nav_msgs::Odometry odomAftMapped;
    odomAftMapped.header.frame_id = "aft_mapped";
    odomAftMapped.header.stamp = ros::Time::now();
    odomAftMapped.pose.pose.orientation.x = x_out(0);
    odomAftMapped.pose.pose.orientation.y = x_out(1);
    odomAftMapped.pose.pose.orientation.z = x_out(2);
    odomAftMapped.pose.pose.orientation.w = x_out(3);

    odomAftMapped.pose.pose.position.x = x_out(4);
    odomAftMapped.pose.pose.position.y = x_out(6);
    odomAftMapped.pose.pose.position.z = x_out(7);

    odomAftMapped.twist.twist.linear.x=score;
    debug_odom.publish(odomAftMapped);
}


void GetKalman()
{

    Eigen::Matrix3d R1;
    R1(0,0)=transformation_last(0,0);
    R1(0,1)=transformation_last(0,1);
    R1(0,2)=transformation_last(0,2);
    R1(1,0)=transformation_last(1,0);
    R1(2,0)=transformation_last(2,0);
    R1(1,1)=transformation_last(1,1);
    R1(1,2)=transformation_last(1,2);
    R1(2,1)=transformation_last(2,1);
    R1(2,2)=transformation_last(2,2);
    Eigen::Quaterniond Q1;
    Q1=R1;
    //Eigen::Vector3d euler_angles = R1.eulerAngles(2, 1, 0);
    // m_yaw = euler_angles(0);
    // m_pitch = euler_angles(1);
    // m_roll = euler_angles(2);

  //计算预测值
  kalman.X_now_last[0]=kalman.X_last_last[0];
  kalman.X_now_last[1]=kalman.X_last_last[1];
  kalman.X_now_last[2]=kalman.X_last_last[2];
  kalman.X_now_last[3]=kalman.X_last_last[3];
  kalman.X_now_last[4]=kalman.X_last_last[4];
  kalman.X_now_last[5]=kalman.X_last_last[5];
  kalman.X_now_last[6]=kalman.X_last_last[6];

  kalman.P_now_last[0] = kalman.P_last_last[0]+kalman.Q[0];
  kalman.P_now_last[1] = kalman.P_last_last[1]+kalman.Q[1];
  kalman.P_now_last[2] = kalman.P_last_last[2]+kalman.Q[2];
  kalman.P_now_last[3] = kalman.P_last_last[0]+kalman.Q[3];
  kalman.P_now_last[4] = kalman.P_last_last[1]+kalman.Q[4];
  kalman.P_now_last[5] = kalman.P_last_last[2]+kalman.Q[5];
  kalman.P_now_last[6] = kalman.P_last_last[2]+kalman.Q[6];
  //计算测量值
  kalman.Z_now[0]=transformation_last(0, 3);
  kalman.Z_now[1]=transformation_last(1, 3);
  kalman.Z_now[2]=transformation_last(2, 3);
  kalman.Z_now[3]=Q1.x();
  kalman.Z_now[4]=Q1.y();
  kalman.Z_now[5]=Q1.z();
  kalman.Z_now[6]=Q1.w();
  //计算卡尔曼增益
  kalman.Kg_now[0] = kalman.P_now_last[0]/(kalman.P_now_last[0]+kalman.R[0]);
  kalman.Kg_now[1] = kalman.P_now_last[1]/(kalman.P_now_last[1]+kalman.R[1]);
  kalman.Kg_now[2] = kalman.P_now_last[2]/(kalman.P_now_last[2]+kalman.R[2]);
  kalman.Kg_now[3] = kalman.P_now_last[3]/(kalman.P_now_last[3]+kalman.R[3]);
  kalman.Kg_now[4] = kalman.P_now_last[4]/(kalman.P_now_last[4]+kalman.R[4]);
  kalman.Kg_now[5] = kalman.P_now_last[5]/(kalman.P_now_last[5]+kalman.R[5]);
  kalman.Kg_now[6] = kalman.P_now_last[6]/(kalman.P_now_last[6]+kalman.R[6]);
  //
  //double cell;
  //cell<<1.0,1.0,1.0;
  kalman.X_now_now[0] = kalman.X_now_last[0]+kalman.Kg_now[0]*(kalman.Z_now[0]-kalman.X_now_last[0]);
  kalman.X_now_now[1] = kalman.X_now_last[1]+kalman.Kg_now[1]*(kalman.Z_now[1]-kalman.X_now_last[1]);
  kalman.X_now_now[2] = kalman.X_now_last[2]+kalman.Kg_now[2]*(kalman.Z_now[2]-kalman.X_now_last[2]);
  kalman.X_now_now[3] = kalman.X_now_last[3]+kalman.Kg_now[3]*(kalman.Z_now[3]-kalman.X_now_last[3]);
  kalman.X_now_now[4] = kalman.X_now_last[4]+kalman.Kg_now[4]*(kalman.Z_now[4]-kalman.X_now_last[4]);
  kalman.X_now_now[5] = kalman.X_now_last[5]+kalman.Kg_now[5]*(kalman.Z_now[5]-kalman.X_now_last[5]);
  kalman.X_now_now[6] = kalman.X_now_last[6]+kalman.Kg_now[6]*(kalman.Z_now[6]-kalman.X_now_last[6]);

  kalman.P_now_now[0] = (1.0-kalman.Kg_now[0])*kalman.P_now_last[0];
  kalman.P_now_now[1] = (1.0-kalman.Kg_now[1])*kalman.P_now_last[1];
  kalman.P_now_now[2] = (1.0-kalman.Kg_now[2])*kalman.P_now_last[2];
  kalman.P_now_now[3] = (1.0-kalman.Kg_now[3])*kalman.P_now_last[3];
  kalman.P_now_now[4] = (1.0-kalman.Kg_now[4])*kalman.P_now_last[4];
  kalman.P_now_now[5] = (1.0-kalman.Kg_now[5])*kalman.P_now_last[5];
  kalman.P_now_now[6] = (1.0-kalman.Kg_now[6])*kalman.P_now_last[6];

  kalman.X_last_last[0]=kalman.X_now_now[0];
  kalman.X_last_last[1]=kalman.X_now_now[1];
  kalman.X_last_last[2]=kalman.X_now_now[2];
  kalman.X_last_last[3]=kalman.X_now_now[3];
  kalman.X_last_last[4]=kalman.X_now_now[4];
  kalman.X_last_last[5]=kalman.X_now_now[5];
  kalman.X_last_last[6]=kalman.X_now_now[6];

  kalman.P_last_last[0]=kalman.P_now_now[0];
  kalman.P_last_last[1]=kalman.P_now_now[1];
  kalman.P_last_last[2]=kalman.P_now_now[2];
  kalman.P_last_last[3]=kalman.P_now_now[3];
  kalman.P_last_last[4]=kalman.P_now_now[4];
  kalman.P_last_last[5]=kalman.P_now_now[5];
  kalman.P_last_last[6]=kalman.P_now_now[6];

  // std::cout<<"求出的方程"<<mat_k.at<double>(2, 0)<<"  "<<mat_k.at<double>(1, 0)<<" "<<mat_k.at<double>(0, 0)<<std::endl;
  // std::cout<<"修正后的"<<kalman.X_now_now[0]<<" "<<kalman.X_now_now[1]<<" "<<kalman.X_now_now[2]<<std::endl;
  transformation_last(0, 3)=kalman.X_now_now[0];
  transformation_last(1, 3)=kalman.X_now_now[1];
  transformation_last(2, 3)=kalman.X_now_now[2];

  R1=Q1;
    //   R1=Eigen::AngleAxisd(euler_angles[0], Eigen::Vector3d::UnitZ()) * 
    //                        Eigen::AngleAxisd(euler_angles[1], Eigen::Vector3d::UnitY()) * 
    //                        Eigen::AngleAxisd(euler_angles[2], Eigen::Vector3d::UnitX());

    // transformation_last(0,0) = R1(0,0);
    // transformation_last(0,1) = R1(0,1);
    // transformation_last(0,2) = R1(0,2);
    // transformation_last(1,0) = R1(1,0);
    // transformation_last(2,0) = R1(2,0);
    // transformation_last(1,1) = R1(1,1);
    // transformation_last(1,2) = R1(1,2);
    // transformation_last(2,1) = R1(2,1);
    // transformation_last(2,2) = R1(2,2);

}

void visualize_pcd(PointCloud::Ptr pcd_src, PointCloud::Ptr pcd_tgt, PointCloud::Ptr pcd_final)
{
	pcl::visualization::PCLVisualizer viewer("registration Viewer");
	//原始点云绿色
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> src_h(pcd_src, 0, 255, 0);
	//目标点云红色
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tgt_h(pcd_tgt, 255, 0, 0);
	//匹配好的点云蓝色
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> final_h(pcd_final, 0, 0, 255);

	viewer.setBackgroundColor(255,255,255);
	viewer.addPointCloud(pcd_src, src_h, "source cloud");
	viewer.addPointCloud(pcd_tgt, tgt_h, "target cloud");
	viewer.addPointCloud(pcd_final, final_h, "result cloud");
	while (!viewer.wasStopped())
	{
		viewer.spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}

}

void Color_Cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_src)
{
    int rgb[3];
    if(color==0)//绿色
        {
            rgb[0]=0;
            rgb[1]=255;
            rgb[2]=0;
        }
    else if(color==1)//红
        {
            rgb[0]=255;
            rgb[1]=0;
            rgb[2]=0;
        }
    else if(color==2)//蓝
        {
            rgb[0]=255;
            rgb[1]=0;
            rgb[2]=255;
        }
    int i=0;
    for(i;i<cloud_src->points.size();i++)
    {
        cloud_src->points[i].r=rgb[0];
        cloud_src->points[i].g=rgb[1];
        cloud_src->points[i].b=rgb[2];
    }
    color++;
    if(color==3) color=0;
}


//ros可视化数据
void visualize_cloud(PointCloud::Ptr pcd_src, PointCloud::Ptr pcd_tgt, PointCloud::Ptr pcd_final)
{

    pcl::PointCloud<pcl::PointXYZRGB> pcl_output;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr src_color(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr tag_color(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_color(new pcl::PointCloud<pcl::PointXYZRGB>);

    copyPointCloud(*pcd_src, *src_color);
    copyPointCloud(*pcd_tgt, *tag_color);
    copyPointCloud(*pcd_final, *final_color);

    Color_Cloud(src_color);
    Color_Cloud(tag_color);
    Color_Cloud(final_color);


    pcl_output = *src_color + *tag_color;
    pcl_output += *final_color;

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(pcl_output, output);
    output.header.frame_id = "aft_mapped";
    pub.publish(output);//发布数据;
}


Eigen::Quaterniond MapPathToRviz(Eigen::Matrix3d R1, geometry_msgs::PoseStamped path_map, Eigen::Matrix4f& point_trans)
{
    // Eigen::Matrix4f point_trans;
    Eigen::Quaterniond Q1;
    // Eigen::Quaterniond Q1(path_map.pose.orientation.w,path_map.pose.orientation.x,path_map.pose.orientation.y,path_map.pose.orientation.z);
    // Eigen::Vector3d eulerAngle1 = Q1.matrix().eulerAngles(2,1,0);
    // cout << "yaw(z) pitch(y) roll(x) = " << eulerAngle1.transpose() << endl;
    Eigen::Matrix3d R2;//=Q1.matrix();;
    point_trans(0,0)=1.0;
    point_trans(0,1)=0.0;
    point_trans(0,2)=0.0;
    point_trans(1,0)=0.0;
    point_trans(1,1)=1.0;
    point_trans(1,2)=0.0;
    point_trans(2,0)=0.0;
    point_trans(2,1)=0.0;
    point_trans(2,2)=1.0;

    point_trans(0,3)=path_map.pose.position.x;
    point_trans(1,3)=path_map.pose.position.y;
    point_trans(2,3)=path_map.pose.position.z;

    point_trans(3,0)=0;
    point_trans(3,1)=0;
    point_trans(3,2)=0;
    point_trans(3,3)=1;

    point_trans=transformation_last.inverse()*point_trans;

    R2(0,0)=point_trans(0,0);
    R2(0,1)=point_trans(0,1);
    R2(0,2)=point_trans(0,2);
    R2(1,0)=point_trans(1,0);
    R2(2,0)=point_trans(2,0);
    R2(1,1)=point_trans(1,1);
    R2(1,2)=point_trans(1,2);
    R2(2,1)=point_trans(2,1);
    R2(2,2)=point_trans(2,2);
    // R2=R2*R1.inverse();
    Q1=R2;
    return Q1;
}

//发布odom 和path
void Pub_odom(Eigen::Matrix4f result_trans)
{
    // std::cout<<result_trans<<std::endl;
    Eigen::Matrix3d R1;
    R1(0,0)=result_trans(0,0);
    R1(0,1)=result_trans(0,1);
    R1(0,2)=result_trans(0,2);
    R1(1,0)=result_trans(1,0);
    R1(2,0)=result_trans(2,0);
    R1(1,1)=result_trans(1,1);
    R1(1,2)=result_trans(1,2);
    R1(2,1)=result_trans(2,1);
    R1(2,2)=result_trans(2,2);
    Eigen::Quaterniond Q1;
    Q1 = R1;
    Eigen::Vector3d eulerAngle1 = Q1.matrix().eulerAngles(2,1,0);
    // cout << "   当前点云yaw(z) pitch(y) roll(x) = " << eulerAngle1.transpose() << endl;
    nav_msgs::Odometry odomAftMapped;
    odomAftMapped.header.frame_id = "aft_mapped";
    odomAftMapped.header.stamp = ros::Time::now();
    odomAftMapped.pose.pose.orientation.x = Q1.x();
    odomAftMapped.pose.pose.orientation.y = Q1.y();
    odomAftMapped.pose.pose.orientation.z = Q1.z();
    odomAftMapped.pose.pose.orientation.w = Q1.w();

    odomAftMapped.pose.pose.position.x = result_trans(0, 3);
    odomAftMapped.pose.pose.position.y = result_trans(1, 3);
    odomAftMapped.pose.pose.position.z = result_trans(2, 3);
    pub_odom.publish(odomAftMapped);

    geometry_msgs::PoseStamped laserAfterMappedPose;
    laserAfterMappedPose.header = odomAftMapped.header;
    laserAfterMappedPose.pose = odomAftMapped.pose.pose;
    m_laser_after_mapped_path.header.stamp = odomAftMapped.header.stamp;
    m_laser_after_mapped_path.header.frame_id = "aft_mapped";
    m_laser_after_mapped_path.poses.push_back( laserAfterMappedPose );
    pub_odom_path.publish( m_laser_after_mapped_path ); 

    // Eigen::Vector3d eulerAngle1 = Q1.matrix().eulerAngles(2,1,0);
    Eigen::Matrix4f point_trans;
    geometry_msgs::PoseStamped CarPose;
    Eigen::Quaterniond Q2;
    nav_msgs::Path map_path,car_path1;
    map_path.header.stamp = car_path1.header.stamp = odomAftMapped.header.stamp;
    map_path.header.frame_id = car_path1.header.frame_id = "aft_mapped";
    if(all_pose[0].pose.position.x-result_trans(0, 3)<3.0&&all_pose[all_pose.size()-1].pose.position.x-result_trans(0, 3)>1.0)
    for(size_t i=0;i<all_pose.size();i++)
    {
        if(all_pose[i].pose.position.x-result_trans(0, 3)>-10.0&&all_pose[i].pose.position.x-result_trans(0, 3)<10.0&&
        all_pose[i].pose.position.y-result_trans(1, 3)>-10.0&&all_pose[i].pose.position.y-result_trans(1, 3)<10.0)
        {
            Q2=MapPathToRviz(R1,all_pose[i],point_trans);
            // std::cout<<"pose:   "<<CarPose<<std::endl;
            // CarPose.header.frame_id = "odom";
            // CarPose.header.stamp = ros::Time::now();
            // CarPose.header = odomAftMapped.header;
            CarPose.pose.position.x = point_trans(0, 3);
            CarPose.pose.position.y = point_trans(1, 3);
            CarPose.pose.position.z = point_trans(2, 3);
            // CarPose.pose.position.x = all_pose[i].pose.position.x-result_trans(0, 3);
            // CarPose.pose.position.y = all_pose[i].pose.position.y-result_trans(1, 3);
            // CarPose.pose.position.z = all_pose[i].pose.position.z-result_trans(2, 3);
            CarPose.pose.orientation.x = Q2.x();
            CarPose.pose.orientation.y = Q2.y();
            CarPose.pose.orientation.z = Q2.z();
            CarPose.pose.orientation.w = Q2.w();
            if(CarPose.pose.position.x>0&&CarPose.pose.position.x<10.0&&CarPose.pose.position.y>-5.0&&CarPose.pose.position.y<5.0)
            map_path.poses.push_back(CarPose);

            CarPose.pose.position.x = point_trans(1, 3);
            CarPose.pose.position.y = point_trans(0, 3);
            car_path1.poses.push_back(CarPose);
        }    
        if(all_pose[i].pose.position.x-result_trans(0, 3)>7.2||all_pose[i].pose.position.y-result_trans(1, 3)>7.2)
        break;

        // // 全局状态下
        // CarPose.pose.position.x = all_pose[i].pose.position.x;
        // CarPose.pose.position.y = all_pose[i].pose.position.y;
        // CarPose.pose.position.z = all_pose[i].pose.position.z;
        // CarPose.pose.orientation.x = all_pose[i].pose.orientation.x;
        // CarPose.pose.orientation.y = all_pose[i].pose.orientation.y;
        // CarPose.pose.orientation.z = all_pose[i].pose.orientation.z;
        // CarPose.pose.orientation.w = all_pose[i].pose.orientation.w;
        // map_path.poses.push_back(CarPose);

    }
    if(map_path.poses.size()>0&&map_path.poses[map_path.poses.size()-1].pose.position.x>2)
    icp_ok.data=true;
    else icp_ok.data=false;

    if(icp_ok.data)
    std::cout<<"using icp path"<<std::endl;
    else std::cout<<"don't using icp path"<<std::endl;

    if(!icp_ok.data) return;
    rviz_path.publish(car_path1);
    car_path.publish(map_path);//map_path

}

int frist=0,path_start=0,icp_error=0;
//进行icp配准
void Cloud_ICP( pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_source,  pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_target,  pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_source_registration)
{
    clock_t start = clock();
    // ICP配准
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	//PointCloud::Ptr cloud_source_registration(new PointCloud);
	//kdTree 加速搜索
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZ>);
	tree1->setInputCloud(cloud_source);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZ>);
	tree2->setInputCloud(cloud_target);
	icp.setSearchMethodSource(tree1);
	icp.setSearchMethodTarget(tree2);

	//设置参数
	icp.setInputSource(cloud_source);
	icp.setInputTarget(cloud_target);
	icp.setMaxCorrespondenceDistance(10);//当两个点云相距较远时候，距离值要变大，所以一开始需要粗配准。
	icp.setTransformationEpsilon(1e-6);//svd奇异值分解，对icp时间影响不大
	icp.setEuclideanFitnessEpsilon(1e-6);//前后两次误差大小，当误差值小于这个值停止迭代
	icp.setMaximumIterations(40);//最大迭代次数
	
    icp.setRANSACIterations(0);
    icp.align(*cloud_source_registration);
    std::cout << "score: " <<icp.getFitnessScore() << std::endl; 
    std::cout << "----------------------------------------------------------"<< std::endl;
    score=icp.getFitnessScore();

    if(icp_ok.data)
    std::cout<<"use icp path"<<std::endl;
    else std::cout<<"don't use icp path"<<std::endl;
    icp_is_ok.publish(icp_ok);
    if(score>1)
    icp_error++;
    if(path_start!=10)
    {
        path_start++;
        return;
    }
    else if(icp_error>3)
    {
            path_start=0;
            icp_error=0;
            return;
    }
    icp_error=0;

    Eigen::Matrix4f transformation = icp.getFinalTransformation();
    if(frist)
    {
        if(icp.getFitnessScore()<1)
        transformation_last=transformation*transformation_last;
        // GetKalman();
        // StartKalman();

    }
    else 
    {
        //if(icp.getFitnessScore()<30)
        transformation_last=transformation;
        frist=1;
    }
    StartKalman();

    
    //std::cout << transformation << std::endl;
	pcl::transformPointCloud(*cloud_source, *cloud_source_registration, transformation);

    //获取参数
    //Eigen::Vector3f ANGLE_result;
	//matrix2angle(transformation, ANGLE_result);

    // if(score>1)
    // icp_error++;
    // if(path_start!=10)
    // {
    //     path_start++;
    //     return;
    // }
    // else if(icp_error>3)
    // {
    //         path_start=0;
    //         icp_error=0;
    //         return;
    // }
    // icp_error=0;
    Pub_odom(transformation_last);

    cout << "\nicp time" << (double)(clock() - start) / CLOCKS_PER_SEC << endl;
}


//进行gicp配准
void Cloud_GICP( pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_source,  pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_target,  pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_source_registration)
{
    clock_t start = clock();
    // ICP配准
	pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	//kdTree 加速搜索
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree1(new pcl::search::KdTree<pcl::PointXYZ>);
	tree1->setInputCloud(cloud_source);
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZ>);
	tree2->setInputCloud(cloud_target);
	icp.setSearchMethodSource(tree1);
	icp.setSearchMethodTarget(tree2);

	//设置参数
	icp.setInputSource(cloud_source);
	icp.setInputTarget(cloud_target);
	icp.setMaxCorrespondenceDistance(10);//当两个点云相距较远时候，距离值要变大，所以一开始需要粗配准。
	icp.setTransformationEpsilon(1e-6);//svd奇异值分解，对icp时间影响不大
	icp.setEuclideanFitnessEpsilon(1);//前后两次误差大小，当误差值小于这个值停止迭代
	icp.setMaximumIterations(40);//最大迭代次数
	
    icp.setRANSACIterations(0);
    icp.align(*cloud_source_registration);
    std::cout << "score: " <<icp.getFitnessScore() << std::endl; 
    std::cout << "----------------------------------------------------------"<< std::endl;
    score=icp.getFitnessScore();
    Eigen::Matrix4f transformation = icp.getFinalTransformation();
    if(frist)
    {
        if(icp.getFitnessScore()<2)
        transformation_last=transformation*transformation_last;
        // GetKalman();
        // StartKalman();

    }
    else 
    {
        transformation_last=transformation;
        frist=1;
    }
	pcl::transformPointCloud(*cloud_source, *cloud_source_registration, transformation);

    Pub_odom(transformation_last);

    cout << "\nicp time" << (double)(clock() - start) / CLOCKS_PER_SEC << endl;
}

void pcl_ndt(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_source,  pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_target,  pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_source_registration)
{
    clock_t start = clock();
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ndt(new pcl::PointCloud<pcl::PointXYZ>);
    ndt.setTransformationEpsilon(0.01);
    ndt.setStepSize(3);
    ndt.setResolution(2.0);//网格分辨率
    ndt.setMaximumIterations(100);

    //载入点云
    ndt.setInputSource(cloud_source);
    ndt.setInputTarget(cloud_target);

            //设置初始变换矩阵，可有可无
    Eigen::AngleAxisf init_rotation(0.0, Eigen::Vector3f::UnitZ());//以z轴为坐标轴，旋转45°
    Eigen::Translation3f init_transtion(0.0, 0, 0);
    Eigen::Matrix4f init_guess = (init_transtion*init_rotation).matrix();

    ndt.align(*cloud_ndt, init_guess);

    Eigen::Matrix4f transformation = ndt.getFinalTransformation();
    std::cout << "score: " <<ndt.getFitnessScore() << std::endl; 
    std::cout << "----------------------------------------------------------"<< std::endl;
    if(frist)
    {
        if(ndt.getFitnessScore()<3)
        transformation_last=transformation*transformation_last;
        //GetKalman();

    }
    else 
    {
        //if(icp.getFitnessScore()<30)
        transformation_last=transformation;
        frist=1;
    }

    //std::cout << transformation << std::endl;
	pcl::transformPointCloud(*cloud_source, *cloud_source_registration, transformation);

    //获取参数
    //Eigen::Vector3f ANGLE_result;
	//matrix2angle(transformation, ANGLE_result);
    Pub_odom(transformation_last);

    cout << "\nndt time" << (double)(clock() - start) / CLOCKS_PER_SEC << endl;



}

//将地面分割抽取后再来进行匹配
void cloud_seg(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    //ransac分割算法
    //创建分割时所需要的模型系数对象，coefficients及存储内点的点索引集合对象inliers
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliners(new pcl::PointIndices);

        // 创建分割对象
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // 可选择配置，设置模型系数需要优化
    seg.setOptimizeCoefficients (true);
    // 必要的配置，设置分割的模型类型，所用的随机参数估计方法，距离阀值，输入点云
    seg.setModelType (pcl::SACMODEL_PLANE);   //设置模型类型
    seg.setMethodType (pcl::SAC_RANSAC);      //设置随机采样一致性方法类型
    seg.setDistanceThreshold (0.3);    //设定距离阀值，距离阀值决定了点被认为是局内点是必须满足的条件
                                        //表示点到估计模型的距离最大值，

    seg.setInputCloud (cloud);
      //引发分割实现，存储分割结果到点几何inliers及存储平面模型的系数coefficients
    seg.segment (*inliners, *coefficients);

    if (inliners->indices.size () == 0)
        {
            PCL_ERROR ("Could not estimate a planar model for the given dataset.");
            //return (-1);
        }
    pcl::ExtractIndices<pcl::PointXYZ> extract;      //点提取对象
      // 从点云中抽取分割的处在平面上的点集
    extract.setInputCloud (cloud);
    extract.setIndices (inliners);
    extract.setNegative (true);
    //pcl::PointCloud<pcl::PointXYZ> cloud_seg;
    extract.filter (*cloud);
}


//去除点云的离群点
void remove_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
{
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;  //创建滤波器
    
    outrem.setInputCloud(cloud);    //设置输入点云
    outrem.setRadiusSearch(1);     //设置半径为0.8的范围内找临近点
    outrem.setMinNeighborsInRadius (3); //设置查询点的邻域点集数小于2的删除
    // apply filter
    outrem.filter (*cloud);     //执行条件滤波   在半径为0.8 在此半径内必须要有两个邻居点，此点才会保存
}


//去除NAN点并进行下采样滤波
void NAN_VoxelGrid(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_source)
{
    std::cout << "down size *cloud_src_o from "<<cloud_source->size();
	pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
	voxel_grid.setLeafSize(0.4, 0.4, 0.4);
	voxel_grid.setInputCloud(cloud_source);
	voxel_grid.filter(*cloud_source);
	std::cout << "to" << cloud_source->size() << endl;
}



void club_icp(const sensor_msgs::PointCloud2ConstPtr & input)
{
     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input(new pcl::PointCloud<pcl::PointXYZ>); 
     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
     PointCloud::Ptr cloud_source_registration(new PointCloud);
     pcl::fromROSMsg (*input, *cloud_input);
     NAN_VoxelGrid(cloud_input);

    if(frist)
    {
        pcl::transformPointCloud(*cloud_input, *cloud_out, transformation_last);
        Cloud_ICP(cloud_out, cloud_tgt, cloud_source_registration);
    }
    else 
    {
        Cloud_ICP(cloud_input, cloud_tgt, cloud_source_registration);
    }
    visualize_cloud(cloud_out, cloud_tgt, cloud_source_registration);
     
}



int main(int argc, char** argv)
{

      //初始化
  //kalman.R={0.0015,2.0,300.0};
  kalman.R[0]=0.03;
  kalman.R[1]=0.03;
  kalman.R[2]=0.05;

  kalman.R[3]=0.002;
  kalman.R[4]=0.003;
  kalman.R[5]=0.005;
  kalman.R[6]=0.0001;
  //kalman.Q={0.0005,0.4,120.0};
  kalman.Q[0]=0.01;
  kalman.Q[1]=0.01;
  kalman.Q[2]=0.01;

  kalman.Q[3]=0.001;
  kalman.Q[4]=0.003;
  kalman.Q[5]=0.0015;
  kalman.Q[6]=0.0001;

  //kalman.X_last_last={0.0,0.0,600.0};
  kalman.X_last_last[0]=0.0;
  kalman.X_last_last[1]=0.0;
  kalman.X_last_last[2]=0.0;
  
  kalman.X_last_last[3]=0.0;
  kalman.X_last_last[4]=0.0;
  kalman.X_last_last[5]=0.0;
  kalman.X_last_last[6]=1.0;
  kalman.P_last_last[0]=0.8;
  kalman.P_last_last[1]=0.8;
  kalman.P_last_last[2]=0.8;

  kalman.P_last_last[3]=0.8;
  kalman.P_last_last[4]=0.8;
  kalman.P_last_last[5]=0.8;
  kalman.P_last_last[6]=0.8;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile("/home/cidi/Loam_livox_pcd/surround.pcd", *cloud_target);//目标点云
	std::cout << "target loaded!" << std::endl;
    std::cout << "down size *cloud_src_o from "<<cloud_target->size();
    
	pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
	voxel_grid.setLeafSize(0.4, 0.4, 0.4);
	voxel_grid.setInputCloud(cloud_target);
	//PointCloud::Ptr cloud_src(new PointCloud);
	voxel_grid.filter(*cloud_tgt);
	std::cout << "to" << cloud_tgt->size() << endl;

    std::string file;
    file = "/home/cidi/path/path.txt";
    GetPose(all_pose,file);
    ros::init(argc, argv,"ros_icp");

    ros::NodeHandle nh;//创建节点句柄
    ros::Subscriber sub = nh.subscribe("/pc2_gradient", 5, club_icp);  //pc2_gradient,pc2_corners
    pub = nh.advertise<sensor_msgs::PointCloud2>("output", 1);
    pub_odom_path = nh.advertise<nav_msgs::Path>("/odom_path",1);
    rviz_path = nh.advertise<nav_msgs::Path>("/path_1",1);
    car_path = nh.advertise<nav_msgs::Path>("/path",1);
    pub_odom =  nh.advertise<nav_msgs::Odometry>("/odom",1);
    debug_odom =  nh.advertise<nav_msgs::Odometry>("/debug_odom",1);
    icp_is_ok = nh.advertise<std_msgs::Bool>("/path_is_ok", 1000);
    ros::spin();
}
