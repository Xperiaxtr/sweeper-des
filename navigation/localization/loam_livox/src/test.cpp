#include "plane_line_icp.hpp"
#include "tools/common.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <ros/ros.h>

int main()
{
  // Plane_Line_ICP plicp;
  pcl::PointCloud<pcl::PointXYZI> cloud_tgt;
  pcl::io::loadPCDFile("../sweeper_ws/src/sweeper_haide/data/map/684648.pcd", cloud_tgt); //目标点云
  // plicp.GetCornerSurfMap(cloud_tgt);
  pcl::PointCloud<pcl::PointXYZI>::Ptr m_laser_cloud_corner_from_map(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr m_laser_cloud_surf_from_map(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr m_laser_cloud_intensity_from_map(new pcl::PointCloud<pcl::PointXYZI>());
  // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_surf( new pcl::PointCloud<pcl::PointXYZI>() );
  // pcl::PointCloud<PointType>::Ptr cloud_corner( new pcl::PointCloud<PointType>() );
  // *cloud_surf = *plicp.m_laser_cloud_surf_from_map;
  // *cloud_corner = *plicp.m_laser_cloud_corner_from_map;

  for (size_t i = 0; i < cloud_tgt.points.size(); i++)
  {
    // if(i<10000)
    // std::cout<< cloud_tgt.points[i].intensity<<std::endl;
    // if(cloud_tgt.points[i].intensity < 0.5)
    // cloud_surf->points.push_back(cloud_tgt.points[i]);
    //             PointType point_now = laserCloudMap.points[i];
    //       if (point_now.intensity < 1.0)
    pcl::PointXYZI point_now = cloud_tgt.points[i];
    if (point_now.intensity < 1.0)
      m_laser_cloud_surf_from_map->points.push_back(point_now);
    if (point_now.intensity > 3.0 && point_now.intensity < 10.0)
      m_laser_cloud_corner_from_map->points.push_back(point_now);
    if (point_now.intensity > 10.0)
    {
      m_laser_cloud_intensity_from_map->points.push_back(point_now);
    }
  }
  m_laser_cloud_surf_from_map->width = 1;
  m_laser_cloud_surf_from_map->height = m_laser_cloud_surf_from_map->points.size();
  // pcl::io::savePCDFileASCII("surf.pcd", *cloud_surf);
  std::cout<<"size : "<<m_laser_cloud_corner_from_map->points.size()<<std::endl;
  pcl::visualization::CloudViewer viewer("Cluster viewer");
  viewer.showCloud(m_laser_cloud_intensity_from_map);
  while (!viewer.wasStopped())
  {
  }
}