#include "mapping/loop_closing/loop_closing.hpp"

namespace mapping {
LoopClosing::LoopClosing(int loop_history_search_num, double icp_loop_score,
                         double cloud_size, double map_size) {
  kdtree_history_key_poses_.reset(new pcl::KdTreeFLANN<CloudData::POINT>());
  history_frame_id_ = -1;
  loop_history_search_num_ = loop_history_search_num;  // 5
  cloud_filter_.setLeafSize(cloud_size, cloud_size, cloud_size);
  map_cloud_filter_.setLeafSize(map_size, map_size, map_size);
  icp_loop_score_ = icp_loop_score;  // 0.3
  last_sucess_loop_id_ = 0;
  save_pcd_times_ = 0;
  now_key_frame_id_ = 0;
}

void LoopClosing::ResetLoopParam() { last_sucess_loop_id_ = 0; }
int LoopClosing::PerformLoopClosure(const std::vector<KeyFrame> &key_frames,
                                    const BackData &back_data) {
  // loop_mutex_.lock();

  CloudData::CLOUD_PTR cloud_key_pose(new CloudData::CLOUD());
  for (int i = 0; i <= now_key_frame_id_; i++) {
    CloudData::POINT key_frame_point;
    key_frame_point.x = key_frames[i].pose(0, 3);
    key_frame_point.y = key_frames[i].pose(1, 3);
    key_frame_point.z = key_frames[i].pose(2, 3);
    cloud_key_pose->points.push_back(key_frame_point);
  }

  std::vector<int> point_search_ind_loop;
  std::vector<float> point_search_sqrt_dis_loop;
  history_frame_id_ = -1;
  double icp_loop_radius = 10.0;
  kdtree_history_key_poses_->setInputCloud(cloud_key_pose);
  CloudData::POINT last_key_pose;
  last_key_pose.x = back_data.odom_data(0, 3);
  last_key_pose.y = back_data.odom_data(1, 3);
  last_key_pose.z = back_data.odom_data(2, 3);
  kdtree_history_key_poses_->radiusSearch(last_key_pose, icp_loop_radius,
                                          point_search_ind_loop,
                                          point_search_sqrt_dis_loop, 0);
  AWARN << "loop search size : " << point_search_ind_loop.size();
  double max_time = 0.0, max_id = -1;
  for (size_t i = point_search_ind_loop.size() - 1; i > 0; i--) {
    int id = point_search_ind_loop[i];
    double time = key_frames[now_key_frame_id_].time - key_frames[id].time;
    if (max_time < time) {
      max_id = id;
      max_time = time;
    }
     if (time > 30.0) {
       AWARN << "time : " << key_frames[id].time << "   "
             << key_frames[now_key_frame_id_].time;
       AWARN << "pefore loop ok! the time is : " << time << " id is " << id;
       history_frame_id_ = id;
       break;
    }
  }
  //if (max_time > 30.0) {
    // AWARN << "time : " << key_frames[id].time << "   "
    //       << key_frames[now_key_frame_id_].time;
    // AWARN << "pefore loop ok! the time is : " << time << " id is " << id;
    //history_frame_id_ = max_id;
    // break;
  //}
  // loop_mutex_.unlock();
  return history_frame_id_;
}

void LoopClosing::DetectionLoopClosure(const std::vector<KeyFrame> &key_frames,
                                       const BackData &back_data,
                                       std::vector<int> &loop_points_,
                                       std::vector<Eigen::Matrix4d> &loop_trans,
                                       BackEnd &back_end) {
  loop_mutex_.lock();
  std::vector<KeyFrame> copy_key_frames(key_frames);
  loop_mutex_.unlock();

  if (copy_key_frames.size() < 20) return;
  AWARN << "Start DetectionLoopClosure";
  now_key_frame_id_ = back_data.index;
  // last_key_pose_.x = back_data.odom_data(0, 3);
  // last_key_pose_.y = back_data.odom_data(1, 3);
  // last_key_pose_.z = back_data.odom_data(2, 3);

  AWARN << "key frame num : " << now_key_frame_id_;
  // loop_mutex_.lock();
  // while (now_key_frame_id > 1) {
  //   if (now_key_frame_id /* && copy_key_frames[now_key_frame_id].optimized*/)
  //   {
  //     now_key_frame_id_ = now_key_frame_id;
  //     break;
  //   }
  //   now_key_frame_id--;
  //   if (now_key_frame_id <= last_sucess_loop_id_) break;
  // }
  // loop_mutex_.unlock();
  if (now_key_frame_id_ < 20 || now_key_frame_id_ - last_sucess_loop_id_ < 2)
    return;

  AWARN << "now key frame num : " << now_key_frame_id_
        << " last keyframe : " << last_sucess_loop_id_;

  int history_id = PerformLoopClosure(copy_key_frames, back_data);
  if (history_id != -1 && now_key_frame_id_ - history_id > 15) {
    AWARN << "PerformLoopClosure is true!";
    AWARN << "history id is : " << history_id;
    //构建局部地图
    CloudData::CLOUD_PTR history_clouds(new CloudData::CLOUD());
    CloudData::CLOUD_PTR history_clouds_ds(new CloudData::CLOUD());
    CloudData::CLOUD_PTR now_clouds(new CloudData::CLOUD());
    CloudData::CLOUD_PTR now_clouds_ds(new CloudData::CLOUD());
    std::string key_frame_path =
        "../sweeper_ws/src/sweeper_haide/data/map/slam_data/key_frames/";

    // loop_mutex_.lock();

    for (int i = -loop_history_search_num_; i < loop_history_search_num_; i++) {
      int key_frame_id = i + history_id;
      if (key_frame_id < 0 || key_frame_id > now_key_frame_id_) continue;
      CloudData::CLOUD history_cloud;
      CloudData::CLOUD now_cloud;
      pcl::io::loadPCDFile(
          key_frame_path + std::to_string(key_frame_id) + "_cloud.pcd",
          now_cloud);
      pcl::transformPointCloud(now_cloud, history_cloud,
                               copy_key_frames[key_frame_id].pose);
      *history_clouds += history_cloud;
    }

    for (int i = -loop_history_search_num_ / 4; i <= 0; i++) {
      int key_frame_id = i + now_key_frame_id_;
      if (key_frame_id < 0 || key_frame_id > now_key_frame_id_) continue;
      CloudData::CLOUD history_cloud;
      CloudData::CLOUD now_cloud;
      pcl::io::loadPCDFile(
          key_frame_path + std::to_string(key_frame_id) + "_cloud.pcd",
          now_cloud);
      pcl::transformPointCloud(now_cloud, history_cloud,
                               copy_key_frames[key_frame_id].pose);
      *now_clouds += history_cloud;
    }

    // for (int i = 0; i <= 0; i++) {
    //   int key_frame_id = i + now_key_frame_id_;
    //   if (key_frame_id < 0 || key_frame_id > now_key_frame_id_) continue;
    //   CloudData::CLOUD history_cloud;
    //   CloudData::CLOUD now_cloud;
    //   pcl::io::loadPCDFile(
    //       key_frame_path + std::to_string(now_key_frame_id_) + "_cloud.pcd",
    //       now_cloud);
    //   pcl::transformPointCloud(now_cloud, history_cloud,
    //                            copy_key_frames[key_frame_id].pose);
    //   *now_clouds += history_cloud;
    // }

    // loop_mutex_.unlock();
    cloud_filter_.setInputCloud(now_clouds);
    cloud_filter_.filter(*now_clouds_ds);
    map_cloud_filter_.setInputCloud(history_clouds);
    map_cloud_filter_.filter(*history_clouds_ds);

    AWARN << "loop cloud size: " << now_clouds_ds->points.size();
    if (now_clouds_ds->points.size() < 200) return;

    // std::string save_corner_name = "corner";
    // std::string save_map_name = "corner_map";

    // std::string save_corner_name =
    //     "../corner_cloud" + std::to_string(save_pcd_times_) + ".pcd";
    // std::string save_corner_map_name =
    //     "../corner_map" + std::to_string(save_pcd_times_) + ".pcd";
    // pcl::io::savePCDFileASCII(save_corner_name.c_str(), *now_clouds_ds);
    // pcl::io::savePCDFileASCII(save_corner_map_name.c_str(),
    // *history_clouds_ds);

    // save_pcd_times_++;
    AWARN << "Start icp matcher";

    AWARN << "now cloud_ds_size: " << now_clouds_ds->points.size()
          << " last size: " << history_clouds_ds->points.size();
    //进行icp匹配
    pcl::IterativeClosestPoint<CloudData::POINT, CloudData::POINT> icp;
    icp.setMaxCorrespondenceDistance(100);
    icp.setMaximumIterations(100);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);
    icp.setRANSACIterations(0);
    icp.setInputSource(now_clouds_ds);
    icp.setInputTarget(history_clouds_ds);
    CloudData::CLOUD_PTR unused_result(new CloudData::CLOUD());
    icp.align(*unused_result);
    double matcher_score = icp.getFitnessScore();
    AERROR << "icp getFitnessScore" << matcher_score;
    if (icp.hasConverged() == false || matcher_score > icp_loop_score_) return;

    // std::string save_corner_name =
    //     "../corner_cloud" + std::to_string(save_pcd_times_) + ".pcd";
    // std::string save_corner_map_name =
    //     "../corner_map" + std::to_string(save_pcd_times_) + ".pcd";
    // pcl::io::savePCDFileASCII(save_corner_name.c_str(), *now_clouds_ds);
    // pcl::io::savePCDFileASCII(save_corner_map_name.c_str(),
    // *history_clouds_ds); save_pcd_times_++;

    AWARN << "last position : " << copy_key_frames[history_id].pose;
    AWARN << "now position : " << back_data.odom_data;
    //得到变换位姿与编号
    Eigen::Matrix4f icp_trans = icp.getFinalTransformation();
    AWARN << "icp trans : " << icp_trans(0, 3) << " " << icp_trans(1, 3) << " "
          << icp_trans(2, 3);

    //如果误差太大
    if (fabs(icp_trans(0, 3)) > 10.0 || fabs(icp_trans(1, 3)) > 10.0 ||
        fabs(icp_trans(2, 3)) > 10.0)
      return;
    //当前点云到过去的变换
    Eigen::Isometry3d tran_now_to_last =
        MathCalculation::Matrix4dToIsometry3d(icp_trans.cast<double>());

    //当前的位姿
    loop_mutex_.lock();
    Eigen::Isometry3d tran_now =
        MathCalculation::Matrix4dToIsometry3d(back_data.odom_data);
    //过去的姿态
    Eigen::Isometry3d tran_last = MathCalculation::Matrix4dToIsometry3d(
        copy_key_frames[history_id].pose.cast<double>());
    loop_mutex_.unlock();
    //当前帧的实际位姿
    Eigen::Isometry3d now_key_frame_pose = tran_now_to_last * tran_now;

    AERROR << "tran now position : " << now_key_frame_pose(0, 3) << " "
           << now_key_frame_pose(1, 3) << " " << now_key_frame_pose(2, 3);
    //当前到过去帧的变换
    Eigen::Isometry3d now_to_last_tran =
        tran_last.inverse() * now_key_frame_pose;

    AERROR << "delta position : " << now_to_last_tran(0, 3) << " "
           << now_to_last_tran(1, 3) << " " << now_to_last_tran(2, 3);

    //将数据添加到后端中
    back_end.AddLoopToG2O(now_to_last_tran, history_id, now_key_frame_id_,
                          matcher_score);

    loop_points_.push_back(history_id);
    loop_points_.push_back(now_key_frame_id_);
    loop_trans.push_back(
        MathCalculation::Isometry3dToMatrix4d(now_to_last_tran));
    last_sucess_loop_id_ = now_key_frame_id_;
    // loop_mutex_.unlock();
  } else {
    AWARN << "not loop";
  }
}
}  // namespace mapping