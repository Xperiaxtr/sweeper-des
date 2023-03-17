#include "tracker/tracker.h"

#include <pcl/console/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>

#include "tracker/hungarian_bigraph_matcher.h"

namespace sweeper {
namespace navigation {

bool ObjectTracker::Init() {
  id_ = 0;
  first_gps = true;
  car_pose_ = Eigen::Matrix3f::Identity();
  Tracked_object_.clear();
  return true;
}

bool ObjectTracker::Track(
    const std::vector<std::shared_ptr<Object>> objects, double timestamp,
    std::vector<std::shared_ptr<Object>> *tracked_objects) {
  double new_time = 0.0;
  Current_Tracked_object_.clear();
  std::vector<tracker> trackers;
  Eigen::Matrix4d localcar_pose = Eigen::Matrix4d::Identity();
  if (objects.size() == 0) {
    return false;
  } else {
    for (int i = 0; i < objects.size(); ++i) {
      tracker tmp_track(objects[i]);
      tmp_track.kalman_init();
      //   tmp_track.num_points = objects[i]->cloud.points.size();
      tmp_track.num_points = objects[i]->contour.size();
      Eigen::Vector4d global_center(objects[i]->center[0],
                                    objects[i]->center[1],
                                    objects[i]->center[2], 1);

      global_center = localcar_pose * global_center;
      tmp_track.center[0] = global_center[0];
      tmp_track.center[1] = global_center[1];
      tmp_track.center[2] = global_center[2];
      tmp_track.local_center[0] = objects[i]->center[0];
      tmp_track.local_center[1] = objects[i]->center[1];
      tmp_track.local_center[2] = objects[i]->center[2];
      tmp_track.max_height = objects[i]->anchor_point[2];
      tmp_track.latest_tracked_time = timestamp;
      trackers.push_back(tmp_track);
    }
  }

  pcl::console::TicToc tt4;
  tt4.tic();
  if (Tracked_object_.size() == 0) {
    createNewTrackers(trackers);
  } else {
    new_time = timestamp;
    double diff_time = new_time - last_frame_time_;
    //预测跟踪物的位置
    predictOldTrackers(diff_time);
    std::vector<std::vector<double>> cost;
    //计算关联矩阵
    computeAssociateMatrix(Tracked_object_, trackers, cost);
    std::vector<int> tracks_idx;
    std::vector<int> objects_idx;
    //匈牙利算法匹配
    HungarianOptimizer hungarian_optimizer(cost);
    hungarian_optimizer.minimize(&tracks_idx, &objects_idx);
    std::vector<std::pair<int, int>> local_assignments;
    std::vector<int> local_unassigned_tracks;
    std::vector<int> local_unassigned_objects;
    local_assignments.resize(trackers.size());
    local_unassigned_tracks.assign(Tracked_object_.size(), -1);
    local_unassigned_objects.assign(trackers.size(), -1);
    AssignObjectsToTracks(tracks_idx, objects_idx, cost, &local_assignments,
                          &local_unassigned_tracks, &local_unassigned_objects);
    updateTrackers(trackers, local_assignments, local_unassigned_tracks,
                   local_unassigned_objects, timestamp);
  }
  int box_count = 0;
  visualization_msgs::MarkerArray markers;
  markers.markers.clear();
  for (int i = 0; i < Current_Tracked_object_.size(); ++i) {
    Current_Tracked_object_[i].object_ptr->track_id =
        Current_Tracked_object_[i].track_id;
    Current_Tracked_object_[i].object_ptr->velocity =
        Current_Tracked_object_[i].velocity;
    tracked_objects->push_back(Current_Tracked_object_[i].object_ptr);
  }
  last_frame_time_ = new_time;
  // std::cout << "the track code cost time is " << tt4.toc() << std::endl;
  // std::cout << "the size of tracks is " << Tracked_object_.size() <<
  // std::endl;
  return true;
}

void ObjectTracker::getObjects(std::vector<tracker> &trackers) {
  trackers = Current_Tracked_object_;
}

void ObjectTracker::updateTrackers(std::vector<tracker> trackers,
                                   std::vector<std::pair<int, int>> assignments,
                                   std::vector<int> unassigned_tracks,
                                   std::vector<int> unassigned_objects,
                                   double frame_time) {
  for (int i = 0; i < assignments.size(); ++i) {
    Tracked_object_[assignments[i].first].untracked_time = 0;
    Tracked_object_[assignments[i].first].center =
        trackers[assignments[i].second].center;
    Tracked_object_[assignments[i].first].local_center =
        trackers[assignments[i].second].local_center;
    Tracked_object_[assignments[i].first].vel_kalman(
        Tracked_object_[assignments[i].first].center[0],
        Tracked_object_[assignments[i].first].center[1], frame_time);
    Tracked_object_[assignments[i].first].num_points =
        trackers[assignments[i].second].num_points;
    Tracked_object_[assignments[i].first].object_ptr =
        trackers[assignments[i].second].object_ptr;
    Tracked_object_[assignments[i].first].latest_tracked_time =
        trackers[assignments[i].second].latest_tracked_time;
    Current_Tracked_object_.push_back(Tracked_object_[assignments[i].first]);
  }
  for (int i = 0; i < unassigned_tracks.size(); ++i) {
    Tracked_object_[unassigned_tracks[i]].untracked_time++;
    // Tracked_object_[unassigned_tracks[i]].velocity<<0,0,0;
  }
  size_t track_num = 0;
  //保留跟踪物的track_id，给后面新的tracker指定id
  std::vector<int> vec_track_id;
  for (int i = 0; i < Tracked_object_.size(); ++i) {
    if (Tracked_object_[i].untracked_time > 5) continue;
    if (i == track_num) {
      vec_track_id.push_back(Tracked_object_[i].track_id);
      track_num++;
    } else {
      tracker tmp_track = Tracked_object_[i];
      vec_track_id.push_back(Tracked_object_[i].track_id);
      Tracked_object_[track_num] = tmp_track;
      track_num++;
    }
  }
  //保留前track_num个元素
  // Tracked_object_.resize(track_num);
  std::vector<tracker> Server_Tracked_object;
  for (int i = 0; i < track_num; ++i) {
    Server_Tracked_object.push_back(Tracked_object_[i]);
  }
  for (int i = 0; i < unassigned_objects.size(); ++i) {
    int j = 0;
    std::vector<int>::iterator result;
    while (result != vec_track_id.end()) {
      result = find(vec_track_id.begin(), vec_track_id.end(), j);
      ++j;
    }
    vec_track_id.push_back(j - 1);
    // trackers[unassigned_objects[i]].track_id = j - 1;
    trackers[unassigned_objects[i]].track_id = id_++ % INT_MAX;
    Server_Tracked_object.push_back(trackers[unassigned_objects[i]]);
    Current_Tracked_object_.push_back(trackers[unassigned_objects[i]]);
  }
  Tracked_object_ = Server_Tracked_object;
}

void ObjectTracker::createNewTrackers(std::vector<tracker> trackers) {
  for (int i = 0; i < trackers.size(); ++i) {
    trackers[i].track_id = id_++ % INT_MAX;
    Tracked_object_.push_back(trackers[i]);
  }
}

void ObjectTracker::predictOldTrackers(double diff_time) {
  for (int i = 0; i < Tracked_object_.size(); ++i) {
    // std::cout<<"the velocity of i is
    // "<<Tracked_object_[i].velocity[0]<<","<<Tracked_object_[i].velocity[1]<<std::endl;
    Tracked_object_[i].center[0] += diff_time * Tracked_object_[i].velocity[0];
    Tracked_object_[i].center[1] += diff_time * Tracked_object_[i].velocity[1];
  }
}

void ObjectTracker::computeAssociateMatrix(
    const std::vector<tracker> &tracks, const std::vector<tracker> &new_objects,
    std::vector<std::vector<double>> &cost) {
  // Compute matrix of association distance
  Eigen::MatrixXf association_mat(tracks.size(), new_objects.size());
  int no_track = tracks.size();
  int no_object = new_objects.size();
  for (size_t i = 0; i < tracks.size(); ++i) {
    for (size_t j = 0; j < new_objects.size(); ++j) {
      float diff_points_num =
          fabs(new_objects[j].num_points - tracks[i].num_points);
      float num_cost = diff_points_num / tracks[i].num_points;

      int diff_area =
          abs(new_objects[j].object_ptr->area - tracks[i].object_ptr->area);
      float area_cost =
          (float)(diff_area) / (float)(tracks[i].object_ptr->area);

      /*
      float diff_height =
          fabs(new_objects[j].max_height -
      tracks[i].max_height); float height_cost = diff_height;
      */

      // float diff_position_y = fabs(tracks[i].center[1] -
      // new_objects[j].center[1])/fabs(tracks[i].center[1]);
      float diff_position_x = (tracks[i].center - new_objects[j].center).norm();

      float position_cost = diff_position_x;
      float sum_cost = position_cost * 0.5 + area_cost + num_cost;
      if (sum_cost > 3.0) sum_cost = 3.1;
      association_mat(i, j) = sum_cost;
    }
  }
  cost.resize(tracks.size() + new_objects.size());
  for (int i = 0; i < no_track; ++i) {
    cost[i].resize(association_mat.cols());
    for (int j = 0; j < association_mat.cols(); ++j) {
      cost[i][j] = association_mat(i, j);
    }
  }
  for (int i = 0; i < no_object; ++i) {
    cost[i + no_track].resize(association_mat.cols());
    for (int j = 0; j < association_mat.cols(); ++j) {
      if (j == i) {
        cost[i + no_track][j] = 999 * 1.2f;
      } else {
        cost[i + no_track][j] = 999999.0f;
      }
    }
  }
}

void ObjectTracker::AssignObjectsToTracks(
    std::vector<int> tracks_idx, std::vector<int> objects_idx,
    std::vector<std::vector<double>> cost_value,
    std::vector<std::pair<int, int>> *assignments,
    std::vector<int> *unassigned_tracks, std::vector<int> *unassigned_objects) {
  int assignments_num = 0;
  int no_track = cost_value.size() - cost_value[0].size();
  int no_object = cost_value[0].size();
  std::vector<bool> tracks_used(no_track + no_object, false);
  std::vector<bool> objects_used(no_object, false);
  for (size_t i = 0; i < tracks_idx.size(); ++i) {
    if (tracks_idx[i] < 0 || tracks_idx[i] >= no_track || objects_idx[i] < 0 ||
        objects_idx[i] >= no_object) {
      continue;
    }
    if (cost_value[tracks_idx[i]][objects_idx[i]] < 3.0) {
      (*assignments)[assignments_num++] =
          std::make_pair(tracks_idx[i], objects_idx[i]);
      tracks_used[tracks_idx[i]] = true;
      objects_used[objects_idx[i]] = true;
    }
  }
  assignments->resize(assignments_num);
  unassigned_tracks->resize(no_track);
  int unassigned_tracks_num = 0;
  for (int i = 0; i < no_track; ++i) {
    if (tracks_used[i] == false) {
      (*unassigned_tracks)[unassigned_tracks_num++] = i;
    }
  }
  unassigned_tracks->resize(unassigned_tracks_num);
  unassigned_objects->resize(no_object);
  int unassigned_objects_num = 0;
  for (int i = 0; i < no_object; ++i) {
    if (objects_used[i] == false) {
      (*unassigned_objects)[unassigned_objects_num++] = i;
    }
  }
  unassigned_objects->resize(unassigned_objects_num);
}

}  // namespace navigation
}  // namespace sweeper
