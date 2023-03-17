// #include "sensor_data/key_frame.hpp"

// namespace mapping {
// Eigen::Quaterniond KeyFrame::GetQuaternion() {
//   Eigen::Quaterniond q;
//   q = pose.block<3, 3>(0, 0);
//   Eigen::Quaterniond q_d((double)q.w(), (double)q.x(), (double)q.y(),
//                          (double)q.z());
//   return q_d;
// }

// void KeyFrame::AddQuaterPoseToKeyFrame(const Eigen::Quaterniond &quat, const Eigen::Vector3d &pos) {
//   // quater((double)quat.w(), (double)quat.x(), (double)quat.y(),
//   // (double)quat.z()); position((double)pos.x(), (double)pos.y(),
//   // (double)pos.z());
//   quater = quat;
//   position = pos;

//   pose.block(0, 0, 3, 3) = quater.matrix();
//   pose.block(0, 3, 3, 1) = position;
// }

// void KeyFrame::AddPoseToKeyFrame(const Eigen::Matrix4d &receive_pose) {
//   pose = receive_pose;
//   Eigen::Quaterniond q;
//   q = pose.block<3, 3>(0, 0);
//   quater = q;
//   position = Eigen::Vector3d(pose(0, 3), pose(1, 3), pose(2, 3));
// }
// }  // namespace mapping