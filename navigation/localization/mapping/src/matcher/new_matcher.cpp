#include "new_matcher.hpp"

NewMatcher::NewMatcher(int icp_max_iterations) {
  laserCloudOri.reset(new pcl::PointCloud<pcl::PointXYZI>());
  coeffSel.reset(new pcl::PointCloud<pcl::PointXYZI>());
  laser_cloud_corner_map_.reset(new pcl::PointCloud<pcl::PointXYZI>());
  laser_cloud_surf_map_.reset(new pcl::PointCloud<pcl::PointXYZI>());
  laser_cloud_intensity_map_.reset(new pcl::PointCloud<pcl::PointXYZI>());


  kdtree_corner_from_map_ = pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr(
      new pcl::KdTreeFLANN<pcl::PointXYZI>());
  kdtree_surf_from_map_ = pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr(
      new pcl::KdTreeFLANN<pcl::PointXYZI>());
  kdtree_intensity_from_map_ = pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr(
      new pcl::KdTreeFLANN<pcl::PointXYZI>());
  m_q_w_curr_ = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
  m_t_w_curr_ = Eigen::Vector3d(0.0, 0.0, 0.0);
  icp_max_iterations_ = icp_max_iterations;
}

double NewMatcher::rad2deg(double radians) { return radians * 180.0 / M_PI; }

void NewMatcher::PointAssociateToMap(pcl::PointXYZI const *const pi,
                                     pcl::PointXYZI *const po) {
  // rot z（transformTobeMapped[2]）
  float x1 =
      cos(transformTobeMapped[2]) * pi->x - sin(transformTobeMapped[2]) * pi->y;
  float y1 =
      sin(transformTobeMapped[2]) * pi->x + cos(transformTobeMapped[2]) * pi->y;
  float z1 = pi->z;

  // rot x（transformTobeMapped[0]）
  float x2 = x1;
  float y2 =
      cos(transformTobeMapped[0]) * y1 - sin(transformTobeMapped[0]) * z1;
  float z2 =
      sin(transformTobeMapped[0]) * y1 + cos(transformTobeMapped[0]) * z1;

  // rot y（transformTobeMapped[1]）then add trans
  po->x = cos(transformTobeMapped[1]) * x2 + sin(transformTobeMapped[1]) * z2 +
          transformTobeMapped[3];
  po->y = y2 + transformTobeMapped[4];
  po->z = -sin(transformTobeMapped[1]) * x2 + cos(transformTobeMapped[1]) * z2 +
          transformTobeMapped[5];
  po->intensity = pi->intensity;
}


void NewMatcher::CopyMapToMacther(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &laser_cloud_corner_map,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &laser_cloud_surf_map,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &laser_cloud_intensity_map) {
  *laser_cloud_corner_map_ = *laser_cloud_corner_map;
  *laser_cloud_surf_map_ = *laser_cloud_surf_map;
  *laser_cloud_intensity_map_ = *laser_cloud_intensity_map;
  kdtree_corner_from_map_->setInputCloud(laser_cloud_corner_map_);
  kdtree_surf_from_map_->setInputCloud(laser_cloud_surf_map_);
  kdtree_intensity_from_map_->setInputCloud(laser_cloud_intensity_map_);
}

double NewMatcher::CloudToMapMacth(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &laser_cloud_corner,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &laser_cloud_surf,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &laser_cloud_intensity) {
  int laser_corner_point_num = laser_cloud_corner->points.size();
  int laser_surf_point_num = laser_cloud_surf->points.size();
  int laser_intensity_point_num = laser_cloud_intensity->points.size();

  // kdtree_corner_from_map_->setInputCloud(laser_cloud_corner_map);
  // kdtree_surf_from_map_->setInputCloud(laser_cloud_surf_map);
  // kdtree_intensity_from_map_->setInputCloud(laser_cloud_intensity_map);

  // laserCloudOri->clear();
  // coeffSel->clear();
  pcl::PointXYZI point_origin;
  pcl::PointXYZI point_select, coeff;
  std::vector<int> point_search_idx;
  std::vector<float> point_search_dis;

  cv::Mat matA0(10, 3, CV_32F, cv::Scalar::all(0));
  cv::Mat matB0(10, 1, CV_32F, cv::Scalar::all(-1));
  cv::Mat matX0(10, 1, CV_32F, cv::Scalar::all(0));

  cv::Mat matA1(3, 3, CV_32F, cv::Scalar::all(0));
  cv::Mat matD1(1, 3, CV_32F, cv::Scalar::all(0));
  cv::Mat matV1(3, 3, CV_32F, cv::Scalar::all(0));

  bool isDegenerate = false;
  cv::Mat matP(6, 6, CV_32F, cv::Scalar::all(0));

  for (int matcher_time = 0; matcher_time < icp_max_iterations_;
       matcher_time++) {
    float average_distance = 0.0;
    laserCloudOri->clear();
    coeffSel->clear();
    for (int i = 0; i < laser_corner_point_num; i++) {
      point_origin = laser_cloud_corner->points[i];
      PointAssociateToMap(&point_origin, &point_select);
      kdtree_corner_from_map_->nearestKSearch(point_select, 5, point_search_idx,
                                              point_search_dis);

      if (point_search_dis[4] < 1.5) {
        float cx = 0;
        float cy = 0;
        float cz = 0;
        for (int j = 0; j < 5; j++) {
          cx += laser_cloud_corner_map_->points[point_search_idx[j]].x;
          cy += laser_cloud_corner_map_->points[point_search_idx[j]].y;
          cz += laser_cloud_corner_map_->points[point_search_idx[j]].z;
        }
        cx /= 5;
        cy /= 5;
        cz /= 5;
        // mean square error
        float a11 = 0;
        float a12 = 0;
        float a13 = 0;
        float a22 = 0;
        float a23 = 0;
        float a33 = 0;
        for (int j = 0; j < 5; j++) {
          float ax =
              laser_cloud_corner_map_->points[point_search_idx[j]].x - cx;
          float ay =
              laser_cloud_corner_map_->points[point_search_idx[j]].y - cy;
          float az =
              laser_cloud_corner_map_->points[point_search_idx[j]].z - cz;

          a11 += ax * ax;
          a12 += ax * ay;
          a13 += ax * az;
          a22 += ay * ay;
          a23 += ay * az;
          a33 += az * az;
        }
        a11 /= 5;
        a12 /= 5;
        a13 /= 5;
        a22 /= 5;
        a23 /= 5;
        a33 /= 5;

        matA1.at<float>(0, 0) = a11;
        matA1.at<float>(0, 1) = a12;
        matA1.at<float>(0, 2) = a13;
        matA1.at<float>(1, 0) = a12;
        matA1.at<float>(1, 1) = a22;
        matA1.at<float>(1, 2) = a23;
        matA1.at<float>(2, 0) = a13;
        matA1.at<float>(2, 1) = a23;
        matA1.at<float>(2, 2) = a33;

        cv::eigen(matA1, matD1, matV1);

        if (matD1.at<float>(0, 0) > 3 * matD1.at<float>(0, 1)) {
          float x0 = point_select.x;
          float y0 = point_select.y;
          float z0 = point_select.z;
          float x1 = cx + 0.1 * matV1.at<float>(0, 0);
          float y1 = cy + 0.1 * matV1.at<float>(0, 1);
          float z1 = cz + 0.1 * matV1.at<float>(0, 2);
          float x2 = cx - 0.1 * matV1.at<float>(0, 0);
          float y2 = cy - 0.1 * matV1.at<float>(0, 1);
          float z2 = cz - 0.1 * matV1.at<float>(0, 2);

          // OA = (x0 - x1, y0 - y1, z0 - z1),OB = (x0 - x2, y0 - y2, z0 -
          // z2)，AB = （x1 - x2, y1 - y2, z1 - z2） cross: |  i      j k |
          //|x0-x1  y0-y1  z0-z1|
          //|x0-x2  y0-y2  z0-z2|
          float a012 =
              sqrt(((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) *
                       ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) +
                   ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) *
                       ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) +
                   ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1)) *
                       ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1)));

          float l12 = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) +
                           (z1 - z2) * (z1 - z2));

          float la =
              ((y1 - y2) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) +
               (z1 - z2) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1))) /
              a012 / l12;

          float lb =
              -((x1 - x2) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) -
                (z1 - z2) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1))) /
              a012 / l12;

          float lc =
              -((x1 - x2) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) +
                (y1 - y2) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1))) /
              a012 / l12;

          float ld2 = a012 / l12;
          // if(fabs(ld2) > 1) continue;

          float s = 1 - 0.9 * fabs(ld2);
          // float s = 1.0;// - 0.9 * fabs(ld2);

          coeff.x = s * la;
          coeff.y = s * lb;
          coeff.z = s * lc;
          coeff.intensity = s * ld2;

          average_distance += ld2;

          if (s > 0.1) {
            laserCloudOri->push_back(point_origin);
            coeffSel->push_back(coeff);
          }
        }
      }
    }

    for (int i = 0; i < laser_intensity_point_num; i++) {
      point_origin = laser_cloud_intensity->points[i];
      PointAssociateToMap(&point_origin, &point_select);
      kdtree_intensity_from_map_->nearestKSearch(
          point_select, 5, point_search_idx, point_search_dis);

      if (point_search_dis[4] < 1.5) {
        float cx = 0;
        float cy = 0;
        float cz = 0;
        for (int j = 0; j < 5; j++) {
          cx += laser_cloud_intensity_map_->points[point_search_idx[j]].x;
          cy += laser_cloud_intensity_map_->points[point_search_idx[j]].y;
          cz += laser_cloud_intensity_map_->points[point_search_idx[j]].z;
        }
        cx /= 5;
        cy /= 5;
        cz /= 5;
        // mean square error
        float a11 = 0;
        float a12 = 0;
        float a13 = 0;
        float a22 = 0;
        float a23 = 0;
        float a33 = 0;
        for (int j = 0; j < 5; j++) {
          float ax =
              laser_cloud_intensity_map_->points[point_search_idx[j]].x - cx;
          float ay =
              laser_cloud_intensity_map_->points[point_search_idx[j]].y - cy;
          float az =
              laser_cloud_intensity_map_->points[point_search_idx[j]].z - cz;

          a11 += ax * ax;
          a12 += ax * ay;
          a13 += ax * az;
          a22 += ay * ay;
          a23 += ay * az;
          a33 += az * az;
        }
        a11 /= 5;
        a12 /= 5;
        a13 /= 5;
        a22 /= 5;
        a23 /= 5;
        a33 /= 5;

        matA1.at<float>(0, 0) = a11;
        matA1.at<float>(0, 1) = a12;
        matA1.at<float>(0, 2) = a13;
        matA1.at<float>(1, 0) = a12;
        matA1.at<float>(1, 1) = a22;
        matA1.at<float>(1, 2) = a23;
        matA1.at<float>(2, 0) = a13;
        matA1.at<float>(2, 1) = a23;
        matA1.at<float>(2, 2) = a33;

        cv::eigen(matA1, matD1, matV1);

        if (matD1.at<float>(0, 0) > 3 * matD1.at<float>(0, 1)) {
          float x0 = point_select.x;
          float y0 = point_select.y;
          float z0 = point_select.z;
          float x1 = cx + 0.1 * matV1.at<float>(0, 0);
          float y1 = cy + 0.1 * matV1.at<float>(0, 1);
          float z1 = cz + 0.1 * matV1.at<float>(0, 2);
          float x2 = cx - 0.1 * matV1.at<float>(0, 0);
          float y2 = cy - 0.1 * matV1.at<float>(0, 1);
          float z2 = cz - 0.1 * matV1.at<float>(0, 2);

          // OA = (x0 - x1, y0 - y1, z0 - z1),OB = (x0 - x2, y0 - y2, z0 -
          // z2)，AB = （x1 - x2, y1 - y2, z1 - z2） cross: |  i      j k |
          //|x0-x1  y0-y1  z0-z1|
          //|x0-x2  y0-y2  z0-z2|
          float a012 =
              sqrt(((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) *
                       ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) +
                   ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) *
                       ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) +
                   ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1)) *
                       ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1)));

          float l12 = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) +
                           (z1 - z2) * (z1 - z2));

          float la =
              ((y1 - y2) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) +
               (z1 - z2) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1))) /
              a012 / l12;

          float lb =
              -((x1 - x2) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) -
                (z1 - z2) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1))) /
              a012 / l12;

          float lc =
              -((x1 - x2) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) +
                (y1 - y2) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1))) /
              a012 / l12;

          float ld2 = a012 / l12;
          // if(fabs(ld2) > 1) continue;

          float s = 1 - 0.9 * fabs(ld2);
          // float s = 1.0;

          coeff.x = s * la;
          coeff.y = s * lb;
          coeff.z = s * lc;
          coeff.intensity = s * ld2;

              average_distance += ld2;

          if (s > 0.1) {
            laserCloudOri->push_back(point_origin);
            coeffSel->push_back(coeff);
          }
        }
      }
    }

    for (int i = 0; i < laser_surf_point_num; i++) {
      point_origin = laser_cloud_surf->points[i];

      PointAssociateToMap(&point_origin, &point_select);
      kdtree_surf_from_map_->nearestKSearch(point_select, 8, point_search_idx,
                                            point_search_dis);

      if (point_search_dis[7] < 5.0) {
        for (int j = 0; j < 8; j++) {
          matA0.at<float>(j, 0) =
              laser_cloud_surf_map_->points[point_search_idx[j]].x;
          matA0.at<float>(j, 1) =
              laser_cloud_surf_map_->points[point_search_idx[j]].y;
          matA0.at<float>(j, 2) =
              laser_cloud_surf_map_->points[point_search_idx[j]].z;
        }
        // matA0*matX0=matB0
        // AX+BY+CZ+D = 0 <=> AX+BY+CZ=-D <=> (A/D)X+(B/D)Y+(C/D)Z = -1
        //(X,Y,Z)<=>mat_a0
        // A/D, B/D, C/D <=> mat_x0

        cv::solve(matA0, matB0, matX0, cv::DECOMP_QR);  // TODO

        float pa = matX0.at<float>(0, 0);
        float pb = matX0.at<float>(1, 0);
        float pc = matX0.at<float>(2, 0);
        float pd = 1;

        // ps is the norm of the plane normal vector
        // pd is the distance from point to plane
        float ps = sqrt(pa * pa + pb * pb + pc * pc);
        pa /= ps;
        pb /= ps;
        pc /= ps;
        pd /= ps;

        bool planeValid = true;
        for (int j = 0; j < 8; j++) {
          if (fabs(pa * laser_cloud_surf_map_->points[point_search_idx[j]].x +
                   pb * laser_cloud_surf_map_->points[point_search_idx[j]].y +
                   pc * laser_cloud_surf_map_->points[point_search_idx[j]].z +
                   pd) > 0.2) {
            planeValid = false;
            break;
          }
        }

        if (planeValid) {
          // loss fuction
          float pd2 = pa * point_select.x + pb * point_select.y +
                      pc * point_select.z + pd;

          // if(fabs(pd2) > 0.1) continue;

          float s = 1 - 0.9 * fabs(pd2) /
                            sqrt(sqrt(point_select.x * point_select.x +
                                      point_select.y * point_select.y +
                                      point_select.z * point_select.z));
          // float s = 1.0;

          coeff.x = s * pa;
          coeff.y = s * pb;
          coeff.z = s * pc;
          coeff.intensity = s * pd2;

          average_distance += pd2;

          if (s > 0.1) {
            laserCloudOri->push_back(point_origin);
            coeffSel->push_back(coeff);
          }
        }
      }
    }

    float srx = sin(transformTobeMapped[0]);
    float crx = cos(transformTobeMapped[0]);
    float sry = sin(transformTobeMapped[1]);
    float cry = cos(transformTobeMapped[1]);
    float srz = sin(transformTobeMapped[2]);
    float crz = cos(transformTobeMapped[2]);

    average_distance /= laserCloudOri->points.size();


    std::cout<<"average dis : "<<average_distance<<std::endl;
    pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_ori(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr coeff_sel(new pcl::PointCloud<pcl::PointXYZI>());
    for(size_t i = 0; i < laserCloudOri->points.size(); i++)
    {
      // if(coeffSel->points[i].intensity < 20 * average_distance && coeffSel->points[i].intensity > 0.05 * average_distance)
      // {
        laser_cloud_ori->points.push_back(laserCloudOri->points[i]);
        coeff_sel->points.push_back(coeffSel->points[i]);
      // }
    }



    int laserCloudSelNum = laser_cloud_ori->points.size();
    if (laserCloudSelNum < 50) {
      continue;
    }

    //|c1c3+s1s2s3 c3s1s2-c1s3 c2s1|
    //|   c2s3        c2c3      -s2|
    //|c1s2s3-c3s1 c1c3s2+s1s3 c1c2|
    // AT*A*x = AT*b
    cv::Mat matA(laserCloudSelNum, 6, CV_32F, cv::Scalar::all(0));
    cv::Mat matAt(6, laserCloudSelNum, CV_32F, cv::Scalar::all(0));
    cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
    cv::Mat matB(laserCloudSelNum, 1, CV_32F, cv::Scalar::all(0));
    cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
    cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));
    float debug_distance = 0;
    for (int i = 0; i < laserCloudSelNum; i++) {
      point_origin = laser_cloud_ori->points[i];
      coeff = coeff_sel->points[i];

      float arx =
          (crx * sry * srz * point_origin.x + crx * crz * sry * point_origin.y -
           srx * sry * point_origin.z) *
              coeff.x +
          (-srx * srz * point_origin.x - crz * srx * point_origin.y -
           crx * point_origin.z) *
              coeff.y +
          (crx * cry * srz * point_origin.x + crx * cry * crz * point_origin.y -
           cry * srx * point_origin.z) *
              coeff.z;

      float ary = ((cry * srx * srz - crz * sry) * point_origin.x +
                   (sry * srz + cry * crz * srx) * point_origin.y +
                   crx * cry * point_origin.z) *
                      coeff.x +
                  ((-cry * crz - srx * sry * srz) * point_origin.x +
                   (cry * srz - crz * srx * sry) * point_origin.y -
                   crx * sry * point_origin.z) *
                      coeff.z;

      float arz =
          ((crz * srx * sry - cry * srz) * point_origin.x +
           (-cry * crz - srx * sry * srz) * point_origin.y) *
              coeff.x +
          (crx * crz * point_origin.x - crx * srz * point_origin.y) * coeff.y +
          ((sry * srz + cry * crz * srx) * point_origin.x +
           (crz * sry - cry * srx * srz) * point_origin.y) *
              coeff.z;

      matA.at<float>(i, 0) = arx;
      matA.at<float>(i, 1) = ary;
      matA.at<float>(i, 2) = arz;
      // TODO: the partial derivative
      matA.at<float>(i, 3) = coeff.x;
      matA.at<float>(i, 4) = coeff.y;
      matA.at<float>(i, 5) = coeff.z;
      matB.at<float>(i, 0) = -coeff.intensity;

      debug_distance += fabs(coeff.intensity);
    }
    cv::transpose(matA, matAt);
    matAtA = matAt * matA;
    matAtB = matAt * matB;
    cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

    // Deterioration judgment
    if (matcher_time == 0) {
      cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0));
      cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0));
      cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));

      cv::eigen(matAtA, matE, matV);
      matV.copyTo(matV2);

      isDegenerate = false;
      float eignThre[6] = {1, 1, 1, 1, 1, 1};
      for (int i = 5; i >= 0; i--) {
        if (matE.at<float>(0, i) < eignThre[i]) {
          for (int j = 0; j < 6; j++) {
            matV2.at<float>(i, j) = 0;
          }
          isDegenerate = true;
        } else {
          break;
        }
      }
      matP = matV.inv() * matV2;
    }

    if (isDegenerate) {
      cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
      matX.copyTo(matX2);
      matX = matP * matX2;
    }

    transformTobeMapped[0] += matX.at<float>(0, 0);
    transformTobeMapped[1] += matX.at<float>(1, 0);
    transformTobeMapped[2] += matX.at<float>(2, 0);
    transformTobeMapped[3] += matX.at<float>(3, 0);
    transformTobeMapped[4] += matX.at<float>(4, 0);
    transformTobeMapped[5] += matX.at<float>(5, 0);

    float deltaR = sqrt(pow(rad2deg(matX.at<float>(0, 0)), 2) +
                        pow(rad2deg(matX.at<float>(1, 0)), 2) +
                        pow(rad2deg(matX.at<float>(2, 0)), 2));
    float deltaT = sqrt(pow(matX.at<float>(3, 0) * 100, 2) +
                        pow(matX.at<float>(4, 0) * 100, 2) +
                        pow(matX.at<float>(5, 0) * 100, 2));

    if (deltaR < 0.05 && deltaT < 0.05) {
      break;
    }
  }

  m_q_w_curr_ =
      Eigen::AngleAxisd(transformTobeMapped[2], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(transformTobeMapped[1], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(transformTobeMapped[0], Eigen::Vector3d::UnitX());

  m_t_w_curr_ = Eigen::Vector3d(transformTobeMapped[3], transformTobeMapped[4],
                                transformTobeMapped[5]);


  //计算误差
    double fitness_score = 0.0;
    double nr = 0.0;
    for (int i = 0; i < laser_corner_point_num; i++) {
      point_origin = laser_cloud_corner->points[i];
      if(point_origin.x > 20.0) continue;
      PointAssociateToMap(&point_origin, &point_select);
      kdtree_corner_from_map_->nearestKSearch(point_select, 1, point_search_idx,
                                              point_search_dis);
      if (point_search_dis[0] < 15.0) {
        fitness_score += point_search_dis[0];
        nr += 1.0;
        }
    }
    
    for (int i = 0; i < laser_intensity_point_num; i++) {
      point_origin = laser_cloud_intensity->points[i];
      if(point_origin.x > 20.0) continue;
      PointAssociateToMap(&point_origin, &point_select);
      kdtree_intensity_from_map_->nearestKSearch(
          point_select, 1, point_search_idx, point_search_dis);
      if (point_search_dis[0] < 15.0) {
        fitness_score += point_search_dis[0];
        nr += 1.0;
      }
    }

    for (int i = 0; i < laser_surf_point_num; i++) {
      point_origin = laser_cloud_surf->points[i];
      if(point_origin.x > 20.0) continue;
      PointAssociateToMap(&point_origin, &point_select);
      kdtree_surf_from_map_->nearestKSearch(point_select, 1, point_search_idx,
                                            point_search_dis);
      if (point_search_dis[0] < 15.0) {
        fitness_score += point_search_dis[0];
        nr += 1.0;
      }
    }

    if (nr > 0)
      return (fitness_score / nr);
    else
      return 1000.0;
}







double NewMatcher::GetCloudToMapMacth(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &laser_cloud_corner,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &laser_cloud_surf,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &laser_cloud_intensity,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &laser_cloud_corner_map,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &laser_cloud_surf_map,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr &laser_cloud_intensity_map) {
      std::cout<<"619"<<std::endl;
  int laser_corner_point_num = laser_cloud_corner->points.size();
  int laser_surf_point_num = laser_cloud_surf->points.size();
  int laser_intensity_point_num = laser_cloud_intensity->points.size();

  kdtree_corner_from_map_->setInputCloud(laser_cloud_corner_map);
  kdtree_surf_from_map_->setInputCloud(laser_cloud_surf_map);
  kdtree_intensity_from_map_->setInputCloud(laser_cloud_intensity_map);
std::cout<<"627"<<std::endl;
  // laserCloudOri->clear();
  // coeffSel->clear();
  pcl::PointXYZI point_origin;
  pcl::PointXYZI point_select, coeff;
  std::vector<int> point_search_idx;
  std::vector<float> point_search_dis;

  cv::Mat matA0(5, 3, CV_32F, cv::Scalar::all(0));
  cv::Mat matB0(5, 1, CV_32F, cv::Scalar::all(-1));
  cv::Mat matX0(3, 1, CV_32F, cv::Scalar::all(0));

  cv::Mat matA1(3, 3, CV_32F, cv::Scalar::all(0));
  cv::Mat matD1(1, 3, CV_32F, cv::Scalar::all(0));
  cv::Mat matV1(3, 3, CV_32F, cv::Scalar::all(0));

  bool isDegenerate = false;
  cv::Mat matP(6, 6, CV_32F, cv::Scalar::all(0));
std::cout<<"645               "<<icp_max_iterations_<<std::endl;
  for (int matcher_time = 0; matcher_time < icp_max_iterations_;
       matcher_time++) {
    float average_distance = 0.0;
    laserCloudOri->clear();
    coeffSel->clear();
    for (int i = 0; i < laser_corner_point_num; i++) {
      point_origin = laser_cloud_corner->points[i];
      PointAssociateToMap(&point_origin, &point_select);
      kdtree_corner_from_map_->nearestKSearch(point_select, 5, point_search_idx,
                                              point_search_dis);
      if (point_search_dis[4] < 1.5) {
        float cx = 0;
        float cy = 0;
        float cz = 0;
        for (int j = 0; j < 5; j++) {
          cx += laser_cloud_corner_map->points[point_search_idx[j]].x;
          cy += laser_cloud_corner_map->points[point_search_idx[j]].y;
          cz += laser_cloud_corner_map->points[point_search_idx[j]].z;
        }
        cx /= 5.0;
        cy /= 5.0;
        cz /= 5.0;
        // mean square error
        float a11 = 0;
        float a12 = 0;
        float a13 = 0;
        float a22 = 0;
        float a23 = 0;
        float a33 = 0;
        for (int j = 0; j < 5; j++) {
          float ax =
              laser_cloud_corner_map->points[point_search_idx[j]].x - cx;
          float ay =
              laser_cloud_corner_map->points[point_search_idx[j]].y - cy;
          float az =
              laser_cloud_corner_map->points[point_search_idx[j]].z - cz;

          a11 += ax * ax;
          a12 += ax * ay;
          a13 += ax * az;
          a22 += ay * ay;
          a23 += ay * az;
          a33 += az * az;
        }
        a11 /= 5.0;
        a12 /= 5.0;
        a13 /= 5.0;
        a22 /= 5.0;
        a23 /= 5.0;
        a33 /= 5.0;

        matA1.at<float>(0, 0) = a11;
        matA1.at<float>(0, 1) = a12;
        matA1.at<float>(0, 2) = a13;
        matA1.at<float>(1, 0) = a12;
        matA1.at<float>(1, 1) = a22;
        matA1.at<float>(1, 2) = a23;
        matA1.at<float>(2, 0) = a13;
        matA1.at<float>(2, 1) = a23;
        matA1.at<float>(2, 2) = a33;

        cv::eigen(matA1, matD1, matV1);

        if (matD1.at<float>(0, 0) > 3 * matD1.at<float>(0, 1)) {
          float x0 = point_select.x;
          float y0 = point_select.y;
          float z0 = point_select.z;
          // float x1 = cx + 0.1 * matV1.at<float>(0, 0);
          // float y1 = cy + 0.1 * matV1.at<float>(0, 1);
          // float z1 = cz + 0.1 * matV1.at<float>(0, 2);
          // float x2 = cx - 0.1 * matV1.at<float>(0, 0);
          // float y2 = cy - 0.1 * matV1.at<float>(0, 1);
          // float z2 = cz - 0.1 * matV1.at<float>(0, 2);
          float x1 = laser_cloud_corner_map->points[point_search_idx[0]].x;
          float y1 = laser_cloud_corner_map->points[point_search_idx[0]].y;
          float z1 = laser_cloud_corner_map->points[point_search_idx[0]].z;

          float x2 = laser_cloud_corner_map->points[point_search_idx[1]].x;
          float y2 = laser_cloud_corner_map->points[point_search_idx[1]].x;
          float z2 = laser_cloud_corner_map->points[point_search_idx[1]].x;

          // OA = (x0 - x1, y0 - y1, z0 - z1),OB = (x0 - x2, y0 - y2, z0 -
          // z2)，AB = （x1 - x2, y1 - y2, z1 - z2） cross: |  i      j k |
          //|x0-x1  y0-y1  z0-z1|
          //|x0-x2  y0-y2  z0-z2|
          float a012 =
              sqrt(((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) *
                       ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) +
                   ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) *
                       ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) +
                   ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1)) *
                       ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1)));

          float l12 = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) +
                           (z1 - z2) * (z1 - z2));

          float la =
              ((y1 - y2) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) +
               (z1 - z2) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1))) /
              a012 / l12;

          float lb =
              -((x1 - x2) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) -
                (z1 - z2) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1))) /
              a012 / l12;

          float lc =
              -((x1 - x2) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) +
                (y1 - y2) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1))) /
              a012 / l12;

          float ld2 = a012 / l12;
          // if(fabs(ld2) > 1) continue;

          float s = 1 - 0.9 * fabs(ld2);
          // float s = 1.0;// - 0.9 * fabs(ld2);
          // if(s < 0.1)continue;
          // s = 1.0;

          coeff.x = s * la;
          coeff.y = s * lb;
          coeff.z = s * lc;
          coeff.intensity = s * ld2;

          average_distance += ld2;

          if (s > 0.1) {
            laserCloudOri->push_back(point_origin);
            coeffSel->push_back(coeff);
          }
        }
      }
    }

// std::cout<<"intensity: "<<laser_intensity_point_num<<std::endl;
//     for (int i = 0; i < laser_intensity_point_num; i++) {
//       point_origin = laser_cloud_intensity->points[i];
//       PointAssociateToMap(&point_origin, &point_select);
//       kdtree_intensity_from_map_->nearestKSearch(
//           point_select, 5, point_search_idx, point_search_dis);

//       if (point_search_dis[4] < 1.5) {
//         float cx = 0;
//         float cy = 0;
//         float cz = 0;
//         for (int j = 0; j < 5; j++) {
//           cx += laser_cloud_intensity_map->points[point_search_idx[j]].x;
//           cy += laser_cloud_intensity_map->points[point_search_idx[j]].y;
//           cz += laser_cloud_intensity_map->points[point_search_idx[j]].z;
//         }
//         cx /= 5.0;
//         cy /= 5.0;
//         cz /= 5.0;
//         // mean square error
//         float a11 = 0;
//         float a12 = 0;
//         float a13 = 0;
//         float a22 = 0;
//         float a23 = 0;
//         float a33 = 0;
//         for (int j = 0; j < 5; j++) {
//           float ax =
//               laser_cloud_intensity_map->points[point_search_idx[j]].x - cx;
//           float ay =
//               laser_cloud_intensity_map->points[point_search_idx[j]].y - cy;
//           float az =
//               laser_cloud_intensity_map->points[point_search_idx[j]].z - cz;

//           a11 += ax * ax;
//           a12 += ax * ay;
//           a13 += ax * az;
//           a22 += ay * ay;
//           a23 += ay * az;
//           a33 += az * az;
//         }
//         a11 /= 5.0;
//         a12 /= 5.0;
//         a13 /= 5.0;
//         a22 /= 5.0;
//         a23 /= 5.0;
//         a33 /= 5.0;

//         matA1.at<float>(0, 0) = a11;
//         matA1.at<float>(0, 1) = a12;
//         matA1.at<float>(0, 2) = a13;
//         matA1.at<float>(1, 0) = a12;
//         matA1.at<float>(1, 1) = a22;
//         matA1.at<float>(1, 2) = a23;
//         matA1.at<float>(2, 0) = a13;
//         matA1.at<float>(2, 1) = a23;
//         matA1.at<float>(2, 2) = a33;

//         cv::eigen(matA1, matD1, matV1);

//         if (matD1.at<float>(0, 0) > 3 * matD1.at<float>(0, 1)) {
//           float x0 = point_select.x;
//           float y0 = point_select.y;
//           float z0 = point_select.z;
//           float x1 = cx + 0.1 * matV1.at<float>(0, 0);
//           float y1 = cy + 0.1 * matV1.at<float>(0, 1);
//           float z1 = cz + 0.1 * matV1.at<float>(0, 2);
//           float x2 = cx - 0.1 * matV1.at<float>(0, 0);
//           float y2 = cy - 0.1 * matV1.at<float>(0, 1);
//           float z2 = cz - 0.1 * matV1.at<float>(0, 2);

//           // OA = (x0 - x1, y0 - y1, z0 - z1),OB = (x0 - x2, y0 - y2, z0 -
//           // z2)，AB = （x1 - x2, y1 - y2, z1 - z2） cross: |  i      j k |
//           //|x0-x1  y0-y1  z0-z1|
//           //|x0-x2  y0-y2  z0-z2|
//           float a012 =
//               sqrt(((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) *
//                        ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) +
//                    ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) *
//                        ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) +
//                    ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1)) *
//                        ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1)));

//           float l12 = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) +
//                            (z1 - z2) * (z1 - z2));

//           float la =
//               ((y1 - y2) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) +
//                (z1 - z2) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1))) /
//               a012 / l12;

//           float lb =
//               -((x1 - x2) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) -
//                 (z1 - z2) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1))) /
//               a012 / l12;

//           float lc =
//               -((x1 - x2) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) +
//                 (y1 - y2) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1))) /
//               a012 / l12;

//           float ld2 = a012 / l12;
//           // if(fabs(ld2) > 1) continue;

//           float s = 1 - 0.9 * fabs(ld2);
//           if(s < 0.1)continue;
//           s = 1.0;

//           coeff.x = s * la;
//           coeff.y = s * lb;
//           coeff.z = s * lc;
//           coeff.intensity = s * ld2;

//               average_distance += ld2;

//           if (s > 0.1) {
//             laserCloudOri->push_back(point_origin);
//             coeffSel->push_back(coeff);
//           }
//         }
//       }
//     }

    std::cout<<"intensity: "<<laser_surf_point_num<<std::endl;

    std::cout<<"corner size: "<<laserCloudOri->points.size()<<std::endl;
    for (int i = 0; i < laser_surf_point_num; i++) {
      point_origin = laser_cloud_surf->points[i];

      PointAssociateToMap(&point_origin, &point_select);
      kdtree_surf_from_map_->nearestKSearch(point_select, 5, point_search_idx,
                                            point_search_dis);

      if (point_search_dis[4] < 3.0) {
      //          bool plane_is_avail = true;
      //   std::vector<Eigen::Vector3d> near_corners;
      //   Eigen::Vector3d center(0, 0, 0);
      // for (int j = 0; j < 5; j++) {
      //     Eigen::Vector3d tmp(
      //         laser_cloud_surf_map_->points[point_search_idx[j]].x,
      //         laser_cloud_surf_map_->points[point_search_idx[j]].y,
      //         laser_cloud_surf_map_->points[point_search_idx[j]].z);
      //     center = center + tmp;
      //     near_corners.push_back(tmp);
      //   }

      //   center = center / (float)(5);
      //   Eigen::Matrix3d covMat = Eigen::Matrix3d::Zero();

      //   for (int j = 0; j < 5; j++) {
      //     Eigen::Matrix<double, 3, 1> tmpZeroMean = near_corners[j] - center;
      //     covMat = covMat + tmpZeroMean * tmpZeroMean.transpose();
      //   }
      //   Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> saes(covMat);

      //   if ((saes.eigenvalues()[2] > 3 * saes.eigenvalues()[0]) &&
      //       (saes.eigenvalues()[2] < 10 * saes.eigenvalues()[1])) {
      //     plane_is_avail = true;
      //   } else {
      //     plane_is_avail = false;
      //   }


        for (int j = 0; j < 5; j++) {
          matA0.at<float>(j, 0) =
              laser_cloud_surf_map->points[point_search_idx[j]].x;
          matA0.at<float>(j, 1) =
              laser_cloud_surf_map->points[point_search_idx[j]].y;
          matA0.at<float>(j, 2) =
              laser_cloud_surf_map->points[point_search_idx[j]].z;
        }
        // matA0*matX0=matB0
        // AX+BY+CZ+D = 0 <=> AX+BY+CZ=-D <=> (A/D)X+(B/D)Y+(C/D)Z = -1
        //(X,Y,Z)<=>mat_a0
        // A/D, B/D, C/D <=> mat_x0

        cv::solve(matA0, matB0, matX0, cv::DECOMP_QR);  // TODO

        float pa = matX0.at<float>(0, 0);
        float pb = matX0.at<float>(1, 0);
        float pc = matX0.at<float>(2, 0);
        float pd = 1;

        // ps is the norm of the plane normal vector
        // pd is the distance from point to plane
        float ps = sqrt(pa * pa + pb * pb + pc * pc);
        pa /= ps;
        pb /= ps;
        pc /= ps;
        pd /= ps;

        bool planeValid = true;
        for (int j = 0; j < 5; j++) {
          if (fabs(pa * laser_cloud_surf_map->points[point_search_idx[j]].x +
                   pb * laser_cloud_surf_map->points[point_search_idx[j]].y +
                   pc * laser_cloud_surf_map->points[point_search_idx[j]].z +
                   pd) > 0.2) {
            planeValid = false;
            break;
          }
        }




        if (planeValid) {
          // loss fuction
          float pd2 = pa * point_select.x + pb * point_select.y +
                      pc * point_select.z + pd;

          if(fabs(pd2) > 0.1) continue;

          float s = 1 - 0.9 * fabs(pd2) /
                            sqrt(sqrt(point_select.x * point_select.x +
                                      point_select.y * point_select.y +
                                      point_select.z * point_select.z));
          // float s = 1.0;
          // if(s < 0.1)continue;
          // s = 1.0;

          coeff.x = s * pa;
          coeff.y = s * pb;
          coeff.z = s * pc;
          coeff.intensity = s * pd2;

          average_distance += pd2;

          if (s > 0.1) {
            laserCloudOri->push_back(point_origin);
            coeffSel->push_back(coeff);
          }
        }
      }
    }
    std::cout<<"corner+surf size: "<<laserCloudOri->points.size()<<std::endl;

    float srx = sin(transformTobeMapped[0]);
    float crx = cos(transformTobeMapped[0]);
    float sry = sin(transformTobeMapped[1]);
    float cry = cos(transformTobeMapped[1]);
    float srz = sin(transformTobeMapped[2]);
    float crz = cos(transformTobeMapped[2]);

    average_distance /= laserCloudOri->points.size();


    std::cout<<"average dis : "<<average_distance<<std::endl;
    // pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_ori(new pcl::PointCloud<pcl::PointXYZI>());
    // pcl::PointCloud<pcl::PointXYZI>::Ptr coeff_sel(new pcl::PointCloud<pcl::PointXYZI>());
    // for(size_t i = 0; i < laserCloudOri->points.size(); i++)
    // {
    //   // if(coeffSel->points[i].intensity < 20 * average_distance && coeffSel->points[i].intensity > 0.05 * average_distance)
    //   // {
    //     laser_cloud_ori->points.push_back(laserCloudOri->points[i]);
    //     coeff_sel->points.push_back(coeffSel->points[i]);
    //   // }
    // }



    int laserCloudSelNum = laserCloudOri->points.size();
    if (laserCloudSelNum < 50) {
      continue;
    }

    //|c1c3+s1s2s3 c3s1s2-c1s3 c2s1|
    //|   c2s3        c2c3      -s2|
    //|c1s2s3-c3s1 c1c3s2+s1s3 c1c2|
    // AT*A*x = AT*b
    cv::Mat matA(laserCloudSelNum, 6, CV_32F, cv::Scalar::all(0));
    cv::Mat matAt(6, laserCloudSelNum, CV_32F, cv::Scalar::all(0));
    cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
    cv::Mat matB(laserCloudSelNum, 1, CV_32F, cv::Scalar::all(0));
    cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
    cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));
    float debug_distance = 0;
    for (int i = 0; i < laserCloudSelNum; i++) {
      point_origin = laserCloudOri->points[i];
      coeff = coeffSel->points[i];

      float arx =
          (crx * sry * srz * point_origin.x + crx * crz * sry * point_origin.y -
           srx * sry * point_origin.z) *
              coeff.x +
          (-srx * srz * point_origin.x - crz * srx * point_origin.y -
           crx * point_origin.z) *
              coeff.y +
          (crx * cry * srz * point_origin.x + crx * cry * crz * point_origin.y -
           cry * srx * point_origin.z) *
              coeff.z;

      float ary = ((cry * srx * srz - crz * sry) * point_origin.x +
                   (sry * srz + cry * crz * srx) * point_origin.y +
                   crx * cry * point_origin.z) *
                      coeff.x +
                  ((-cry * crz - srx * sry * srz) * point_origin.x +
                   (cry * srz - crz * srx * sry) * point_origin.y -
                   crx * sry * point_origin.z) *
                      coeff.z;

      float arz =
          ((crz * srx * sry - cry * srz) * point_origin.x +
           (-cry * crz - srx * sry * srz) * point_origin.y) *
              coeff.x +
          (crx * crz * point_origin.x - crx * srz * point_origin.y) * coeff.y +
          ((sry * srz + cry * crz * srx) * point_origin.x +
           (crz * sry - cry * srx * srz) * point_origin.y) *
              coeff.z;

      matA.at<float>(i, 0) = arx;
      matA.at<float>(i, 1) = ary;
      matA.at<float>(i, 2) = arz;
      // TODO: the partial derivative
      matA.at<float>(i, 3) = coeff.x;
      matA.at<float>(i, 4) = coeff.y;
      matA.at<float>(i, 5) = coeff.z;
      matB.at<float>(i, 0) = -coeff.intensity;

      debug_distance += fabs(coeff.intensity);
    }
    cv::transpose(matA, matAt);
    matAtA = matAt * matA;
    matAtB = matAt * matB;
    cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

    // Deterioration judgment
    if (matcher_time == 0) {
      cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0));
      cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0));
      cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));

      cv::eigen(matAtA, matE, matV);
      matV.copyTo(matV2);

      // isDegenerate = false;
      // float eignThre[6] = {0.1, 0.1, 0.1, 0.1, 0.1, 0.05};
      // for (int i = 5; i >= 0; i--) {
      //   if (matE.at<float>(0, i) < eignThre[i]) {
      //     for (int j = 0; j < 6; j++) {
      //       matV2.at<float>(i, j) = 0;
      //     }
      //     isDegenerate = true;
      //   } else {
      //     break;
      //   }
      // }
      isDegenerate = true;
      matP = matV.inv() * matV2;
    }

    if (isDegenerate) {
      cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
      matX.copyTo(matX2);
      matX = matP * matX2;
    }

    transformTobeMapped[0] += matX.at<float>(0, 0);
    transformTobeMapped[1] += matX.at<float>(1, 0);
    transformTobeMapped[2] += matX.at<float>(2, 0);
    transformTobeMapped[3] += matX.at<float>(3, 0);
    transformTobeMapped[4] += matX.at<float>(4, 0);
    transformTobeMapped[5] += matX.at<float>(5, 0);

    float deltaR = sqrt(pow(rad2deg(matX.at<float>(0, 0)), 2) +
                        pow(rad2deg(matX.at<float>(1, 0)), 2) +
                        pow(rad2deg(matX.at<float>(2, 0)), 2));
    float deltaT = sqrt(pow(matX.at<float>(3, 0) * 100, 2) +
                        pow(matX.at<float>(4, 0) * 100, 2) +
                        pow(matX.at<float>(5, 0) * 100, 2));

    if (deltaR < 0.05 && deltaT < 0.05) {
      break;
    }
  }
  // laserCloudOri->clear();
  std::cout<<"size: 1109: "<<laserCloudOri->points.size()<<"  "<<coeffSel->points.size()<<std::endl;

  m_q_w_curr_ =
      Eigen::AngleAxisd(transformTobeMapped[2], Eigen::Vector3d::UnitZ()) *
      Eigen::AngleAxisd(transformTobeMapped[1], Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(transformTobeMapped[0], Eigen::Vector3d::UnitX());

  m_t_w_curr_ = Eigen::Vector3d(transformTobeMapped[3], transformTobeMapped[4],
                                transformTobeMapped[5]);


  //计算误差
    double fitness_score = 0.0;
    double nr = 0.0;
    for (int i = 0; i < laser_corner_point_num; i++) {
      point_origin = laser_cloud_corner->points[i];
      if(point_origin.x > 20.0) continue;
      PointAssociateToMap(&point_origin, &point_select);
      kdtree_corner_from_map_->nearestKSearch(point_select, 1, point_search_idx,
                                              point_search_dis);
      if (point_search_dis[0] < 15.0) {
        fitness_score += point_search_dis[0];
        nr += 1.0;
        }
    }
    
    for (int i = 0; i < laser_intensity_point_num; i++) {
      point_origin = laser_cloud_intensity->points[i];
      if(point_origin.x > 20.0) continue;
      PointAssociateToMap(&point_origin, &point_select);
      kdtree_intensity_from_map_->nearestKSearch(
          point_select, 1, point_search_idx, point_search_dis);
      if (point_search_dis[0] < 15.0) {
        fitness_score += point_search_dis[0];
        nr += 1.0;
      }
    }

    for (int i = 0; i < laser_surf_point_num; i++) {
      point_origin = laser_cloud_surf->points[i];
      if(point_origin.x > 20.0) continue;
      PointAssociateToMap(&point_origin, &point_select);
      kdtree_surf_from_map_->nearestKSearch(point_select, 1, point_search_idx,
                                            point_search_dis);
      if (point_search_dis[0] < 15.0) {
        fitness_score += point_search_dis[0];
        nr += 1.0;
      }
    }

    if (nr > 0)
      return (fitness_score / nr);
    else
      return 1000.0;
}

void NewMatcher::PredictMatcherPose(const Eigen::Quaterniond &predict_quater,
                                    const Eigen::Vector3d &predict_position) {
  //   m_t_w_curr_ = predict_position;
  //   m_q_w_curr_ = predict_quater;

  tf::Quaternion tf_quater(predict_quater.x(), predict_quater.y(),
                           predict_quater.z(), predict_quater.w());
  double roll, pitch, yaw;
  tf::Matrix3x3(tf_quater).getRPY(roll, pitch, yaw);
  transformTobeMapped[0] = roll;
  transformTobeMapped[1] = pitch;
  transformTobeMapped[2] = yaw;

  transformTobeMapped[3] = predict_position.x();
  transformTobeMapped[4] = predict_position.y();
  transformTobeMapped[5] = predict_position.z();

  std::cout<<"predict quater: "<<predict_quater.x()<<" "<<predict_quater.y()<<" "<<predict_quater.z()<<" "<<predict_quater.w()<<std::endl;
  std::cout<<" predict : "<<transformTobeMapped[0]<<" "<<transformTobeMapped[1]<<" "<<transformTobeMapped[2]<<" "<<transformTobeMapped[3]<<" "<<transformTobeMapped[4]<<" "<<transformTobeMapped[5]<<std::endl;
}

NewMatcher::~NewMatcher() {}
