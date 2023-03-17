#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <fstream>
#include <vector>

#include "log.h"

// using double type to define x, y, z.

namespace sweeper {
namespace common {

typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZI>::Ptr PointCloudPtr;
typedef PointCloud PolygonDType;

class PolygonUtil {
 public:
  template <typename PointT>
  static bool IsXyPointIn2dXyPolygon(const PointT &point,
                                     const PolygonDType &polygon) {
    bool in_poly = false;
    double x1, x2, y1, y2;
    int nr_poly_points = static_cast<int>(polygon.points.size());
    // start with the last point to make the check last point<->first point
    // the　first one
    double xold = polygon.points[nr_poly_points - 1].x;
    double yold = polygon.points[nr_poly_points - 1].y;
    for (int i = 0; i < nr_poly_points; i++) {
      double xnew = polygon.points[i].x;
      double ynew = polygon.points[i].y;
      if (xnew > xold) {
        x1 = xold;
        x2 = xnew;
        y1 = yold;
        y2 = ynew;
      } else {
        x1 = xnew;
        x2 = xold;
        y1 = ynew;
        y2 = yold;
      }
      if ((x1 < point.x) == (point.x <= x2) &&
          (point.y - y1) * (x2 - x1) < (y2 - y1) * (point.x - x1)) {
        in_poly = !in_poly;
      }
      xold = xnew;
      yold = ynew;
    }
    return in_poly;

    // int sum = 0;
    // double x1, y1, x2, y2, tmp_y;
    // double x = point.x;
    // double y = point.y;

    // int nr_poly_points = static_cast<int>(polygon.points.size());
    // if (nr_poly_points < 3) return false;

    // for (int i = 0; i < nr_poly_points; i++) {
    //   if (i == (nr_poly_points - 1)) {
    //     x1 = polygon.points[i].x;
    //     y1 = polygon.points[i].y;
    //     x2 = polygon.points[0].x;
    //     y2 = polygon.points[0].y;
    //   } else {
    //     x1 = polygon.points[i].x;
    //     y1 = polygon.points[i].y;
    //     x2 = polygon.points[i + 1].x;
    //     y2 = polygon.points[i + 1].y;
    //   }

    //   if ((x >= x1 && x < x2) || (x >= x2 && x < x1)) {
    //     if (fabs(x1 - x2) > 0.000001) {
    //       tmp_y = y1 - (x1 - x) * ((y2 - y1) / (x2 - x1));
    //       if (tmp_y < y) sum++;
    //     }
    //   }
    // }
    // //　点射线与多边形相交点为奇数个则在多边形内部
    // if (sum % 2 != 0) return true;

    // return false;
  }

  template <typename PointT>
  static bool IsXyPointInHdmap(const PointT &p,
                               const std::vector<PolygonDType> &polygons) {
    bool in_flag = false;
    for (std::size_t j = 0; j < polygons.size(); j++) {
      if (IsXyPointIn2dXyPolygon<PointT>(p, polygons[j])) {
        in_flag = true;
        break;
      }
    }
    return in_flag;
  }

  static void MockPolygon(const Eigen::Vector3d &center, const double length,
                          const double width, const double theta,
                          PolygonDType *polygon);
};

}  // namespace common
}  // namespace sweeper
