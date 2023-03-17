#pragma once

#include <cmath>
#include <opencv2/opencv.hpp>

namespace sweeper {
namespace navigation {

class BSpline {
 public:
  BSpline();
  ~BSpline();

  std::vector<cv::Point2f> CoutVector(int step, std::vector<cv::Point> cin);

 private:
  double Divide(double x, double y);
  double GetNiPu(double u, int i, int degree);
  void InitKnots(std::vector<cv::Point> cin);

  int knots;    //节点
  int _degree;  //次数
  std::vector<double> knots_u;
  std::vector<cv::Point2f> cin_points;
  // std::vector<cv::Point> cout_points;
};

}  // namespace navigation
}  // namespace sweeper
