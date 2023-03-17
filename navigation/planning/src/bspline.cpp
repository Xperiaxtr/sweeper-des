#include "bspline.h"

namespace sweeper {
namespace navigation {

BSpline::BSpline() : _degree(3) {}
BSpline::~BSpline() {}

double BSpline::Divide(double x, double y) {
  if (y == 0.0) {
    return 0.0;
  }
  return x / y;
}

double BSpline::GetNiPu(double u, int i, int degree) {
  if (degree == 0) {
    if (u >= knots_u[i] && u < knots_u[i + 1]) {
      return 1.0;
    } else {
      return 0.0;
    }
  }
  double l0 = u - knots_u[i];  //系数1
  double l1 = knots_u[i + degree] - knots_u[i];
  double l2 = Divide(l0, l1);

  double k0 = knots_u[i + degree + 1] - u;
  double k1 = knots_u[i + degree + 1] - knots_u[i + 1];
  double k2 = Divide(k0, k1);

  double res =
      l2 * GetNiPu(u, i, degree - 1) + k2 * GetNiPu(u, i + 1, degree - 1);
  return res;
}

void BSpline::InitKnots(std::vector<cv::Point> cin) {
  for (int i = 0; i < cin.size(); i++) {
    cin_points.push_back(cv::Point2f(cin[i].x + 0.0001, cin[i].y + 0.0001));
  }
  int control = cin_points.size();

  for (int i = 0; i < _degree; i++) {
    knots_u.push_back(0.0);
  }

  double param = 0.0;

  for (int i = 0; i < control - _degree + 1; i++) {
    if (i > 0) param += 1.0;
    knots_u.push_back(param);
  }

  for (int i = 0; i < _degree; i++) {
    knots_u.push_back(param);
  }
}

std::vector<cv::Point2f> BSpline::CoutVector(int step,
                                             std::vector<cv::Point> cin) {
  // cv::Mat cin1 = cv::Mat(1000,1000,CV_8UC3, cv::Scalar(0, 0, 0));
  // for(size_t i=0;i<cin.size();i++)
  // {
  // circle(cin1, cin[i], 10, cv::Scalar(255, 0, 0),-1,8);
  // }

  std::vector<cv::Point2f> cout_points;
  if (cin.size() < 4) return cout_points;
  InitKnots(cin);
  int ilen = knots_u.size();
  double dstart = knots_u[0];
  double dend = knots_u[ilen - 1];
  double delta = (dend - dstart) / step;
  for (int s = 0; s < step; s++) {
    double u = s * delta;
    cv::Point2f v;

    for (int i = 0; i < cin_points.size(); i++) {
      double t = GetNiPu(u, i, _degree);
      v.x += cin_points[i].x * t;
      v.y += cin_points[i].y * t;
    }

    cout_points.push_back(v);
  }
  // for(size_t i=0;i<cout_points.size();i++)
  // {
  // circle(cin1, cout_points[i], 10, cv::Scalar(0, 0, 255),-1,8);
  // }
  // cv::imshow("cin",cin1);
  // cv::waitKey(1);
  knots_u.clear();
  cin_points.clear();
  return cout_points;
}
}  // namespace navigation
}  // namespace sweeper