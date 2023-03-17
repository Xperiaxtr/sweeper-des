#pragma once

#include <math.h>

#include <iostream>

using namespace std;

typedef struct _POSITION {
  double longitude;
  double latitude;
} GpsPosition;

class GpsConvert {
 public:
  GpsConvert();
  ~GpsConvert();
  GpsPosition bd09togcj02(double bd_lon, double bd_lat);
  GpsPosition gcj02tobd09(double gcj_lon, double gcj_lat);
  GpsPosition gcj02towgs84(double gcj_lon, double gcj_lat);
  GpsPosition wgs84togcj02(double wgs_lon, double wgs_lat);

 private:
  double translate_lon(double lon, double lat);
  double translate_lat(double lon, double lat);
  bool outof_China(double lon, double lat);

  GpsPosition bd_pos;
  GpsPosition gcj_pos;
  GpsPosition wgs_pos;

  double x_PI = 3.14159265358979323846 * 3000.0 / 180.0;
  double PI = 3.1415926535897932384626;
  double a = 6378245.0;
  double ee = 0.00669342162296594323;
};
