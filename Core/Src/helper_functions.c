#include <math.h>
#include "helper_functions.h"

#include <math.h>
#include <stdio.h>

#define R 6371000 // Earth's radius in meters

static double deg2rad(double deg) {
  return deg * (M_PI / 180.0);
}

static double rad2deg(double rad) {
  return rad * (180.0 / M_PI);
}

double bearing(double lat1, double lon1, double lat2, double lon2) {
  double dLon = deg2rad(lon2 - lon1);
  double y = sin(dLon) * cos(deg2rad(lat2));
  double x = cos(deg2rad(lat1)) * sin(deg2rad(lat2)) - sin(deg2rad(lat1)) * cos(deg2rad(lat2)) * cos(dLon);
  double brng = atan2(y, x);
  return rad2deg(brng);
}

double distance(double lat1, double lon1, double lat2, double lon2) {
  double dLat = deg2rad(lat2 - lat1);
  double dLon = deg2rad(lon2 - lon1);
  double a = pow(sin(dLat / 2), 2) + cos(deg2rad(lat1)) * cos(deg2rad(lat2)) * pow(sin(dLon / 2), 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  double d = R * c;
  return d;
}
