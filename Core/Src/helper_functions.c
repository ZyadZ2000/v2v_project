#include <math.h>
#include "helper_functions.h"

#define R 6371000.0 // Radius of the earth in meters

double haversine(double lat1, double lon1, double lat2, double lon2) {
    double dlat = (lat2 - lat1) * 0.01745329252; // Convert degrees to radians
    double dlon = (lon2 - lon1) * 0.01745329252;
    double a = pow(sin(dlat / 2.0), 2.0) + cos(lat1 * 0.01745329252) * cos(lat2 * 0.01745329252) * pow(sin(dlon / 2.0), 2.0);
    double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
    double d = R * c;
    return d;
}

double bearing(double lat1, double lon1, double lat2, double lon2) {
    double dlon = (lon2 - lon1) * 0.01745329252;
    double y = sin(dlon) * cos(lat2 * 0.01745329252);
    double x = cos(lat1 * 0.01745329252) * sin(lat2 * 0.01745329252) - sin(lat1 * 0.01745329252) * cos(lat2 * 0.01745329252) * cos(dlon);
    double bearing = atan2(y, x) * 57.2957795131; // Convert radians to degrees
    return bearing;
}

