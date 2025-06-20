/*
 * gnss_conversion.h
 *
 *  Created on: May 19, 2025
 *      Author: theo8
 */

// ========================== gnss_conversion.h ==========================

#ifndef INC_GNSS_CONVERSION_H_
#define INC_GNSS_CONVERSION_H_

#include <math.h>

#define WGS84_A 6378137.0
#define WGS84_E 0.081819191
#define UTM_SCALE_FACTOR 0.9996
#define M_PI 3.14159265358979323846

typedef struct {
    double lat;
    double lon;
} GeoPoint;

typedef struct {
    double easting;
    double northing;
    int zone;
} UTMPoint;

UTMPoint geoToUTM(GeoPoint geo);
GeoPoint utmToGeo(UTMPoint utm);

double distanceUTM(UTMPoint a, UTMPoint b);

#endif /* INC_GNSS_CONVERSION_H_ */
