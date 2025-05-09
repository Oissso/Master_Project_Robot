/*
 * gnss.h
 *
 *  Created on: Apr 17, 2025
 *      Author: theo8
 */

#ifndef INC_GNSS_H_
#define INC_GNSS_H_

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
//#include <proj.h>


#define GNSS_ADDRESS_BASE 0x40

// Définition des structures

typedef struct {
    double latitude;  // WGS84 (degrés)
    double longitude; // WGS84 (degrés)
} GeoPoint;

typedef struct {
    double easting;   // UTM (mètres)
    double northing;  // UTM (mètres)
    int zone;        // Fuseau UTM (1-60)
    char band;       // Bande latitudinale (C-X)
} UTMPoint;

UTMPoint geoToUTM(GeoPoint geo);
double distanceUTM(UTMPoint a, UTMPoint b);
double perpendicularDistance(UTMPoint p, UTMPoint a, UTMPoint b);
void douglasPeucker(UTMPoint *points, int start, int end, double epsilon, bool *keep);
void simplifyTrack(UTMPoint *points, int *count, double epsilon);
GeoPoint utmToGeo(UTMPoint utm);
void processGNSS(GeoPoint *output, int *numPoints);


#endif /* INC_GNSS_H_ */
