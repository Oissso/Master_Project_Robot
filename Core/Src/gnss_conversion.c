/*
 * gnss_conversion.c
 *
 *  Created on: May 19, 2025
 *      Author: theo8
 */

// ========================== gnss_conversion.c ==========================

#include "gnss_conversion.h"

UTMPoint geoToUTM(GeoPoint geo) {
    double latRad = geo.lat * M_PI / 180.0;
    double lonRad = geo.lon * M_PI / 180.0;
    int zone = (int)(floor((geo.lon + 180.0)/6.0) + 1);

    double N = WGS84_A / sqrt(1 - WGS84_E*WGS84_E * sin(latRad)*sin(latRad));
    double T = tan(latRad)*tan(latRad);
    double C = WGS84_E*WGS84_E * cos(latRad)*cos(latRad);
    double A = cos(latRad) * (lonRad - ((zone - 1)*6 - 177)*M_PI/180);

    double M = WGS84_A * ((1 - WGS84_E*WGS84_E/4 - 3*pow(WGS84_E,4)/64 - 5*pow(WGS84_E,6)/256)*latRad
        - (3*WGS84_E*WGS84_E/8 + 3*pow(WGS84_E,4)/32 + 45*pow(WGS84_E,6)/1024)*sin(2*latRad)
        + (15*pow(WGS84_E,4)/256 + 45*pow(WGS84_E,6)/1024)*sin(4*latRad)
        - (35*pow(WGS84_E,6)/3072)*sin(6*latRad));

    double easting = UTM_SCALE_FACTOR * N * (A + (1 - T + C)*pow(A,3)/6 + (5 - 18*T + T*T + 72*C - 58*WGS84_E*WGS84_E)*pow(A,5)/120) + 500000.0;
    double northing = UTM_SCALE_FACTOR * (M + N*tan(latRad)*(pow(A,2)/2 + (5 - T + 9*C + 4*C*C)*pow(A,4)/24 + (61 - 58*T + T*T + 600*C - 330*WGS84_E*WGS84_E)*pow(A,6)/720));
    if (geo.lat < 0) northing += 10000000.0;

    UTMPoint utm = {easting, northing, zone};
    return utm;
}

GeoPoint utmToGeo(UTMPoint utm) {
    double x = utm.easting - 500000.0;
    double y = utm.northing;
    double a = WGS84_A;
    double e = WGS84_E;
    double k0 = UTM_SCALE_FACTOR;

    double e1sq = 0.006739497;
    double lon_origin = (utm.zone - 1)*6 - 180 + 3;

    double M = y / k0;
    double mu = M / (a * (1 - e*e/4 - 3*pow(e,4)/64 - 5*pow(e,6)/256));

    double e1 = (1 - sqrt(1 - e*e)) / (1 + sqrt(1 - e*e));
    double J1 = 3*e1/2 - 27*pow(e1,3)/32;
    double J2 = 21*pow(e1,2)/16 - 55*pow(e1,4)/32;
    double J3 = 151*pow(e1,3)/96;
    double J4 = 1097*pow(e1,4)/512;

    double fp = mu + J1*sin(2*mu) + J2*sin(4*mu) + J3*sin(6*mu) + J4*sin(8*mu);

    double C1 = e1sq * cos(fp)*cos(fp);
    double T1 = tan(fp)*tan(fp);
    double N1 = a / sqrt(1 - e*e * sin(fp)*sin(fp));
    double R1 = a * (1 - e*e) / pow(1 - e*e * sin(fp)*sin(fp), 1.5);
    double D = x / (N1 * k0);

    double lat = fp - (N1*tan(fp)/R1) * (D*D/2 - (5 + 3*T1 + 10*C1 - 4*C1*C1 - 9*e1sq)*pow(D,4)/24 + (61 + 90*T1 + 298*C1 + 45*T1*T1 - 252*e1sq - 3*C1*C1)*pow(D,6)/720);
    double lon = lon_origin*M_PI/180 + (D - (1 + 2*T1 + C1)*pow(D,3)/6 + (5 - 2*C1 + 28*T1 - 3*C1*C1 + 8*e1sq + 24*T1*T1)*pow(D,5)/120)/cos(fp);

    GeoPoint geo = {lat * 180/M_PI, lon * 180/M_PI};
    return geo;
}

double distanceUTM(UTMPoint a, UTMPoint b) {
    return sqrt(pow(a.easting - b.easting, 2) + pow(a.northing - b.northing, 2));
}
