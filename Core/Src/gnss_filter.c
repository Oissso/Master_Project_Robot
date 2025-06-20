/*
 * gnss_filter.c
 *
 *  Created on: May 19, 2025
 *      Author: theo8
 */

// ==================== gnss_filter.c ====================
#include "gnss_filter.h"

int removeOutliers(UTMPoint* in, UTMPoint* out, int n, double threshold) {
    if (n < 3) return 0;
    int j = 0;
    for (int i = 1; i < n-1; ++i) {
        double d1 = distanceUTM(in[i-1], in[i]);
        double d2 = distanceUTM(in[i], in[i+1]);
        if (d1 <= threshold && d2 <= threshold) {
            out[j++] = in[i];
        }
    }
    return j;
}

// Douglas-Peucker helpers
static double perpendicularDist(UTMPoint p, UTMPoint a, UTMPoint b) {
    double dx = b.easting - a.easting;
    double dy = b.northing - a.northing;
    double mag = dx*dx + dy*dy;
    double u = ((p.easting - a.easting)*dx + (p.northing - a.northing)*dy)/mag;
    double ix = a.easting + u*dx;
    double iy = a.northing + u*dy;
    return sqrt(pow(p.easting - ix, 2) + pow(p.northing - iy, 2));
}

static void dpSimplify(UTMPoint* points, int first, int last, double epsilon, int* keep) {
    if (last <= first + 1) return;
    double dmax = 0;
    int index = -1;
    for (int i = first + 1; i < last; ++i) {
        double d = perpendicularDist(points[i], points[first], points[last]);
        if (d > dmax) {
            index = i;
            dmax = d;
        }
    }
    if (dmax > epsilon && index != -1) {
        keep[index] = 1;
        dpSimplify(points, first, index, epsilon, keep);
        dpSimplify(points, index, last, epsilon, keep);
    }
}

int simplifyTrack(UTMPoint* in, UTMPoint* out, int n, double epsilon) {
    int* keep = calloc(n, sizeof(int));
    keep[0] = keep[n-1] = 1;
    dpSimplify(in, 0, n-1, epsilon, keep);

    int j = 0;
    for (int i = 0; i < n; ++i) {
        if (keep[i]) out[j++] = in[i];
    }
    free(keep);
    return j;
}
