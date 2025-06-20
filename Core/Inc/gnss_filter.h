/*
 * gnss_filter.h
 *
 *  Created on: May 19, 2025
 *      Author: theo8
 */

// ==================== gnss_filter.h ====================

#ifndef INC_GNSS_FILTER_H_
#define INC_GNSS_FILTER_H_

#include "gnss_conversion.h"

int removeOutliers(UTMPoint* in, UTMPoint* out, int n, double threshold);
int simplifyTrack(UTMPoint* in, UTMPoint* out, int n, double epsilon);

#endif /* INC_GNSS_FILTER_H_ */
