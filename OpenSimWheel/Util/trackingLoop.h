/*
 * estimator.h
 *
 *  Created on: Jan 22, 2016
 *      Author: JohnTaylor
 */

#ifndef UTIL_TRACKINGLOOP_H_
#define UTIL_TRACKINGLOOP_H_

#include<math.h>
#include "stdbool.h"
typedef struct
{
	float estimate;
	float estimateError;
	float estimateDot;
	float estimateDotIntegrator;

	float kp;
	float ki;
	float deltaT;

}TrackingLoop;
void resetTrackingLoop(TrackingLoop *est);
void runTrackingLoop(TrackingLoop *est, float measuredValue);
void recoverTrackingLoop(TrackingLoop *est);
bool isTooLarge(float f);
#endif /* UTIL_TRACKINGLOOP_H_ */
