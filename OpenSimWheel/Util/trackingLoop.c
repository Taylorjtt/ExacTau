/*
 * estimator.c
 *
 *  Created on: Jan 22, 2016
 *      Author: JohnTaylor
 */

#include "trackingLoop.h"

#include <stdlib.h>


void runTrackingLoop(TrackingLoop *est, float measuredValue)
{
	est->estimate += est->estimateDot*est->deltaT;
	est->estimateError = measuredValue - est->estimate;
	est->estimateDotIntegrator += est->estimateError*est->ki*est->deltaT;
	est->estimateDot = est->kp*est->estimateError + est->estimateDotIntegrator;

}
void resetTrackingLoop(TrackingLoop *est)
{
	est->estimate = 0.0;
	est->estimateDot = 0.0;
	est->estimateDotIntegrator = 0.0;
	est->estimateError = 0.0;
}
bool isTooLarge(float f)
{
	if(f > 1e6)
	{
		return true;
	}
	else return false;
}
void recoverTrackingLoop(TrackingLoop *est)
{
	if(isinf(est->estimate) || isinf(est->estimateDot) || isinf(est->estimateDotIntegrator)||isinf(est->estimateError))
	{
		resetTrackingLoop(est);
	}
	if(isTooLarge(est->estimate) || isTooLarge(est->estimateDot) || isTooLarge(est->estimateDotIntegrator)||isTooLarge(est->estimateError))
	{
		resetTrackingLoop(est);
	}
}
