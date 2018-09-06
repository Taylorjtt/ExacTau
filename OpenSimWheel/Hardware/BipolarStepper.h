/*
 * BipolarStepper.h
 *
 *  Created on: Aug 30, 2018
 *      Author: John
 */

#ifndef HARDWARE_BIPOLARSTEPPER_H_
#define HARDWARE_BIPOLARSTEPPER_H_
#include "Hardware.hh"
#include "../math.h"
#include "OSWHardware.hh"




class BipolarStepper {
public:
	BipolarStepper();
	BipolarStepper(OSWInverter inverter);
	void microstepCW();
	void microstepCCW();
	void microstepCW(uint32_t numberOfSteps);
	void microstepCCW(uint32_t numberOfSteps);
	void initializeEncoder();
	void recalibrate(QuadratureEncoder &encoder);
	void zero(QuadratureEncoder &encoder);
	virtual ~BipolarStepper();
private:
	OSWInverter inv;
	float thetaE;
	float thetaEIncrement;
	float vAlpha;
	float vBeta;
	float max;
	float min;
	float c;
	float a;
	float b;
	float stepsPerSecond;
	int speed;
	float multiplier;
};



#endif /* HARDWARE_BIPOLARSTEPPER_H_ */
