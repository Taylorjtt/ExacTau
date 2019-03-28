/*
 * BipolarStepper.cpp
 *
 *  Created on: Aug 30, 2018
 *      Author: John
 */

#include "BipolarStepper.h"



BipolarStepper::BipolarStepper()
{
	// TODO Auto-generated constructor stub
}
BipolarStepper::BipolarStepper(OSWInverter inverter)
{
	this->inv = inverter;
	thetaE = 0.0;
	thetaEIncrement = 0.00314;
	speed = 5;
	multiplier = 0.5;
	vAlpha = 0.0;
	vBeta = 0.0;
	max = 0.0;
	min = 0.0;
	c = 0.0;
	a = 0.0;
	b = 0.0;

}
void BipolarStepper::zero(QuadratureEncoder &encoder)
{

	microstepCW(100000);
	int32_t distance = (int32_t)encoder.getRawTicks() - 7591;
	if(distance < 0)
	{
		while(distance < 0)
		{
			microstepCCW();
			DELAY_US(speed);
			distance = (int32_t)encoder.getRawTicks() - 7591;
		}
	}
	else
	{
		while(distance > 0)
		{
			microstepCW();
			DELAY_US(speed);
			distance = (int32_t)encoder.getRawTicks() - 7591;
		}
	}
	DELAY_US(1e6);
	inv.modulate(0,10,10);
	DELAY_US(500000);
	encoder.zero();
	DELAY_US(500000);
	inv.modulate(0,0,0);

}

void BipolarStepper::recalibrate(QuadratureEncoder &encoder)
{
	inv.modulate(65.5,34.5,34.5);
	DELAY_US(2e6);
	encoder.zero();
	DELAY_US(2e6);
	int i = 0;
	for(i = 0; i < 30; i++)
	{
		inv.modulate(65.5-i,34.5,34.5);
		DELAY_US(5e3);
	}
}
void BipolarStepper::microstepCW()
{
	thetaE = fmod(thetaE + thetaEIncrement,2*MATH_PI);
	vAlpha = 7.5*sin(thetaE);
	vBeta = 7.5*cos(thetaE);
	max = MAX(vAlpha, vBeta);
	min = MIN(vAlpha, vBeta);
	c =  0.5 * (24.0 - (max-min));
	a = c + vAlpha;
	b = c + vBeta;
	c = 4.1666*c;
	a = 4.1666*a;
	b = 4.1666*b;
	inv.modulate(multiplier*a,multiplier*b,multiplier*c);
}
void BipolarStepper::microstepCCW()
{
	thetaE = fmod(thetaE - thetaEIncrement,2*MATH_PI);
	vAlpha = 7.5*sin(thetaE);
	vBeta = 7.5*cos(thetaE);
	max = MAX(vAlpha, vBeta);
	min = MIN(vAlpha, vBeta);
	c =  0.5 * (24.0 - (max-min));
	a = c + vAlpha;
	b = c + vBeta;
	c = 4.1666*c;
	a = 4.1666*a;
	b = 4.1666*b;
	inv.modulate(multiplier*a,multiplier*b,multiplier*c);

}

void BipolarStepper::microstepCW(uint32_t numberOfSteps)
{
	for(uint32_t i = 0; i < numberOfSteps; i ++)
	{
		microstepCW();
		DELAY_US(speed);
	}

}
void BipolarStepper::microstepCCW(uint32_t numberOfSteps)
{
	for(uint32_t i = 0; i < numberOfSteps; i ++)
	{
		microstepCCW();
		DELAY_US(speed);
	}

}

BipolarStepper::~BipolarStepper() {
	// TODO Auto-generated destructor stub
}


