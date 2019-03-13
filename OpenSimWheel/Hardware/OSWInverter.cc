/*
 * OSWInverter.cc
 *
 *  Created on: Jan 15, 2018
 *      Author: John
 */

#include "OSWHardware.hh"
#include "../math.h"

OSWInverter::OSWInverter()
{

}
OSWInverter::OSWInverter(TMS320F2806 processor, OSWDigital digital,const float_t systemFreq_MHz,const float_t pwmPeriod_usec,
		const uint_least16_t numPwmTicksPerIsrTick)
{

	this->enabled = false;
	this->_digital = digital;
	processor.enablePWMClock(PWM_Number_1);
	processor.enablePWMClock(PWM_Number_2);
	processor.enablePWMClock(PWM_Number_3);

	digital.setDirection(GPIO_Number_0, GPIO_Direction_Output);
	digital.setDirection(GPIO_Number_1, GPIO_Direction_Output);
	digital.setDirection(GPIO_Number_2, GPIO_Direction_Output);
	digital.setDirection(GPIO_Number_3, GPIO_Direction_Output);
	digital.setDirection(GPIO_Number_4, GPIO_Direction_Output);
	digital.setDirection(GPIO_Number_5, GPIO_Direction_Output);
	digital.setDirection(GPIO_Number_50, GPIO_Direction_Output);

	digital.setMode(GPIO_Number_0, GPIO_0_Mode_EPWM1A);
	digital.setMode(GPIO_Number_1, GPIO_1_Mode_EPWM1B);
	digital.setMode(GPIO_Number_2, GPIO_2_Mode_EPWM2A);
	digital.setMode(GPIO_Number_3, GPIO_3_Mode_EPWM2B);
	digital.setMode(GPIO_Number_4, GPIO_4_Mode_EPWM3A);
	digital.setMode(GPIO_Number_5, GPIO_5_Mode_EPWM3B);
	digital.setMode(GPIO_Number_50, GPIO_50_Mode_GeneralPurpose);
	digital.write(GPIO_Number_50,true);

	this->pwmHandle[0] = PWM_init((void *)PWM_ePWM1_BASE_ADDR,sizeof(PWM_Obj));
	this->pwmHandle[1] = PWM_init((void *)PWM_ePWM2_BASE_ADDR,sizeof(PWM_Obj));
	this->pwmHandle[2] = PWM_init((void *)PWM_ePWM3_BASE_ADDR,sizeof(PWM_Obj));

	uint16_t   halfPeriod_cycles = (uint16_t)(systemFreq_MHz*pwmPeriod_usec) >> 1;
	this->dutyToCounts = (float)halfPeriod_cycles/100.0;
	uint_least8_t    count;

	for(count = 0; count < 3; count++)
	{

		PWM_setCounterMode(this->pwmHandle[count],PWM_CounterMode_UpDown);

	    PWM_enableCounterLoad(this->pwmHandle[count]);
		PWM_setPeriodLoad(this->pwmHandle[count],PWM_PeriodLoad_Immediate);
		PWM_setSyncMode(this->pwmHandle[count],PWM_SyncMode_EPWMxSYNC);
		PWM_setHighSpeedClkDiv(this->pwmHandle[count],PWM_HspClkDiv_by_1);
		PWM_setClkDiv(this->pwmHandle[count],PWM_ClkDiv_by_1);
		PWM_setPhaseDir(this->pwmHandle[count],PWM_PhaseDir_CountUp);
		PWM_setRunMode(this->pwmHandle[count],PWM_RunMode_FreeRun);

	    // setup the Timer-Based Phase Register (TBPHS)
	    PWM_setPhase(this->pwmHandle[count],0);

	    // setup the Time-Base Counter Register (TBCTR)
	    PWM_setCount(this->pwmHandle[count],0);

	    // setup the Time-Base Period Register (TBPRD)
	    // set to zero initially
	    PWM_setPeriod(this->pwmHandle[count],0);


	    // setup the Counter-Compare Control Register (CMPCTL)
	    PWM_setLoadMode_CmpA(this->pwmHandle[count],PWM_LoadMode_Zero);
	    PWM_setLoadMode_CmpB(this->pwmHandle[count],PWM_LoadMode_Zero);
	    PWM_setShadowMode_CmpA(this->pwmHandle[count],PWM_ShadowMode_Shadow);
	    PWM_setShadowMode_CmpB(this->pwmHandle[count],PWM_ShadowMode_Shadow);

	    // setup the Action-Qualifier Output A Register (AQCTLA)
		PWM_setActionQual_CntUp_CmpA_PwmA(this->pwmHandle[count], PWM_ActionQual_Clear);
		PWM_setActionQual_CntDown_CmpA_PwmA(this->pwmHandle[count], PWM_ActionQual_Set);

	    // setup the Action-Qualifier Output B Register (AQCTLB)
		PWM_setActionQual_CntUp_CmpA_PwmB(this->pwmHandle[count],PWM_ActionQual_Set);
		PWM_setActionQual_CntDown_CmpA_PwmB(this->pwmHandle[count],PWM_ActionQual_Clear);

	    // setup the Dead-Band Generator Control Register (DBCTL)
	    PWM_setDeadBandOutputMode(this->pwmHandle[count],PWM_DeadBandOutputMode_EPWMxA_Rising_EPWMxB_Falling);
	    PWM_setDeadBandPolarity(this->pwmHandle[count],PWM_DeadBandPolarity_EPWMxB_Inverted);
	    PWM_setDeadBandFallingEdgeDelay(this->pwmHandle[count],1);
	    PWM_setDeadBandRisingEdgeDelay(this->pwmHandle[count],1);
	    PWM_disableChopping(this->pwmHandle[count]);

	    // setup the Trip Zone Select Register (TZSEL)
	    PWM_disableTripZones(this->pwmHandle[count]);
	}

	//setup the Event Trigger Selection Register (ETSEL)
	PWM_disableInt(this->pwmHandle[PWM_Number_1]);
	PWM_setSocAPulseSrc(this->pwmHandle[PWM_Number_1],PWM_SocPulseSrc_CounterEqualZero);
	PWM_enableSocAPulse(this->pwmHandle[PWM_Number_1]);


	// setup the Event Trigger Prescale Register (ETPS)
	if(numPwmTicksPerIsrTick == 3)
	{
	  PWM_setIntPeriod(this->pwmHandle[PWM_Number_1],PWM_IntPeriod_ThirdEvent);
	  PWM_setSocAPeriod(this->pwmHandle[PWM_Number_1],PWM_SocPeriod_ThirdEvent);
	}
	else if(numPwmTicksPerIsrTick == 2)
	{
	  PWM_setIntPeriod(this->pwmHandle[PWM_Number_1],PWM_IntPeriod_SecondEvent);
	  PWM_setSocAPeriod(this->pwmHandle[PWM_Number_1],PWM_SocPeriod_SecondEvent);
	}
	else
	{
	  PWM_setIntPeriod(this->pwmHandle[PWM_Number_1],PWM_IntPeriod_FirstEvent);
	  PWM_setSocAPeriod(this->pwmHandle[PWM_Number_1],PWM_SocPeriod_FirstEvent);
	}


	// setup the Event Trigger Clear Register (ETCLR)
	PWM_clearIntFlag(this->pwmHandle[PWM_Number_1]);
	PWM_clearSocAFlag(this->pwmHandle[PWM_Number_1]);

	processor.enableTbClockSync(false);

	// since the PWM is configured as an up/down counter, the period register is set to one-half
	// of the desired PWM period
	PWM_setPeriod(this->pwmHandle[PWM_Number_1],halfPeriod_cycles);
	PWM_setPeriod(this->pwmHandle[PWM_Number_2],halfPeriod_cycles);
	PWM_setPeriod(this->pwmHandle[PWM_Number_3],halfPeriod_cycles);

	// last step to synchronize the pwms
	processor.enableTbClockSync(true);
}

void OSWInverter::enable(bool enabled)
{
	if(enabled)
	{
		this->enabled = true;
		_digital.write(GPIO_Number_50,false);
	}
	else
	{
		this->enabled = false;
		_digital.write(GPIO_Number_50,true);
	}
}
void OSWInverter::modulate(float ah, float bh, float ch)
{
	ah = MATH_sat(ah, 100.0,0.0);
	bh = MATH_sat(bh, 100.0,0.0);
	ch = MATH_sat(ch, 100.0,0.0);

	ADuty = ah;
	BDuty = bh;
	CDuty = ch;

	float aCount = this->dutyToCounts*ah;
	float bCount = this->dutyToCounts*bh;
	float cCount = this->dutyToCounts*ch;

	PWM_setCmpA(this->pwmHandle[PWM_Number_1],(int)aCount);

	PWM_setCmpA(this->pwmHandle[PWM_Number_2],(int)bCount);

	PWM_setCmpA(this->pwmHandle[PWM_Number_3],(int)cCount);


}

PWM_Handle OSWInverter::getPWMHandle(PWM_Number_e number)
{
	return this->pwmHandle[number];
}

