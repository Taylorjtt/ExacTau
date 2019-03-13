/*
 * CurrentSensor.cc
 *
 *  Created on: Jan 23, 2018
 *      Author: John
 */

#include "OSWHardware.hh"
CurrentSensor::CurrentSensor()
{

}

CurrentSensor::CurrentSensor(TMS320F2806 processor, OSWDigital digital, OSWInverter inverter)
{
	this->adc = ADC_init((void *)ADC_BASE_ADDR,sizeof(ADC_Obj));

	phaseACurrent = 0.0f;
	phaseBCurrent = 0.0f;
	phaseCCurrent = 0.0f;

	ENABLE_PROTECTED_REGISTER_WRITE_MODE;
	(*Device_cal)();
	DISABLE_PROTECTED_REGISTER_WRITE_MODE;
	processor.enableADCClock();

	ADC_enableBandGap(this->adc);
	ADC_enableRefBuffers(this->adc);
	ADC_powerUp(this->adc);
	ADC_enable(this->adc);
	ADC_setVoltRefSrc(this->adc, ADC_VoltageRefSrc_Int);
	ADC_enableNoOverlapMode(this->adc);

	//set up the Coil A start of conversion
	ADC_setSocChanNumber(this->adc,ADC_SocNumber_0, ADC_SocChanNumber_A0);
	ADC_setSocSampleDelay(this->adc, ADC_SocNumber_0, ADC_SocSampleDelay_35_cycles);
	ADC_setSocTrigSrc(this->adc, ADC_SocNumber_0, ADC_SocTrigSrc_EPWM1_ADCSOCA);

	//set up the PWM trigger for Coil A current sense
	PWM_enableSocAPulse(inverter.getPWMHandle(PWM_Number_1));
	PWM_setSocAPulseSrc(inverter.getPWMHandle(PWM_Number_1), PWM_SocPulseSrc_CounterEqualZero);
	PWM_setSocAPeriod(inverter.getPWMHandle(PWM_Number_1), PWM_SocPeriod_FirstEvent);


	//set up the Coil B start of conversion
	ADC_setSocChanNumber(this->adc,ADC_SocNumber_1, ADC_SocChanNumber_B0);
	ADC_setSocSampleDelay(this->adc, ADC_SocNumber_1, ADC_SocSampleDelay_35_cycles);
	ADC_setSocTrigSrc(this->adc, ADC_SocNumber_1, ADC_SocTrigSrc_EPWM2_ADCSOCA);

	//set up the PWM trigger for Coil B current sense
	PWM_enableSocAPulse(inverter.getPWMHandle(PWM_Number_2));
	PWM_setSocAPulseSrc(inverter.getPWMHandle(PWM_Number_2), PWM_SocPulseSrc_CounterEqualZero);
	PWM_setSocAPeriod(inverter.getPWMHandle(PWM_Number_2), PWM_SocPeriod_FirstEvent);

	//set up the Coil C start of conversion
	ADC_setSocChanNumber(this->adc,ADC_SocNumber_2, ADC_SocChanNumber_A1);
	ADC_setSocSampleDelay(this->adc, ADC_SocNumber_2, ADC_SocSampleDelay_35_cycles);
	ADC_setSocTrigSrc(this->adc, ADC_SocNumber_2, ADC_SocTrigSrc_EPWM3_ADCSOCA);

	//set up the PWM trigger for Coil C current sense
	PWM_enableSocAPulse(inverter.getPWMHandle(PWM_Number_3));
	PWM_setSocAPulseSrc(inverter.getPWMHandle(PWM_Number_3), PWM_SocPulseSrc_CounterEqualZero);
	PWM_setSocAPeriod(inverter.getPWMHandle(PWM_Number_3), PWM_SocPeriod_FirstEvent);

	//set up the Vref start of conversion
	ADC_setSocChanNumber(this->adc,ADC_SocNumber_3, ADC_SocChanNumber_B6);
	ADC_setSocSampleDelay(this->adc, ADC_SocNumber_3, ADC_SocSampleDelay_64_cycles);
	ADC_setSocTrigSrc(this->adc, ADC_SocNumber_3, ADC_SocTrigSrc_EPWM3_ADCSOCA);
}
float CurrentSensor::currentFromADCCounts(int16_t adcCounts)
{
	int offset = AdcResult.ADCRESULT3>>1;
	return (adcCounts - offset)*0.0080566;
}
float CurrentSensor::getACurrent()
{
	return currentFromADCCounts(AdcResult.ADCRESULT0);
}
float CurrentSensor::getBCurrent()
{
	return currentFromADCCounts(AdcResult.ADCRESULT1);
}
float CurrentSensor::getCCurrent()
{
	return currentFromADCCounts(AdcResult.ADCRESULT2);
}


