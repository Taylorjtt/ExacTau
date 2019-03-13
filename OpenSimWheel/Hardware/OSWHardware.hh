/*
 * OSWComponents.hh
 *
 *  Created on: Feb 2, 2017
 *      Author: JohnTaylor
 */

#ifndef HARDWARE_OSWHARDWARE_HH_
#define HARDWARE_OSWHARDWARE_HH_
#include "Hardware.hh"
#include "TMS320F2806.hh"
__interrupt void coilACurrentSense(void);

class OSWDigital : public DigitalHardware{
public:
	OSWDigital(void);
	~OSWDigital();
	void setup(void);
	bool read(GPIO_Number_e gpioNumber);
	void write(GPIO_Number_e, bool value);
	void toggle(GPIO_Number_e gpioNumber);
	void setMode(GPIO_Number_e gpioNumber, GPIO_Mode_e mode);
	void setDirection(GPIO_Number_e gpioNumber, GPIO_Direction_e direction);
	void setPullUp(GPIO_Number_e number,GPIO_Pullup_e pullUp);
	void setQualification(GPIO_Number_e number, GPIO_Qual_e qualification);
	GPIO_Handle getGPIOHandle(){return gpio;};
private:
	GPIO_Handle gpio;
};

class OSWSerial: public Serial {
public:
	OSWSerial();
	OSWSerial(TMS320F2806 processor,OSWDigital digital,SCI_BaudRate_e baudRate);
	void send(char c);
	void send3Bytes(uint32_t bytes);
	virtual ~OSWSerial();
private:
	SCI_Handle sciHandle;
};
class OSWInverter: public Inverter
{
public:
	OSWInverter();
	OSWInverter(TMS320F2806 processor, OSWDigital digital,const float_t systemFreq_MHz,const float_t pwmPeriod_usec,
			const uint_least16_t numPwmTicksPerIsrTick);
	void modulate(float ah, float bh, float ch);
	PWM_Handle getPWMHandle(PWM_Number_e number);
	void enable(bool enabled);
private:
	PWM_Handle pwmHandle[3];
	OSWDigital _digital;
	float dutyToCounts;
	float ADuty;
	float BDuty;
	float CDuty;
	bool enabled;
};
class QuadratureEncoder:public Encoder
{
public:
	QuadratureEncoder();
	QuadratureEncoder(TMS320F2806 processor,OSWDigital digital, uint16_t countsPerRevolution);
	uint32_t getRawTicks();
	float getPositionInRadians();
	float getPositionInDegrees();
	float getVelocityInRadiansPerSecond();
	float getVelocityInDegreesPerSecond();
	float getVelocityInRPM();
	uint32_t getShiftedTicks();
	void zero();
	virtual ~QuadratureEncoder(){};
private:

	QEP_Handle qepHandle;
	uint32_t countsPerRev;
	uint32_t ppr;
	int32_t offset;
};
class Spi
{
public:
	Spi();
	Spi(TMS320F2806 processor, OSWDigital digital);
	SPI_Handle getSpiHandle();

private:
	SPI_Handle spi;

};
class DRV8301
{
public:
	DRV8301();
	DRV8301(TMS320F2806 processor, OSWDigital digital,Spi spi);
	void readDriverData();
	void writeDriverData();
private:
DRV8301_Handle driver;
DRV_SPI_8301_Vars_t driverVars;

};
class CurrentSensor
{
public:
	CurrentSensor();
	CurrentSensor(TMS320F2806 processor, OSWDigital digital, OSWInverter inverter);
	float getACurrent();
	float getBCurrent();
	float getCCurrent();

private:
	ADC_Handle adc;
	float phaseACurrent;
	float phaseBCurrent;
	float phaseCCurrent;
	float currentFromADCCounts(int16_t adcCounts);


};


#endif /* HARDWARE_OSWHARDWARE_HH_ */
