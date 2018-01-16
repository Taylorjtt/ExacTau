/*
 * OSWComponents.hh
 *
 *  Created on: Feb 2, 2017
 *      Author: JohnTaylor
 */

#ifndef HARDWARE_OSWHARDWARE_HH_
#define HARDWARE_OSWHARDWARE_HH_
#include "Hardware.hh"

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
	OSWSerial(OSWDigital digital,SCI_BaudRate_e baudRate);
	void send(char c);
	virtual ~OSWSerial();
private:
	SCI_Handle sciHandle;
};
class OSWInverter: public Inverter
{
public:
	OSWInverter();
	void modulate(float ah, float al, float bh, float bl, float ch, float cl);
private:
	PWM_Handle pwmA;
	PWM_Handle pwmB;
	PWM_Handle pwmC;
};
class QuadratureEncoder:public Encoder
{
public:
	QuadratureEncoder();
	QuadratureEncoder(OSWDigital digital, uint16_t countsPerRevolution);
	float getPositionInRadians();
	float getPositionInDegrees();
	float getVelocityInRadiansPerSecond();
	float getVelocityInDegreesPerSecond();
	float getVelocityInRPM();
	void setOffsetInRadians(float offset);
	virtual ~QuadratureEncoder(){};
private:

	QEP_Handle qepHandle;
	uint32_t countsPerRev;
};

#endif /* HARDWARE_OSWHARDWARE_HH_ */
