/*
 * OSWComponents.hh
 *
 *  Created on: Feb 2, 2017
 *      Author: JohnTaylor
 */

#ifndef HARDWARE_OSWCOMPONENTS_HH_
#define HARDWARE_OSWCOMPONENTS_HH_
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
private:
	GPIO_Handle gpio;
};


#endif /* HARDWARE_OSWCOMPONENTS_HH_ */
