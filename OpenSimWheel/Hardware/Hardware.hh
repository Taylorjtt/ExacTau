/*
 * Hardware.hh
 *
 *  Created on: Feb 2, 2017
 *      Author: JohnTaylor
 */

#ifndef HARDWARE_HARDWARE_HH_
#define HARDWARE_HARDWARE_HH_
#include <stdbool.h>
#include "../Peripherial Drivers/gpio.h"
class DigitalHardware
{
public:
	virtual ~DigitalHardware(){};
	virtual bool read(GPIO_Number_e gpioNumber) = 0;
	virtual void write(GPIO_Number_e gpioNumber, bool value) = 0;
	virtual void toggle(GPIO_Number_e gpioNumber) = 0;
	virtual void setMode(GPIO_Number_e gpioNumber, GPIO_Mode_e mode) = 0;
	virtual void setDirection(GPIO_Number_e gpioNumber, GPIO_Direction_e direction) = 0;
};
#endif /* HARDWARE_HARDWARE_HH_ */
