/*
 * OSWDigital.cc
 *
 *  Created on: Feb 2, 2017
 *      Author: JohnTaylor
 */

#include "OSWHardware.hh"


OSWDigital::OSWDigital()
{
	this->gpio = GPIO_init((void *)GPIO_BASE_ADDR,sizeof(GPIO_Obj));
}

void OSWDigital::setup(void)
{
	GPIO_setDirection(this->gpio,GPIO_Number_34,GPIO_Direction_Output);
	GPIO_setDirection(this->gpio,GPIO_Number_36,GPIO_Direction_Output);
}

void OSWDigital::toggle(GPIO_Number_e gpioNumber)
{
	GPIO_toggle(this->gpio,gpioNumber);
}
bool OSWDigital::read(GPIO_Number_e gpioNumber)
{
	return GPIO_read(this->gpio,gpioNumber);
}
void OSWDigital::write(GPIO_Number_e gpioNumber, bool value)
{
	if(value)
	{
		GPIO_setHigh(this->gpio,gpioNumber);
	}
	else
	{
		GPIO_setLow(this->gpio,gpioNumber);
	}
}
void OSWDigital::setDirection(GPIO_Number_e gpioNumber, GPIO_Direction_e direction)
{
	GPIO_setDirection(this->gpio, gpioNumber,direction);
}
void OSWDigital::setMode(GPIO_Number_e gpioNumber, GPIO_Mode_e mode)
{
	GPIO_setMode(this->gpio,gpioNumber,mode);
}
void OSWDigital::setQualification(GPIO_Number_e gpioNumber,GPIO_Qual_e qualification)
{
	GPIO_setQualification(this->gpio,gpioNumber,qualification);
}
void OSWDigital::setPullUp(GPIO_Number_e gpioNumber,GPIO_Pullup_e pullup)
{
	GPIO_setPullup(this->gpio,gpioNumber,pullup);
}
OSWDigital::~OSWDigital()
{

}
