/*
 * DRV8301.cc
 *
 *  Created on: Jan 16, 2018
 *      Author: John
 */
#include "OSWHardware.hh"
DRV8301::DRV8301()
{

}
DRV8301::DRV8301(TMS320F2806 processor, OSWDigital digital, Spi spi)
{


	this->driver = DRV8301_init(&this->driver,sizeof(this->driver));
	//enable
	digital.setDirection(GPIO_Number_50, GPIO_Direction_Output);
	//fault
	digital.setDirection(GPIO_Number_28,GPIO_Direction_Input);
	//OCTW
	digital.setDirection(GPIO_Number_29,GPIO_Direction_Input);
	//DC_CAL
	digital.setDirection(GPIO_Number_51,GPIO_Direction_Output);

	digital.write(GPIO_Number_51,false);
	DRV8301_setGpioNumber(this->driver,GPIO_Number_50);
	DRV8301_setGpioHandle(this->driver,digital.getGPIOHandle());
	DRV8301_enable(this->driver);

	DRV8301_setSpiHandle(this->driver,spi.getSpiHandle());

	DRV8301_setupSpi(this->driver,&(this->driverVars));

	DRV8301_writeData(this->driver,&(this->driverVars));
}

void DRV8301::readDriverData()
{
	DRV8301_readData(this->driver,&(this->driverVars));
}
void DRV8301::writeDriverData()
{
	DRV8301_writeData(this->driver,&(this->driverVars));
}


