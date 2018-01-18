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
	digital.setDirection(GPIO_Number_50, GPIO_Direction_Output);
	DRV8301_setGpioNumber(this->driver,GPIO_Number_50);
	DRV8301_setGpioHandle(this->driver,digital.getGPIOHandle());
	DRV8301_enable(this->driver);

	DRV8301_setSpiHandle(this->driver,spi.getSpiHandle());

	DRV8301_setupSpi(this->driver,&(this->driverVars));
	DRV8301_setPwmMode(this->driver,DRV8301_PwmMode_Three_Inputs);
	DRV8301_setOcMode(this->driver,DRV8301_OcMode_CurrentLimit);
	DRV8301_setPeakCurrent(this->driver,DRV8301_PeakCurrent_0p25_A);
	DRV8301_setOcLevel(this->driver,DRV8301_VdsLevel_2p400_V);
	DRV8301_setShuntAmpGain(this->driver,DRV8301_ShuntAmpGain_10VpV);




}
void DRV8301::readDriverData()
{
	DRV8301_readData(this->driver,&(this->driverVars));
}
void DRV8301::writeDriverData()
{
	DRV8301_writeData(this->driver,&(this->driverVars));
}


