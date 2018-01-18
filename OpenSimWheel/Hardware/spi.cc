/*
 * spi.cc
 *
 *  Created on: Jan 17, 2018
 *      Author: John
 */
#include "OSWHardware.hh"
Spi::Spi()
{

}
Spi::Spi(TMS320F2806 processor, OSWDigital digital)
{
	this->spi = SPI_init((void *)SPIA_BASE_ADDR,sizeof(SPI_Obj));
	processor.enableSPIAClock();
	//SDI
	digital.setDirection(GPIO_Number_16, GPIO_Direction_Output);
	digital.setMode(GPIO_Number_16,GPIO_16_Mode_SPISIMOA);
	//SDO
	digital.setDirection(GPIO_Number_17, GPIO_Direction_Input);
	digital.setMode(GPIO_Number_17,GPIO_17_Mode_SPISOMIA);
	//SCLK
	digital.setDirection(GPIO_Number_18, GPIO_Direction_Output);
	digital.setMode(GPIO_Number_18,GPIO_18_Mode_SPICLKA);
	//CS
	digital.setDirection(GPIO_Number_19, GPIO_Direction_Output);
	digital.setMode(GPIO_Number_19,GPIO_19_Mode_SPISTEA_NOT);



	SPI_reset(this->spi);
	SPI_setMode(this->spi,SPI_Mode_Master);

	SPI_setClkPolarity(this->spi,SPI_ClkPolarity_OutputRisingEdge_InputFallingEdge);
	SPI_enableTx(this->spi);
	SPI_enableTxFifoEnh(this->spi);
	SPI_enableTxFifo(this->spi);
	SPI_setTxDelay(this->spi,0x0018);
	SPI_setBaudRate(this->spi,(SPI_BaudRate_e)0xD);
	SPI_setCharLength(this->spi,SPI_CharLength_16_Bits);
	SPI_setSuspend(this->spi,SPI_TxSuspend_free);
	SPI_enable(this->spi);



}
SPI_Handle Spi::getSpiHandle()
{
	return spi;
}
