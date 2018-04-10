/*
 * OSWSerial.cpp
 *
 *  Created on: Feb 15, 2017
 *      Author: John
 */

#include "OSWHardware.hh"
OSWSerial::OSWSerial()
{

}
OSWSerial::OSWSerial(TMS320F2806 processor,OSWDigital digital, SCI_BaudRate_e baudRate)
{
	processor.enableSCIBClock();
	digital.setPullUp(GPIO_Number_15, GPIO_Pullup_Enable); //rx
	digital.setPullUp(GPIO_Number_58, GPIO_Pullup_Disable); //tx

	digital.setDirection(GPIO_Number_58,GPIO_Direction_Output);
	digital.setQualification(GPIO_Number_15, GPIO_Qual_ASync);

	digital.setMode(GPIO_Number_15, GPIO_15_Mode_SCIRXDB);
	digital.setMode(GPIO_Number_58, GPIO_58_Mode_SCITXDB);

	this->sciHandle = SCI_init((void *)SCIB_BASE_ADDR,sizeof(SCI_Obj));
	SCI_disableParity(this->sciHandle);
	SCI_setNumStopBits(this->sciHandle, SCI_NumStopBits_One);
	SCI_setCharLength(this->sciHandle, SCI_CharLength_8_Bits);
	SCI_enableTx(this->sciHandle);
	SCI_enableRx(this->sciHandle);
	SCI_enableRxInt(this->sciHandle);
	SCI_setBaudRate(this->sciHandle, baudRate);
	SCI_enable(this->sciHandle);


}
void OSWSerial::send(char c)
{
	while(SCI_getTxFifoStatus(this->sciHandle) != SCI_FifoStatus_Empty)
	{

	}
	SCI_putDataBlocking(this->sciHandle, c);
}
void OSWSerial::send3Bytes(uint32_t bytes)
{
	int i = 0;
	for(i = 0; i < 3; i++)
	{
		char sendChar = (bytes >> i*8) & 0xFF;
		send(sendChar);
	}
}
OSWSerial::~OSWSerial()
{
	// TODO Auto-generated destructor stub
}
