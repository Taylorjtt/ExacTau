
#include "Hardware/OSWHardware.hh"
#include "Hardware/TMS320F2806.hh"

int main(void)
{

	TMS320F2806 processor = TMS320F2806();
	OSWDigital digital = OSWDigital();


	processor.setup(PLL_ClkFreq_80_MHz);
	processor.enableSCIBClock();
	OSWSerial serial = OSWSerial(digital,SCI_BaudRate_115_2_kBaud);
	digital.setMode(GPIO_Number_34,GPIO_34_Mode_GeneralPurpose);
	digital.setMode(GPIO_Number_39,GPIO_39_Mode_GeneralPurpose);
	digital.setDirection(GPIO_Number_34,GPIO_Direction_Output);
	digital.setDirection(GPIO_Number_39,GPIO_Direction_Output);



	while(true)
	{
		digital.toggle(GPIO_Number_34);
		DELAY_US(1e5);
		serial.send('A');
		digital.toggle(GPIO_Number_39);
		DELAY_US(1e5);
		serial.send('B');
	}

}
