
#include "Hardware/OSWHardware.hh"
#include "Hardware/TMS320F2806.hh"
TMS320F2806 processor;
OSWDigital digital;
OSWSerial serial;
QuadratureEncoder encoder;

int main(void)
 {

	processor = TMS320F2806();
	digital = OSWDigital();


	processor.setup(PLL_ClkFreq_80_MHz);
	processor.enableSCIBClock();
	processor.enableEQEP1Clock();

	serial = OSWSerial(digital,SCI_BaudRate_115_2_kBaud);
	encoder = QuadratureEncoder(digital, 5000);
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
