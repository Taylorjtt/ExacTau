
#include "Hardware/OSWHardware.hh"
#include "Hardware/TMS320F2806.hh"
TMS320F2806 processor;
OSWDigital digital;
OSWSerial serial;
QuadratureEncoder encoder;
OSWInverter inverter;

float a = 0.0;
float b = 0.0;
float c = 0.0;

int main(void)
 {

	processor = TMS320F2806();
	digital = OSWDigital();


	processor.setup(PLL_ClkFreq_80_MHz);

	serial = OSWSerial(processor,digital,SCI_BaudRate_115_2_kBaud);
	encoder = QuadratureEncoder(processor,digital,5000);
	inverter = OSWInverter(processor, digital,80,50,1);

	digital.setMode(GPIO_Number_34,GPIO_34_Mode_GeneralPurpose);
	digital.setMode(GPIO_Number_39,GPIO_39_Mode_GeneralPurpose);
	digital.setDirection(GPIO_Number_34,GPIO_Direction_Output);
	digital.setDirection(GPIO_Number_39,GPIO_Direction_Output);



	while(true)
	{
		inverter.modulate(a,b,c);
		digital.toggle(GPIO_Number_34);
		DELAY_US(1e5);
		serial.send('A');
		digital.toggle(GPIO_Number_39);
		DELAY_US(1e5);
		serial.send('B');
	}

}
