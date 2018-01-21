
#include "Hardware/OSWHardware.hh"
#include "Hardware/TMS320F2806.hh"
TMS320F2806 processor;
OSWDigital digital;
OSWSerial serial;
QuadratureEncoder encoder;
OSWInverter inverter;
DRV8301 driver;
Spi spi;
uint32_t delay = 3000;
int phase = 0;
float voltage = 100;
uint32_t speed = 3000;
bool doStep = false;
void step(OSWInverter inv, int phase)
{
	if(phase == 0)
	{
		inv.modulate(100,0,0); //1
		DELAY_US(delay);
		inv.modulate(100,100,0);//2
	}

	if(phase == 1)
	{
		inv.modulate(0,100,0);//3
		DELAY_US(delay);
		inv.modulate(0,100,50);//4

	}
	if(phase == 2)
	{
		inv.modulate(0,100,100);//5
		DELAY_US(delay);
		inv.modulate(0,0,100);//6
	}


	if(phase == 3)
	{
		inv.modulate(100,0,100);//7
		DELAY_US(delay);
		inv.modulate(100,0,50);//8
	}
	DELAY_US(delay);
	//inv.modulate(0,0,0);
}

int main(void)
 {

	processor = TMS320F2806();
	digital = OSWDigital();

	processor.setup(PLL_ClkFreq_80_MHz);

	serial = OSWSerial(processor,digital,SCI_BaudRate_115_2_kBaud);
	encoder = QuadratureEncoder(processor,digital,5000);
	inverter = OSWInverter(processor, digital,80,50,1);
	spi = Spi(processor, digital);
	driver = DRV8301(processor, digital,spi);

	digital.setMode(GPIO_Number_34,GPIO_34_Mode_GeneralPurpose);
	digital.setMode(GPIO_Number_39,GPIO_39_Mode_GeneralPurpose);
	digital.setDirection(GPIO_Number_34,GPIO_Direction_Output);
	digital.setDirection(GPIO_Number_39,GPIO_Direction_Output);





	while(true)
	{
//		digital.toggle(GPIO_Number_34);
//		DELAY_US(1e4);
//		serial.send('A');
//		digital.toggle(GPIO_Number_39);
//		DELAY_US(1e4);
//		serial.send('B');
		driver.readDriverData();
		if(doStep)
		{
			step(inverter,phase);
			DELAY_US(speed);
			//doStep = false;
			phase++;
			if(phase > 3)
				phase = 0;
		}

		DELAY_US(speed);

	}

}
