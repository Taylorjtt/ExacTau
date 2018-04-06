
#include "Hardware/OSWHardware.hh"
#include "Hardware/TMS320F2806.hh"
#include "Scheduler/TaskTable.h"

#define ENCODER_CPR 5000

TMS320F2806 processor;
OSWDigital digital;
OSWSerial serial;
QuadratureEncoder encoder;
OSWInverter inverter;
DRV8301 driver;
Spi spi;
CurrentSensor currentSensor;
Task LEDToggle;
TaskTable taskTable;
float a = 0;
float b = 0;
float c = 0;

void toggleLED()
{
	digital.toggle(GPIO_Number_34);
}
int main(void)
 {

	processor = TMS320F2806();
	digital = OSWDigital();
	processor.setup(PLL_ClkFreq_80_MHz);
	processor.setupTimer0();
	serial = OSWSerial(processor,digital,SCI_BaudRate_115_2_kBaud);
	encoder = QuadratureEncoder(processor,digital,ENCODER_CPR);
	inverter = OSWInverter(processor, digital,80,50,1);
	spi = Spi(processor, digital);
	driver = DRV8301(processor, digital,spi);
	currentSensor = CurrentSensor(processor,digital,inverter);
	digital.setMode(GPIO_Number_34,GPIO_34_Mode_GeneralPurpose);
	digital.setMode(GPIO_Number_39,GPIO_39_Mode_GeneralPurpose);
	digital.setDirection(GPIO_Number_34,GPIO_Direction_Output);
	digital.setDirection(GPIO_Number_39,GPIO_Direction_Output);

	LEDToggle = Task(20000,0,&toggleLED);
	taskTable.addTask(LEDToggle);
	inverter.modulate(a,b,c);

	while(true)
	{
		inverter.modulate(a,b,c);
		taskTable.execute(processor);
	}
 }
