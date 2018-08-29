
#include "Hardware/OSWHardware.hh"
#include "Hardware/TMS320F2806.hh"
#include "Scheduler/TaskTable.h"
extern "C"
{
	#include "control/TorqueController.h"
	#include "Math/IQmathLib.h"

}
#include "Math/IQmathCPP.h"
#include "Scheduler/SerialSendTask.h"


#define ENCODER_CPR 5000

TMS320F2806 processor;
OSWDigital digital;
OSWSerial serial;
QuadratureEncoder encoder;
OSWInverter inverter;
DRV8301 driver;
Spi spi;
CurrentSensor currentSensor;
SerialSendTask serialTask;
TaskTable taskTable;
TorqueControllerHandle torqueController;
float a = 0;
float b = 0;
float c = 0;
float ia = 0;
float ib = 0;
float position = 0.0;
int ticksFor1Round = 200;
float amplitude = 1.0;
float mult = 1.0;
bool initPosFlag = true;
uint32_t delay = 3000;
int phase = 0;
float voltage = 100;
uint32_t speed = 3000;
float thetaE = 0.0;

bool doStep = false;
void setupGPIO()
{
	digital.setMode(GPIO_Number_33,GPIO_33_Mode_GeneralPurpose);
	digital.setMode(GPIO_Number_34,GPIO_34_Mode_GeneralPurpose);
	digital.setMode(GPIO_Number_39,GPIO_39_Mode_GeneralPurpose);
	digital.setDirection(GPIO_Number_33,GPIO_Direction_Output);
	digital.setDirection(GPIO_Number_34,GPIO_Direction_Output);
	digital.setDirection(GPIO_Number_39,GPIO_Direction_Output);
}
void step(OSWInverter inv, int phase)
{
	ia = currentSensor.getACurrent();
	ib = currentSensor.getBCurrent();
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
void initializePosition(OSWInverter inv, QuadratureEncoder& enc)
{
	for(int i = 0; i < ticksFor1Round; i++)
	{
		if(i < 20)
		{
			speed = 3000;
			delay = 3000;
		}
		else
		{
			speed = 3000;
			delay = 3000;
		}
		step(inv,phase);
		DELAY_US(speed);
		phase++;
		if(phase > 3)
		phase = 0;
		DELAY_US(speed);
	}
	initPosFlag = false;
	DELAY_US(1e6);
	inv.modulate(75.0,0.0,0.0);
	DELAY_US(1e6);
	enc.zero();
	DELAY_US(5e5);
	inv.modulate(0,0,0);


}
int main(void)
 {
	torqueController = (TorqueControllerHandle)malloc(sizeof(TorqueController_Obj));
	processor = TMS320F2806();
	digital = OSWDigital();
	processor.setup(PLL_ClkFreq_80_MHz);
	processor.setupTimer0();
	setupGPIO();

	serial = OSWSerial(processor,digital,SCI_BaudRate_115_2_kBaud);
	encoder = QuadratureEncoder(processor,digital,ENCODER_CPR);
	inverter = OSWInverter(processor, digital,80,50,1);
	spi = Spi(processor, digital);
	driver = DRV8301(processor, digital,spi);
	currentSensor = CurrentSensor(processor,digital,inverter);
	torqueController = TorqueController_Constructor((void *)torqueController, sizeof(TorqueController_Obj));
	serialTask =  SerialSendTask(FREQ_100HZ,0,serial,digital);
	taskTable.addTask(serialTask);



	while(true)
	{
		//if(initPosFlag)
		//{
			//initializePosition(inverter,encoder);
		//}
		driver.writeDriverData();
		driver.readDriverData();
		position = encoder.getPositionInDegrees();
		thetaE = fmod((encoder.getPositionInRadians()+3.14/2.0) * 50.0,2*3.14159);
		ia = currentSensor.getACurrent();
		ib = currentSensor.getBCurrent();
		TorqueController_doControl(torqueController, ia, ib, thetaE);
		//a = mult*TorqueController_getA(torqueController);
		//b = mult*TorqueController_getB(torqueController);
		//c = mult*TorqueController_getC(torqueController);

		inverter.modulate(a,b,c);
		taskTable.execute(processor);
	}
 }
