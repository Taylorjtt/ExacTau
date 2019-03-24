
#include "Hardware/OSWHardware.hh"
#include "Hardware/TMS320F2806.hh"
#include "Scheduler/TaskTable.h"
#include "Hardware/BipolarStepper.h"
extern "C"
{
	#include "control/TorqueController.h"
	#include "Math/IQmathLib.h"

}
#include "Math/IQmathCPP.h"
#include "Scheduler/SerialSendTask.h"


#define ENCODER_CPR 5000
PARK park;
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
BipolarStepper motor;
uint32_t delay = 1000;
float PWMC = 0.0;
float PWMA = 0.0;
float PWMB = 0.0;

float thetaE = 0.0;
bool cw = false;
_iq theta = 0;
_iq  SINE = 0;
_iq  COS = 0;
float aa = 0;
float bb = 0;
float cc = 0;
float Valpha = 0.0;
float kp = 2.75;
float ki = 0.0;
float iDes = 0.0;
float qErr = 0.0;
float qi = 0.0;
float qiMin = -5.0;
float qiMax = 5.0;
float di = 0.0;
float diMin = -10.0;
float diMax = 10.0;
float dErr = 0.0;
float d = 0.0;
float q = 0.0;
float qOut = 0.0;
float dOut = 0.0;
float a = 0.0;
float b = 0.0;
bool calibrate = false;
float maxCurrent = 9.0;
float minCurrent = -maxCurrent;

float Vbeta = 0.0;
float max = 0.0;
float min = 0.0;
float vc =  0.0;
float va = 0.0;
float vb = 0.0;
float mult = 0.1;
float ia = 0.0;
float ib = 0.0;
float offset =	0.0;
float vectorTheta = 0.0;
bool doStep = false;
uint32_t ppr = 20000;
int32_t encoderOffset = 0;
void setupGPIO()
{
	digital.setDirection(GPIO_Number_10, GPIO_Direction_Input);
	digital.setDirection(GPIO_Number_11, GPIO_Direction_Input);
	digital.setMode(GPIO_Number_11,GPIO_11_Mode_ECAP1);
	digital.setMode(GPIO_Number_10,GPIO_10_Mode_GeneralPurpose);
	digital.setMode(GPIO_Number_33,GPIO_33_Mode_GeneralPurpose);
	digital.setMode(GPIO_Number_34,GPIO_34_Mode_GeneralPurpose);
	digital.setMode(GPIO_Number_39,GPIO_39_Mode_GeneralPurpose);
	digital.setDirection(GPIO_Number_33,GPIO_Direction_Output);
	digital.setDirection(GPIO_Number_34,GPIO_Direction_Output);
	digital.setDirection(GPIO_Number_39,GPIO_Direction_Output);
	digital.setDirection(GPIO_Number_27,GPIO_Direction_Output);
	digital.setMode(GPIO_Number_27,GPIO_27_Mode_GeneralPurpose);
}

int main(void)
 {

	torqueController = (TorqueControllerHandle)malloc(sizeof(TorqueController_Obj));
	processor = TMS320F2806();
	digital = OSWDigital();
	processor.setup(PLL_ClkFreq_80_MHz);
	processor.setupTimer0();
	setupGPIO();
	processor.setupGPIOCapture();
	serial = OSWSerial(processor,digital,SCI_BaudRate_115_2_kBaud);
	encoder = QuadratureEncoder(processor,digital,ENCODER_CPR);
	inverter = OSWInverter(processor, digital,80,50,1);

	currentSensor = CurrentSensor(processor,digital,inverter);
	torqueController = TorqueController_Constructor((void *)torqueController, sizeof(TorqueController_Obj));
	serialTask =  SerialSendTask(FREQ_100HZ,0,serial,digital);
	taskTable.addTask(serialTask);

	park.Alpha = 0;
	park.Angle = 0;
	park.Beta = 0;
	park.Cosine = 0;
	park.Qs = 0;
	park.Sine = 0;
	park.Ds = 0;

	inverter.enable(true);
	motor = BipolarStepper(inverter);

	motor.zero(encoder);
	encoderOffset = EQep1Regs.QPOSCNT;

	while(true)
	{
//
//		int shifted = (EQep1Regs.QPOSCNT - encoderOffset);
//		if(shifted < 0)
//		{
//			shifted = ppr + shifted;
//		}
		thetaE = fmod(((float)encoder.getShiftedTicks() * 0.0157),MATH_TWO_PI) ;
		vectorTheta = fmod(thetaE,MATH_TWO_PI);

		int offset = AdcResult.ADCRESULT3>>1;


		iDes = -2*(dutyCycle - 0.5)*9.0;

		//read the phase currents
		ia = -((int32_t)AdcResult.ADCRESULT0 - offset)*0.0080566;
		ib = -((int32_t)AdcResult.ADCRESULT1 - offset)*0.0080566;

		theta = _IQ24(vectorTheta);
		SINE = _IQ24sin(theta);
		COS = _IQ24cos(theta);
		park.Sine = SINE;
		park.Cosine = COS;
		park.Alpha = _IQ24(ia);
		park.Beta = _IQ24(ib);
		PARK_MACRO(park);
		d = _IQ24toF(park.Ds);
		q = _IQ24toF(park.Qs);
		qErr = iDes - q;
		dErr = 0.0-d;
		qi = qErr*ki;
		di = dErr*ki;
		if (qi < qiMin)
			qi = qiMin;
		if (qi > qiMax)
			qi = qiMax;
		if (di < diMin)
			di = diMin;
		if (di > diMax)
			di = diMax;

		qOut = kp * qErr + qi;
		dOut = kp * dErr + di;

		if (qOut < minCurrent*.31)
			qOut = minCurrent*.31;
		if (qOut > maxCurrent*.31)
			qOut = maxCurrent*.31;
		if (dOut < minCurrent*.31)
			dOut = minCurrent*.31;
		if (dOut > maxCurrent*.31)
			dOut = maxCurrent*.31;

		park.Qs = _IQ24(qOut);
		park.Ds = _IQ24(dOut);

		IPARK_MACRO(park);

		a = _IQ24toF(park.Alpha);
		b = _IQ24toF(park.Beta);

		if (a < minCurrent*.31)
			a = minCurrent*.31;
		if (a > maxCurrent*.31)
			a = maxCurrent*.31;
		if (b < minCurrent*.31)
			b = minCurrent*.31;
		if (b > maxCurrent*.31)
			b = maxCurrent*.31;

		max = MAX(a, b);
		min = MIN(a, b);

		GpioDataRegs.GPATOGGLE.bit.GPIO27 = 1;

		PWMC = 0.5*(100-((max/maxCurrent - min/maxCurrent)*100));
		PWMA = PWMC + (a/maxCurrent)*100;
		PWMB = PWMC + (b/maxCurrent)*100;

		float aCount = 20*PWMA;
		float bCount = 20*PWMB;
		float cCount = 20*PWMC;

		EPwm1Regs.CMPA.half.CMPA = (uint16_t)aCount;
		EPwm2Regs.CMPA.half.CMPA = (uint16_t)bCount;
		EPwm3Regs.CMPA.half.CMPA = (uint16_t)cCount;


	}
 }
