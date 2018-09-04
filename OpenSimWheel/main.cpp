
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

float thetaE = 0.0;
bool cw = false;
_iq theta = 0;
_iq  SINE = 0;
_iq  COS = 0;
float Valpha = 0.0;

float kp = 0.0;
float ki = 0.0;
float iDes = 0.0;
float qErr = 0.0;
float qi = 0.0;
float qiMin = -5.0;
float qiMax = 5.0;
float di = 0.0;
float diMin = -5.0;
float diMax = 5.0;
float dErr = 0.0;
float d = 0.0;
float q = 0.0;
float qOut = 0.0;
float dOut = 0.0;
float a = 0.0;
float b = 0.0;


float Vbeta = 0.0;
float max = 0.0;
float min = 0.0;
float vc =  0.0;
float va = 0.0;
float vb = 0.0;
float mult = 0.1;
float ia = 0.0;
float ib = 0.0;
float offset =	0.1;
float vectorTheta = 0.0;
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

	park.Alpha = 0;
	park.Angle = 0;
	park.Beta = 0;
	park.Cosine = 0;
	park.Qs = 0;
	park.Sine = 0;
	park.Ds = 0;

	motor = BipolarStepper(inverter);
	motor.zero(encoder);

	while(true)
	{
		thetaE = fmod(((float)encoder.getShiftedTicks() * 0.0157),MATH_TWO_PI) ;
		if(cw){
			vectorTheta = fmod(thetaE - offset,MATH_TWO_PI);
		}
		else
		{
			vectorTheta = fmod(thetaE + offset,MATH_TWO_PI);
		}
		//current sensor offset reading
		int offset = AdcResult.ADCRESULT3>>1;

		//read the phase currents
		ia = ((int32_t)AdcResult.ADCRESULT0 - offset)*0.0080566;
		ib = ((int32_t)AdcResult.ADCRESULT1 - offset)*0.0080566;

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

		qOut = kp * qErr + MATH_sat(ki*(qErr) + qi,qiMax,qiMin);
		dOut = kp * dErr + MATH_sat(ki*(dErr) + di,diMax,diMin);

		park.Qs = _IQ24(qOut);
		park.Ds = _IQ24(dOut);

		IPARK_MACRO(park);
		a = _IQ24toF(park.Alpha);
		b = _IQ24toF(park.Beta);
		max = MAX(a, b);
		min = MIN(a, b);
		vc =  0.5 * (1.0 - (max-min));
		va = vc  + a;
		vb = vc + b;
		vc = 70.0*vc;
		va = 70.0*va;
		vb = 70.0*vb;

//		Valpha = 7.5*_IQ24toF(SINE);
//		Vbeta = 7.5*_IQ24toF(COS);
//		max = MAX(Valpha, Vbeta);
//		min = MIN(Valpha, Vbeta);
//
//		va = vc  + Valpha;
//		vb = vc + Vbeta;
//		vc = 4.1666*vc;
//		va = 4.1666*va;
//		vb = 4.1666*vb;
		inverter.modulate(mult*va,mult*vb,mult*vc);
		taskTable.execute(processor);
//		//DELAY_US(del);

	}
 }
