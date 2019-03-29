
#include "Hardware/OSWHardware.hh"
#include "Hardware/TMS320F2806.hh"
#include "Hardware/BipolarStepper.h"
extern "C"
{

	#include "Math/IQmathLib.h"
	#include "Math/clarke.h"
	#include "Math/park.h"
	#include "Math/ipark.h"

}
#include "Math/IQmathCPP.h"


#define ENCODER_CPR 5000
#define ENCODER_PPR ENCODER_CPR*4
#define STEPPER_ROTOR_TEETH 50
#define TICKS_TO_ELECTRICAL_ANGLE MATH_TWO_PI/(ENCODER_PPR/STEPPER_ROTOR_TEETH)

PARK park;
TMS320F2806 processor;
OSWDigital digital;
QuadratureEncoder encoder;
OSWInverter inverter;
CurrentSensor currentSensor;
BipolarStepper motor;

float PWMC = 0.0;
float PWMA = 0.0;
float PWMB = 0.0;
float thetaE = 0.0;
float thetaComp = 0.0;

float kp = 2.5;			//proportional gain
float kp_mirror = kp;	//mirror, do not edit;
float ki = 0.05;		//integral gain
float ki_mirror = ki; 	//mirror, do not edit
//cogging Compensation
float CCamp = 0.1;
float CCamp_mirror = CCamp; 	//mirror, do not edit


float iDes = 0.0;
float qErr = 0.0;
float qi = 0.0;
float qiMin = -2.0;
float qiMax = -qiMin;
float di = 0.0;
float diMin = qiMin;
float diMax = qiMax;
float aCount = 0;
float bCount = 0;
float cCount = 0;

_iq iq_qiMin = _IQ24(qiMin);
_iq iq_qiMax = _IQ24(qiMax);
_iq iq_diMin = _IQ24(diMin);
_iq iq_diMax = _IQ24(diMax);

float dErr = 0.0;
float d = 0.0;
float q = 0.0;
float qOut = 0.0;
float dOut = 0.0;
float a = 0.0;
float b = 0.0;

float maxCurrent = 9.0;
float minCurrent = -maxCurrent;
_iq iq_maxCurrent = _IQ24(maxCurrent);
_iq iq_minCurrent = _IQ24(minCurrent);
float minFOCCurrent = minCurrent*0.31;
float maxFOCCurrent = maxCurrent*0.31;
_iq iq_maxFOCCurrent = _IQ24(maxFOCCurrent);
_iq iq_minFOCCurrent = _IQ24(minFOCCurrent);

float max = 0.0;
float min = 0.0;
float vc =  0.0;
float va = 0.0;
float vb = 0.0;
float ia = 0.0;
float ib = 0.0;
float vectorTheta = 0.0;

_iq theta = 0;
_iq  SINE = 0;
_iq  COS = 0;
_iq iq_ia = 0;
_iq iq_ib = 0;
_iq iq_iDes = 0;
_iq iq_qErr = 0;
_iq iq_dErr = 0;
_iq iq_qi = 0;
_iq iq_di = 0;
_iq iq_qOut = 0;
_iq iq_dOut = 0;
_iq iq_Max = 0;
_iq iq_Min = 0;
_iq iq_PWMA = 0;
_iq iq_PWMB = 0;
_iq iq_PWMC = 0;
_iq iq_KP = _IQ24(kp);
_iq iq_KI = _IQ24(ki);
_iq iq_max = _IQ24(0.0);
_iq iq_min = _IQ24(0.0);
_iq iq_one_half = _IQ24(0.5);
_iq iq_one = _IQ24(1.0);
_iq iq_oneHundred = _IQ24(100.0);
_iq iq_CCamp = _IQ24(CCamp);
_iq iq_CCTerm = _IQ24(0);
float CompN = 1.1;

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

	processor = TMS320F2806();
	digital = OSWDigital();
	processor.setup(PLL_ClkFreq_80_MHz);
	processor.setupTimer0();
	setupGPIO();
	processor.setupGPIOCapture();
	encoder = QuadratureEncoder(processor,digital,ENCODER_CPR);
	inverter = OSWInverter(processor, digital,80,50,1);
	currentSensor = CurrentSensor(processor,digital,inverter);



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

	while(true)
	{
		//check if gains have changed
		if(kp != kp_mirror)
		{
			kp_mirror = kp;
			iq_KP = _IQ24(kp);
		}
		if(kp != kp_mirror)
		{
			kp_mirror = kp;
			iq_KP = _IQ24(kp);
		}
		if(CCamp != CCamp_mirror)
		{
			CCamp_mirror = CCamp;
			iq_CCamp = _IQ24(CCamp);
		}

		thetaE = fmod(((float)encoder.getShiftedTicks() * TICKS_TO_ELECTRICAL_ANGLE),MATH_TWO_PI) ;
		thetaComp = fmod(CompN*thetaE,MATH_TWO_PI);

		int offset = AdcResult.ADCRESULT3>>1;


		//iDes = -2*(dutyCycle - 0.5)*maxCurrent;
		iq_iDes = _IQ24(iDes) + _IQmpy(_IQsin(_IQ24(thetaComp)),iq_CCamp);

		//read the phase currents
		ia = -((int32_t)AdcResult.ADCRESULT0 - offset)*0.0080566;
		ib = -((int32_t)AdcResult.ADCRESULT1 - offset)*0.0080566;

		theta = _IQ24(thetaE);
		SINE = _IQ24sin(theta);
		COS = _IQ24cos(theta);
		park.Sine = SINE;
		park.Cosine = COS;
		park.Alpha = _IQ24(ia);
		park.Beta = _IQ24(ib);
		PARK_MACRO(park);

		iq_qErr = iq_iDes - park.Qs;
		iq_dErr = -park.Ds;

		iq_qi = iq_qi+_IQmpy(iq_qErr,iq_KI);
		iq_di = iq_di+_IQmpy(iq_dErr,iq_KI);

		if (iq_qi < iq_minFOCCurrent)
			iq_qi = iq_minFOCCurrent;
		if (iq_qi > iq_maxFOCCurrent)
			iq_qi = iq_maxFOCCurrent;

		if (iq_di < iq_minFOCCurrent)
			iq_di = iq_minFOCCurrent;
		if (iq_di > iq_maxFOCCurrent)
			iq_di = iq_maxFOCCurrent;


		iq_qOut = _IQmpy(iq_KP,iq_qErr) + iq_qi;
		iq_dOut = _IQmpy(iq_KP,iq_dErr)+ iq_di;

		park.Qs = iq_qOut;
		park.Ds = iq_dOut;

		IPARK_MACRO(park);

		a = _IQ24toF(park.Alpha);
		b = _IQ24toF(park.Beta);

		if (park.Alpha < iq_minFOCCurrent)
			park.Alpha = iq_minFOCCurrent;
		if (park.Alpha > iq_maxFOCCurrent)
			park.Alpha = iq_maxFOCCurrent;
		if (park.Beta < iq_minFOCCurrent)
			park.Beta = iq_minFOCCurrent;
		if (park.Beta > iq_maxFOCCurrent)
			park.Beta = iq_maxFOCCurrent;

		/*
		 * Convert the control loop output to PWM signals on phase A, B and C
		 */
		iq_max = MAX(park.Alpha, park.Beta);
		iq_min = MIN(park.Alpha, park.Beta);
		iq_PWMC = _IQmpy(iq_one_half,(iq_oneHundred-(_IQmpy((_IQdiv(iq_max,iq_maxCurrent) - _IQdiv(iq_min,iq_maxCurrent)),iq_oneHundred))));
		iq_PWMA = iq_PWMC + _IQmpy(iq_oneHundred,_IQdiv(park.Alpha,iq_maxCurrent));
		iq_PWMB = iq_PWMC + _IQmpy(iq_oneHundred,_IQdiv(park.Beta,iq_maxCurrent));

		PWMA = _IQ24toF(iq_PWMA);
		PWMB = _IQ24toF(iq_PWMB);
		PWMC = _IQ24toF(iq_PWMC);
		if(PWMA < 0)
		{
			PWMA = 0;
		}
		if(PWMA > 73)
		{
			PWMA = 73;
		}
		if(PWMB < 0)
		{
			PWMB = 0;
		}
		if(PWMB > 73)
		{
			PWMB = 73;
		}
		if(PWMC < 31)
		{
			PWMC = 31;
		}
		if(PWMC > 50)
		{
			PWMC = 50;
		}

		aCount = 20*PWMA;
		bCount = 20*PWMB;
		cCount = 20*PWMC;

		EPwm1Regs.CMPA.half.CMPA = (uint16_t)aCount;
		EPwm2Regs.CMPA.half.CMPA = (uint16_t)bCount;
		EPwm3Regs.CMPA.half.CMPA = (uint16_t)cCount;

		GpioDataRegs.GPATOGGLE.bit.GPIO27 = 1;


	}
 }
