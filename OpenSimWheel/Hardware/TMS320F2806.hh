/*
 * TMS320f2806.hh
 *
 *  Created on: Feb 2, 2017
 *      Author: JohnTaylor
 */

#ifndef HARDWARE_TMS320F2806_HH_
#define HARDWARE_TMS320F2806_HH_
#include <stdlib.h>
#include "../common/DSP28x_Project.h"
#include "../Util/memCopy.h"
#include "../Peripherial Drivers/osc.h"
#include "../Peripherial Drivers/wdog.h"
#include "../Peripherial Drivers/clk.h"
#include "../Peripherial Drivers/cpu.h"
#include "../Peripherial Drivers/flash.h"
#include "../Peripherial Drivers/cap.h"
#include "../Peripherial Drivers/pie.h"
#include "../Peripherial Drivers/pll.h"
#include "../Peripherial Drivers/timer.h"
#include "../f2806_headers/F2806x_Device.h"


#define Device_cal (void   (*)(void))0x3D7C80
#define FP_SCALE 32768
#define FP_ROUND FP_SCALE/2
#define OSC_POSTRIM 32
#define OSC_POSTRIM_OFF FP_SCALE*OSC_POSTRIM
#define getOsc1FineTrimSlope() (*(int16_t (*)(void))0x3D7E90)()
#define getOsc1FineTrimOffset() (*(int16_t (*)(void))0x3D7E93)()
#define getOsc1CoarseTrim() (*(int16_t (*)(void))0x3D7E96)()
#define getOsc2FineTrimSlope() (*(int16_t (*)(void))0x3D7E99)()
#define getOsc2FineTrimOffset() (*(int16_t (*)(void))0x3D7E9C)()
#define getOsc2CoarseTrim() (*(int16_t (*)(void))0x3D7E9F)()
#define getRefTempOffset() (*(int16_t (*)(void))0x3D7EA2)()
#define HAL_PWM_DBFED_CNT         1
#define HAL_PWM_DBRED_CNT         1
extern volatile uint64_t ticks;
extern volatile float dutyCycle;
class TMS320F2806
{
public:
	TMS320F2806();
	~TMS320F2806();
	void setup(PLL_ClkFreq_e frequency);
	void enableSCIBClock();
	void enableEQEP1Clock();
	void setupGPIOCapture();
	void enableTbClockSync(bool enable);
	void enablePWMClock(PWM_Number_e number);
	void enableSPIAClock();
	void enableADCClock();
	void enableCPUInterrupt(CPU_IntNumber_e number);
	void enablePieInterrupt(PIE_GroupNumber_e group, PIE_InterruptSource_e source)
	{
		PIE_enableInt(this->pie, group, source);
	};
	void clearPieInterrupt(PIE_GroupNumber_e group)
	{
		PIE_clearInt(this->pie, group);
	};
	void registerPIEInterruptHandler(PIE_GroupNumber_e groupNumber, const PIE_SubGroupNumber_e subGroupNumber, const PIE_IntVec_t vector);
	void disablePie(void)
	{
		 PIE_disable(this->pie);
	}
	void disableAllPieInts(void)
	{
		PIE_disableAllInts(this->pie);
	}
	void clearAllPieInts(void)
	{
		 PIE_clearAllInts(this->pie);
	}
	void clearAllPieFlags(void)
	{
		PIE_clearAllFlags(this->pie);
	}
	void setDefaultPieTable(void)
	{
		PIE_setDefaultIntVectorTable(this->pie);
	}
	PIE_Handle getPie(){return pie;};
	void setupTimer0(void);
	uint64_t getTicks();
private:
	OSC_Handle oscillator;
	WDOG_Handle watchdog;
	CLK_Handle clock;
	CPU_Handle centralProcessor;
	CAP_Handle capture;
	FLASH_Handle flash;
	PIE_Handle pie;
	PLL_Handle phaseLockLoop;
	TIMER_Handle timer0;
};


__interrupt void timer0Interrupt(void);
__interrupt void gpioCaptureInterrupt(void);

#endif /* HARDWARE_TMS320F2806_HH_ */
