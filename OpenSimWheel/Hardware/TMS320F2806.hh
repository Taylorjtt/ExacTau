/*
 * TMS320f2806.hh
 *
 *  Created on: Feb 2, 2017
 *      Author: JohnTaylor
 */

#ifndef HARDWARE_TMS320F2806_HH_
#define HARDWARE_TMS320F2806_HH_
#include "../common/DSP28x_Project.h"
#include "../Util/memCopy.h"
#include "../Peripherial Drivers/osc.h"
#include "../Peripherial Drivers/wdog.h"
#include "../Peripherial Drivers/clk.h"
#include "../Peripherial Drivers/cpu.h"
#include "../Peripherial Drivers/flash.h"
#include "../Peripherial Drivers/pie.h"
#include "../Peripherial Drivers/pll.h"

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

class TMS320F2806
{
public:
	TMS320F2806();
	~TMS320F2806();
	void setup(PLL_ClkFreq_e frequency);
	void enableSCIBClock();
	PIE_Handle getPie(){return pie;};
private:
	OSC_Handle oscillator;
	WDOG_Handle watchdog;
	CLK_Handle clock;
	CPU_Handle centralProcessor;
	FLASH_Handle flash;
	PIE_Handle pie;
	PLL_Handle phaseLockLoop;
};




#endif /* HARDWARE_TMS320F2806_HH_ */
