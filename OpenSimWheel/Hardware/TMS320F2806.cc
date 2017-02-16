/*
 * TMS320F2806.cc
 *
 *  Created on: Feb 2, 2017
 *      Author: JohnTaylor
 */

#include "TMS320F2806.hh"

TMS320F2806::TMS320F2806(void)
{
	//copy fast-access functions to RAM
	memCopy((uint16_t *)&RamfuncsLoadStart,(uint16_t *)&RamfuncsLoadEnd,(uint16_t *)&RamfuncsRunStart);
	this->centralProcessor = CPU_init(&cpu,sizeof(cpu));
	this->clock = CLK_init((void *)CLK_BASE_ADDR,sizeof(CLK_Obj));
	this->flash = FLASH_init((void *)FLASH_BASE_ADDR,sizeof(FLASH_Obj));
	this->oscillator = OSC_init((void *)OSC_BASE_ADDR,sizeof(OSC_Obj));
	this->phaseLockLoop = PLL_init((void *)PLL_BASE_ADDR,sizeof(PLL_Obj));
	this->pie = PIE_init((void *)PIE_BASE_ADDR,sizeof(PIE_Obj));
	this->watchdog = WDOG_init((void *)WDOG_BASE_ADDR,sizeof(WDOG_Obj));
}

void TMS320F2806::setup(PLL_ClkFreq_e frequency)
{
	CPU_disableGlobalInts(this->centralProcessor);
	CPU_disableInts(this->centralProcessor);
	CPU_clearIntFlags(this->centralProcessor);

	WDOG_disable(this->watchdog);

	CLK_enableOsc1(this->clock);
	CLK_setOscSrc(this->clock,CLK_OscSrc_Internal);
	CLK_disableClkIn(this->clock);
	CLK_disableCrystalOsc(this->clock);
	CLK_disableOsc2(this->clock);
	CLK_setLowSpdPreScaler(this->clock,CLK_LowSpdPreScaler_SysClkOut_by_1);
	CLK_setClkOutPreScaler(this->clock,CLK_ClkOutPreScaler_SysClkOut_by_1);

	 // make sure PLL is not running in limp mode
    if(PLL_getClkStatus(this->phaseLockLoop) != PLL_ClkStatus_Normal)
    {
    	 // reset the clock detect
    	 PLL_resetClkDetect(this->phaseLockLoop);
    	 asm("        ESTOP0");
    }


    // Divide Select must be ClkIn/4 before the clock rate can be changed
    if(PLL_getDivideSelect(this->phaseLockLoop) != PLL_DivideSelect_ClkIn_by_4)
    {
    	PLL_setDivideSelect(this->phaseLockLoop,PLL_DivideSelect_ClkIn_by_4);
    }


    if(PLL_getClkFreq(this->phaseLockLoop) != frequency)
    {
    	 // disable the clock detect
    	 PLL_disableClkDetect(this->phaseLockLoop);

    	 // set the clock rate
    	 PLL_setClkFreq(this->phaseLockLoop,frequency);
    }


    // wait until locked
    while(PLL_getLockStatus(this->phaseLockLoop) != PLL_LockStatus_Done) {}


    // enable the clock detect
    PLL_enableClkDetect(this->phaseLockLoop);


   // set divide select to ClkIn/2 to get desired clock rate
   // NOTE: clock must be locked before setting this register
   PLL_setDivideSelect(this->phaseLockLoop,PLL_DivideSelect_ClkIn_by_2);


   // enable the ADC clock
   CLK_enableAdcClock(this->clock);


   // Run the Device_cal() function
   // This function copies the ADC and oscillator calibration values from TI reserved
   // OTP into the appropriate trim registers
   // This boot ROM automatically calls this function to calibrate the interal
   // oscillators and ADC with device specific calibration data.
   // If the boot ROM is bypassed by Code Composer Studio during the development process,
   // then the calibration must be initialized by the application
   ENABLE_PROTECTED_REGISTER_WRITE_MODE;
   (*Device_cal)();
   DISABLE_PROTECTED_REGISTER_WRITE_MODE;

   // disable the ADC clock
   CLK_disableAdcClock(this->clock);


   PIE_disable(this->pie);
   PIE_disableAllInts(this->pie);
   PIE_clearAllInts(this->pie);
   PIE_clearAllFlags(this->pie);

   PIE_setDefaultIntVectorTable(this->pie);
   PIE_enable(this->pie);

   FLASH_enablePipelineMode(this->flash);

   FLASH_setNumPagedReadWaitStates(this->flash,FLASH_NumPagedWaitStates_3);

   FLASH_setNumRandomReadWaitStates(this->flash,FLASH_NumRandomWaitStates_3);

   FLASH_setOtpWaitStates(this->flash,FLASH_NumOtpWaitStates_5);

   FLASH_setStandbyWaitCount(this->flash,FLASH_STANDBY_WAIT_COUNT_DEFAULT);

   FLASH_setActiveWaitCount(this->flash,FLASH_ACTIVE_WAIT_COUNT_DEFAULT);

   //DELAY_US(100);
   CPU_enableGlobalInts(this->centralProcessor);

}

void TMS320F2806::enableSCIBClock()
{
	CLK_enableScibClock(this->clock);
}
