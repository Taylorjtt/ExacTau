/*
 * TMS320F2806.cc
 *
 *  Created on: Feb 2, 2017
 *      Author: JohnTaylor
 */

#include "TMS320F2806.hh"
volatile uint64_t ticks = 0;
volatile float dutyCycle  = 0;

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
	this->timer0 = TIMER_init((void *)TIMER0_BASE_ADDR,sizeof(TIMER_Obj));
	this->capture = CAP_init((void *)CAP1_BASE_ADDR, sizeof(CAP_Obj));

}

void TMS320F2806::registerPIEInterruptHandler(PIE_GroupNumber_e groupNumber, const PIE_SubGroupNumber_e subGroupNumber, const PIE_IntVec_t vector)
{
	PIE_registerPieIntHandler(this->pie,groupNumber,subGroupNumber,vector);
}
void TMS320F2806::setupGPIOCapture()
{
	CPU_disableGlobalInts(centralProcessor);
	CAP_disableTimestampCounter(capture);
	CAP_disableInt(capture,CAP_Int_Type_All);
	CLK_enableEcap1Clock(clock);
	CAP_setModeCap(capture);

	//capture on falling edges
	CAP_setCapEvtPolarity(capture, CAP_Event_1,CAP_Polarity_Rising);
	CAP_setCapEvtPolarity(capture, CAP_Event_2,CAP_Polarity_Falling);
	CAP_setCapEvtPolarity(capture, CAP_Event_3,CAP_Polarity_Rising);
	CAP_setCapEvtPolarity(capture, CAP_Event_4,CAP_Polarity_Falling);

	//reset on every capture event (delta-t mode)
	CAP_setCapEvtReset(capture, CAP_Event_1,CAP_Reset_Enable);
	CAP_setCapEvtReset(capture, CAP_Event_2,CAP_Reset_Enable);
	CAP_setCapEvtReset(capture, CAP_Event_3,CAP_Reset_Enable);
	CAP_setCapEvtReset(capture, CAP_Event_4,CAP_Reset_Enable);
	CAP_disableSyncIn(capture);
	CAP_enableCaptureLoad(capture);
	CAP_setCapContinuous(capture);

	EALLOW;
	ECap1Regs.ECCTL1.bit.FREE_SOFT = 0b11;
	EDIS;
	CAP_enableInt(capture,CAP_Int_Type_CEVT1);
	CAP_clearInt(capture, CAP_Int_Type_All);
	CPU_enableInt(centralProcessor, CPU_IntNumber_4);
	PIE_registerPieIntHandler(pie,PIE_GroupNumber_4,PIE_SubGroupNumber_1,(PIE_IntVec_t) &gpioCaptureInterrupt);	//register the interrupt
	PIE_enableInt(pie,PIE_GroupNumber_4,PIE_InterruptSource_ECAP1);		//enable the interrupt in the PIE
	CAP_enableTimestampCounter(capture);
	CPU_enableGlobalInts(centralProcessor);
}
void TMS320F2806::enableCPUInterrupt(CPU_IntNumber_e number)
{
	CPU_enableInt(centralProcessor,number);
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

   DELAY_US(100);
   CPU_enableGlobalInts(this->centralProcessor);

}

void TMS320F2806::setupTimer0()
{

	CPU_enableInt(this->centralProcessor, CPU_IntNumber_1);
    CLK_enableCpuTimerClock(this->clock, CLK_CpuTimerNumber_0);
	PIE_enableTimer0Int(this->pie);
	registerPIEInterruptHandler(PIE_GroupNumber_1, PIE_SubGroupNumber_7, (PIE_IntVec_t) &timer0Interrupt);
	PIE_enableInt(this->pie, PIE_GroupNumber_1, PIE_InterruptSource_TIMER_0);
	TIMER_setPeriod(this->timer0, 800); //set up to interrupt at 100khz
	TIMER_enableInt(this->timer0);		//enable timer 0 interrupt
	TIMER_setEmulationMode(this->timer0, TIMER_EmulationMode_RunFree);
	TIMER_setPreScaler(this->timer0,0);
	TIMER_start(this->timer0);
}

void TMS320F2806::enableSCIBClock()
{
	CLK_enableScibClock(this->clock);
}
void TMS320F2806::enableEQEP1Clock()
{
	 CLK_enableEqep1Clock(this->clock);
}
void TMS320F2806::enablePWMClock(PWM_Number_e number)
{
	CLK_enablePwmClock(this->clock,number);

}
void TMS320F2806::enableSPIAClock()
{
	CLK_enableSpiaClock(this->clock);

}
void TMS320F2806::enableADCClock()
{
	CLK_enableAdcClock(clock);
}
void TMS320F2806::enableTbClockSync(bool enable)
{
	if(!enable)
	{
		CLK_disableTbClockSync(this->clock);
	}
	else
	{
		CLK_enableTbClockSync(this->clock);
	}

}
uint64_t TMS320F2806::getTicks()
{
	return ticks;
}
TMS320F2806::~TMS320F2806()
{

}
__interrupt void gpioCaptureInterrupt(void)
{
	float cap1 = ((80e6/ECap1Regs.CAP1));
	float cap2 = ((80e6/ECap1Regs.CAP2));
	float cap3 = ((80e6/ECap1Regs.CAP3));
	float cap4 = ((80e6/ECap1Regs.CAP4));
	dutyCycle = (cap1/(cap1 + cap2) + cap3/(cap3 + cap4))/2;
	EALLOW;
	ECap1Regs.ECCLR.all |= CAP_Int_Type_CEVT1;
	ECap1Regs.ECCLR.all |= CAP_Int_Type_Global;
	PieCtrlRegs.PIEACK.bit.ACK4 = 1;
	EDIS;
}
__interrupt void timer0Interrupt(void)
{

	//GpioDataRegs.GPBTOGGLE.bit.GPIO33 = 1;

	ticks++;
	EALLOW;

	CpuTimer0Regs.TCR.bit.TIF = 1;
	PieCtrlRegs.PIEACK.bit.ACK1 = 1;
	EDIS;
}
