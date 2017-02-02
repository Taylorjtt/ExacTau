/*
 * hal_obj.h
 *
 *  Created on: Mar 14, 2016
 *      Author: JohnTaylor
 */

#ifndef HARDWARE_ABSTRACTION_LAYER_HAL_OBJ_H_
#define HARDWARE_ABSTRACTION_LAYER_HAL_OBJ_H_

#include "../math.h"
#include "../common/DSP28x_Project.h"
#include "../f2806_headers/F2806x_SysCtrl.h"
#include "../Peripherial Drivers/types.h"
#include "../Peripherial Drivers/adc.h"
#include "../Peripherial Drivers/clk.h"
#include "../Peripherial Drivers/cpu.h"
#include "../Peripherial Drivers/gpio.h"
#include "../Peripherial Drivers/pie.h"
#include "../Peripherial Drivers/pll.h"
#include "../Peripherial Drivers/pwm.h"
#include "../Peripherial Drivers/timer.h"
#include "../Peripherial Drivers/wdog.h"
#include "../Peripherial Drivers/osc.h"
#include "../Peripherial Drivers/flash.h"
#include "../Peripherial Drivers/sci.h"
#include "../Peripherial Drivers/qep.h"
#include "../Peripherial Drivers/cap.h"
#include "../Peripherial Drivers/spi.h"
#define WIFI_RX_BUFFER_SIZE 50
#define LCD_RX_BUFFER_SIZE 40

typedef struct _HAL_OBJ_
{
	OSC_Handle 	  oscHandle;
	CLK_Handle    clkHandle;
	CPU_Handle    cpuHandle;
	GPIO_Handle   gpioHandle;
	PIE_Handle    pieHandle;
	PLL_Handle    pllHandle;
	PWM_Handle    pwmHandle[3];
	TIMER_Handle  timerHandle[3];
	WDOG_Handle   wdogHandle;
	FLASH_Handle flashHandle;
	SPI_Handle spiBHandle;
	SCI_Handle sciBHandle;
	SCI_Handle sciAHandle;
	CAP_Handle ecap1Handle;
	QEP_Handle qepAHandle;
	volatile Uint8 lcdRxBuffer[LCD_RX_BUFFER_SIZE];
	volatile int lcdRxBufferIndex;
	volatile bool lcdFirstByteRecieved;
	volatile Uint8 lcdLastRecievedByte;
	volatile bool lcdCommandRecieved;

	volatile Uint8 wifiRxBuffer[WIFI_RX_BUFFER_SIZE];
	volatile int wifiRxBufferIndex;
	volatile bool wifiFirstByteRecieved;
	volatile Uint8 wifiLastRecievedByte;
	volatile bool wifiCommandRecieved;
	volatile bool motorControlFlag;
	volatile bool stimulationFlag;
	volatile bool auxTimerFlag;
	volatile bool hasHitIndexPulse;
	volatile bool encoderWatchdogTimeout;
	volatile int watchdogTimoutDecimator;
	volatile int watchdogTimeoutDecimatorCount;

	long I2CResetCount;
	long I2CNackCount;
}HAL_Obj;

typedef struct _HAL_Obj_ *HAL_Handle;
extern HAL_Obj hal;

#endif /* HARDWARE_ABSTRACTION_LAYER_HAL_OBJ_H_ */
