/*
 * hal.h
 *
 *  Created on: Mar 11, 2016
 *      Author: JohnTaylor
 */

#ifndef TORQUETESTBEDEMBEDDED_HAL_H_
#define TORQUETESTBEDEMBEDDED_HAL_H_

#include "hal_obj.h"
#include "../Configuration.h"
#include <stdbool.h>
#define LEFT_I2C_ADDRESS 0x21
#define RIGHT_I2C_ADDRESS 0x20
#define RIGHT_QUAD_CHANNEL_NUMBER 0
#define RIGHT_HAM_CHANNEL_NUMBER 1
#define RIGHT_GLUTE_CHANNEL_NUMBER 2

#define LEFT_QUAD_CHANNEL_NUMBER 0
#define LEFT_HAM_CHANNEL_NUMBER 1
#define LEFT_GLUTE_CHANNEL_NUMBER 2
extern float actualErgometerCurrent;
extern float actualErgometerSpeed;

#define MOTOR_DUTY_TO_COUNTS 20.0

#define Device_cal (void   (*)(void))0x3D7C80
//! \brief Defines used in oscillator calibration functions
//! \brief Defines the scale factor for Q15 fixed point numbers (2^15)
#define FP_SCALE 32768

//! \brief Defines the quantity added to Q15 numbers before converting to integer to round the number
#define FP_ROUND FP_SCALE/2

//! \brief Defines the amount to add to Q16.15 fixed point number to shift from a fine trim range of
//! \brief (-31 to 31) to (1 to 63).  This guarantees that the trim is positive and can
//! \brief therefore be efficiently rounded
#define OSC_POSTRIM 32
#define OSC_POSTRIM_OFF FP_SCALE*OSC_POSTRIM

//! \brief The following functions return reference values stored in OTP.

//! \brief Defines the slope used to compensate oscillator 1 (fine trim steps / ADC code). Stored in fixed point Q15 format
#define getOsc1FineTrimSlope() (*(int16_t (*)(void))0x3D7E90)()

//! \brief Defines the oscillator 1 fine trim at high temp
#define getOsc1FineTrimOffset() (*(int16_t (*)(void))0x3D7E93)()

//! \brief Defines the oscillator 1 coarse trim
#define getOsc1CoarseTrim() (*(int16_t (*)(void))0x3D7E96)()

//! \brief Defines the slope used to compensate oscillator 2 (fine trim steps / ADC code). Stored
//! \brief in fixed point Q15 format.
#define getOsc2FineTrimSlope() (*(int16_t (*)(void))0x3D7E99)()

//! \brief Defines the oscillator 2 fine trim at high temp
#define getOsc2FineTrimOffset() (*(int16_t (*)(void))0x3D7E9C)()

//! \brief Defines the oscillator 2 coarse trim
#define getOsc2CoarseTrim() (*(int16_t (*)(void))0x3D7E9F)()

//! \brief Defines the ADC reading of temperature sensor at reference temperature for compensation
#define getRefTempOffset() (*(int16_t (*)(void))0x3D7EA2)()

//! \brief Defines the PWM deadband falling edge delay count (system clocks)
//!
#define HAL_PWM_DBFED_CNT         1


//! \brief Defines the PWM deadband rising edge delay count (system clocks)
//!
#define HAL_PWM_DBRED_CNT         1


extern HAL_Handle HAL_init(void *pMemory,const size_t numBytes);
void HAL_enableEncoderWatchdog(HAL_Handle halHandle, bool enable);
void HAL_disableWdog(HAL_Handle halHandle);
void HAL_setupProcessor(HAL_Handle handle);
void HAL_setupClks(HAL_Handle handle);
void HAL_setupPll(HAL_Handle handle,const PLL_ClkFreq_e clkFreq);
void HAL_setupPie(HAL_Handle handle);
void HAL_cal(HAL_Handle handle);
void HAL_setupFlash(HAL_Handle handle);
void HAL_osc1Comp(HAL_Handle handle, const int16_t sensorSample);
void HAL_osc2Comp(HAL_Handle handle, const int16_t sensorSample);
void HAL_setupGpios(HAL_Handle handle);
void HAL_setupPeripheralClks(HAL_Handle handle);
uint16_t HAL_getOscTrimValue(int16_t coarse, int16_t fine);

#endif
