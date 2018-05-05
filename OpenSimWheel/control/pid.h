/* --COPYRIGHT--,BSD
 * Copyright (c) 2014, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
#ifndef _PID_H_
#define _PID_H_

//! \file   modules/pid/src/float/pid.h
//! \brief  Contains the public interface to the 
//!         Proportional-Integral-Derivative (PID) controller module routines
//!
//! (C) Copyright 2014, Texas Instruments, Inc.


// **************************************************************************
// the includes

// modules
#include "../Peripherial Drivers/types.h"
#include "filter_fo.h"
#include "../math.h"


//!
//!
//! \defgroup PID PID
//!
//@{

// Include the algorithm overview defined in modules/<module>/docs/doxygen/doxygen.h
//! \defgroup PID_OVERVIEW 

#ifdef __cplusplus
extern "C" {
#endif


// **************************************************************************
// the defines



// **************************************************************************
// the typedefs

//! \brief Defines the PID controller object
//!
typedef struct _PID_Obj_
{
  float_t          Kp;                //!< the proportional gain for the PID controller
  float_t          Ki;                //!< the integral gain for the PID controller
  float_t          Kd;                //!< the derivative gain for the PID controller

  float_t          Ui;                //!< the integrator start value for the PID controller

  float_t          refValue;          //!< the reference input value
  float_t          fbackValue;        //!< the feedback input value
  float_t          ffwdValue;         //!< the feedforward input value
  float_t 		   error;
		
  float_t          outMin;            //!< the minimum output value allowed for the PID controller
  float_t          outMax;            //!< the maximum output value allowed for the PID controller

  float_t		   uiOutMax;
  float_t		   uiOutMin;

  FILTER_FO_Handle derFilterHandle;   //!< the derivative filter handle
  FILTER_FO_Obj    derFilter;         //!< the derivative filter object
} PID_Obj; 


//! \brief Defines the PID handle
//!
typedef struct _PID_Obj_ *PID_Handle;


// **************************************************************************
// the function prototypes

//! \brief     Gets the derivative filter parameters
//!
//!            y[n] = b0*x[n] + b1*x[n-1] - a1*y[n-1]
//!
//! \param[in] handle  The PID controller handle
//! \param[in] b0      The numerator filter coefficient value for z^0
//! \param[in] b1      The numerator filter coefficient value for z^(-1)
//! \param[in] a1      The denominator filter coefficient value for z^(-1)
//! \param[in] x1      The input value at time sample n=-1
//! \param[in] y1      The output value at time sample n=-1
void PID_getDerFilterParams(PID_Handle handle,
                            float_t *b0,float_t *b1,
                            float_t *a1,float_t *x1,float_t *y1);


//! \brief     Gets the feedback value in the PID controller
//! \param[in] handle  The PID controller handle
//! \return    The feedback value in the PID controller
static inline float_t PID_getFbackValue(PID_Handle handle)
{
  PID_Obj *obj = (PID_Obj *)handle;

  return(obj->fbackValue);
} // end of PID_getFbackValue() function


//! \brief     Gets the feedforward value in the PID controller
//! \param[in] handle  The PID controller handle
//! \return    The feedforward value in the PID controller
static inline float_t PID_getFfwdValue(PID_Handle handle)
{
  PID_Obj *obj = (PID_Obj *)handle;

  return(obj->ffwdValue);
} // end of PID_getFfwdValue() function


//! \brief      Gets the gains in the PID controller
//! \param[in]  handle  The PID controller handle
//! \param[out] pKp     The pointer to the proportional gain value
//! \param[out] pKi     The pointer to the integrator gain value
//! \param[out] pKd     The pointer to the derivative gain value
static inline void PID_getGains(PID_Handle handle,float_t *pKp,float_t *pKi,float_t *pKd)
{
  PID_Obj *obj = (PID_Obj *)handle;

  *pKp = obj->Kp;
  *pKi = obj->Ki;
  *pKd = obj->Kd;

  return;
} // end of PID_getGains() function


//! \brief     Gets the derivative gain in the PID controller
//! \param[in] handle  The PID controller handle
//! \return    The derivative gain in the PID controller
static inline float_t PID_getKd(PID_Handle handle)
{
  PID_Obj *obj = (PID_Obj *)handle;

  return(obj->Kd);
} // end of PID_getKd() function
static inline float_t PID_getError(PID_Handle handle)
{
  PID_Obj *obj = (PID_Obj *)handle;

  return(obj->error);
}

//! \brief     Gets the integral gain in the PID controller
//! \param[in] handle  The PID controller handle
//! \return    The integral gain in the PID controller
static inline float_t PID_getKi(PID_Handle handle)
{
  PID_Obj *obj = (PID_Obj *)handle;

  return(obj->Ki);
} // end of PID_getKi() function


//! \brief     Gets the proportional gain in the PID controller
//! \param[in] handle  The PID controller handle
//! \return    The proportional gain in the PID controller
static inline float_t PID_getKp(PID_Handle handle)
{
  PID_Obj *obj = (PID_Obj *)handle;

  return(obj->Kp);
} // end of PID_getKp() function


//! \brief      Gets the minimum and maximum output value allowed in the PID controller
//! \param[in]  handle  The PID controller handle
//! \param[out] pOutMin    The pointer to the minimum output value allowed
//! \param[out] pOutMax    The pointer to the maximum output value allowed
static inline void PID_getMinMax(PID_Handle handle,float_t *pOutMin,float_t *pOutMax)
{
  PID_Obj *obj = (PID_Obj *)handle;

  *pOutMin = obj->outMin;
  *pOutMax = obj->outMax;

  return;
} // end of PID_getMinMax() function


//! \brief      Gets the maximum output value allowed in the PID controller
//! \param[in]  handle  The PID controller handle
//! \return     The maximum output value allowed
static inline float_t PID_getOutMax(PID_Handle handle)
{
  PID_Obj *obj = (PID_Obj *)handle;

  return(obj->outMax);
} // end of PID_getOutMax() function


//! \brief      Gets the minimum output value allowed in the PID controller
//! \param[in]  handle  The PID controller handle
//! \return     The minimum output value allowed
static inline float_t PID_getOutMin(PID_Handle handle)
{
  PID_Obj *obj = (PID_Obj *)handle;

  return(obj->outMin);
} // end of PID_getOutMin() function


//! \brief     Gets the reference value in the PID controller
//! \param[in] handle  The PID controller handle
//! \return    The reference value in the PID controller
static inline float_t PID_getRefValue(PID_Handle handle)
{
  PID_Obj *obj = (PID_Obj *)handle;

  return(obj->refValue);
} // end of PID_getRefValue() function


//! \brief     Gets the integrator start value in the PID controller
//! \param[in] handle  The PID controller handle
//! \return    The integrator start value for the PID controller
static inline float_t PID_getUi(PID_Handle handle)
{
  PID_Obj *obj = (PID_Obj *)handle;

  return(obj->Ui);
} // end of PID_getUi() function


//! \brief     Initializes the PID controller
//! \param[in] pMemory   A pointer to the memory for the PID controller object
//! \param[in] numBytes  The number of bytes allocated for the PID controller object, bytes
//! \return The PID controller (PID) object handle
extern PID_Handle PID_init(void *pMemory,const size_t numBytes);


//! \brief     Sets the derivative filter parameters
//!
//!            y[n] = b0*x[n] + b1*x[n-1] - a1*y[n-1]
//!
//! \param[in] handle  The PID controller handle
//! \param[in] b0      The numerator filter coefficient value for z^0
//! \param[in] b1      The numerator filter coefficient value for z^(-1)
//! \param[in] a1      The denominator filter coefficient value for z^(-1)
//! \param[in] x1      The input value at time sample n=-1
//! \param[in] y1      The output value at time sample n=-1
void PID_setDerFilterParams(PID_Handle handle,
                            const float_t b0,const float_t b1,
                            const float_t a1,const float_t x1,const float_t y1);


//! \brief     Sets the feedback value in the PID controller
//! \param[in] handle      The PID controller handle
//! \param[in] fbackValue  The feedback value
static inline void PID_setFbackValue(PID_Handle handle,const float_t fbackValue)
{
  PID_Obj *obj = (PID_Obj *)handle;

  obj->fbackValue = fbackValue;

  return;
} // end of PID_setFbackValue() function


//! \brief     Sets the feedforward value in the PID controller
//! \param[in] handle     The PID controller handle
//! \param[in] ffwdValue  The feedforward value
static inline void PID_setFfwdValue(PID_Handle handle,const float_t ffwdValue)
{
  PID_Obj *obj = (PID_Obj *)handle;

  obj->ffwdValue = ffwdValue;

  return;
} // end of PID_setFfwdValue() function


//! \brief     Sets the gains in the PID controller
//! \param[in] handle  The PID controller handle
//! \param[in] Kp      The proportional gain for the PID controller
//! \param[in] Ki      The integrator gain for the PID controller
//! \param[in] Kd      The derivative gain for the PID controller
static inline void PID_setGains(PID_Handle handle,const float_t Kp,const float_t Ki, const float_t Kd)
{
  PID_Obj *obj = (PID_Obj *)handle;

  obj->Kp = Kp;
  obj->Ki = Ki;
  obj->Kd = Kd;

  return;
} // end of PID_setGains() function


//! \brief     Sets the derivative gain in the PID controller
//! \param[in] handle  The PID controller handle
//! \param[in] Kd      The derivative gain for the PID controller
static inline void PID_setKd(PID_Handle handle,const float_t Kd)
{
  PID_Obj *obj = (PID_Obj *)handle;

  obj->Kd = Kd;

  return;
} // end of PID_setKd() function


//! \brief     Sets the integral gain in the PID controller
//! \param[in] handle  The PID controller handle
//! \param[in] Ki      The integral gain for the PID controller
static inline void PID_setKi(PID_Handle handle,const float_t Ki)
{
  PID_Obj *obj = (PID_Obj *)handle;

  obj->Ki = Ki;

  return;
} // end of PID_setKi() function


//! \brief     Sets the proportional gain in the PID controller
//! \param[in] handle  The PID controller handle
//! \param[in] Kp      The proportional gain for the PID controller
static inline void PID_setKp(PID_Handle handle,const float_t Kp)
{
  PID_Obj *obj = (PID_Obj *)handle;

  obj->Kp = Kp;

  return;
} // end of PID_setKp() function


//! \brief     Sets the minimum and maximum output value allowed in the PID controller
//! \param[in] handle  The PID controller handle
//! \param[in] outMin  The minimum output value allowed
//! \param[in] outMax  The maximum output value allowed
static inline void PID_setMinMax(PID_Handle handle,const float_t outMin,const float_t outMax)
{
  PID_Obj *obj = (PID_Obj *)handle;

  obj->outMin = outMin;
  obj->outMax = outMax;

  return;
} // end of PID_setMinMax() function

static inline void PID_setUIMinMax(PID_Handle handle,const float_t outMin,const float_t outMax)
{
  PID_Obj *obj = (PID_Obj *)handle;

  obj->uiOutMin = outMin;
  obj->uiOutMax = outMax;

  return;
} // end of PID_setMinMax() function


//! \brief     Sets the maximum output value allowed in the PID controller
//! \param[in] handle  The PID controller handle
//! \param[in] outMax  The maximum output value allowed
static inline void PID_setOutMax(PID_Handle handle,const float_t outMax)
{
  PID_Obj *obj = (PID_Obj *)handle;

  obj->outMax = outMax;

  return;
} // end of PID_setOutMax() function


//! \brief     Sets the minimum output value allowed in the PID controller
//! \param[in] handle  The PID controller handle
//! \param[in] outMax  The minimum output value allowed
static inline void PID_setOutMin(PID_Handle handle,const float_t outMin)
{
  PID_Obj *obj = (PID_Obj *)handle;

  obj->outMin = outMin;

  return;
} // end of PID_setOutMin() function


//! \brief     Sets the reference value in the PID controller
//! \param[in] handle  The PID controller handle
//! \param[in] refValue   The reference value
static inline void PID_setRefValue(PID_Handle handle,const float_t refValue)
{
  PID_Obj *obj = (PID_Obj *)handle;

  obj->refValue = refValue;

  return;
} // end of PID_setRefValue() function


//! \brief     Sets the integrator start value in the PID controller
//! \param[in] handle  The PID controller handle
//! \param[in] Ui         The integral start value for the PID controller
static inline void PID_setUi(PID_Handle handle,const float_t Ui)
{
  PID_Obj *obj = (PID_Obj *)handle;

  obj->Ui = Ui;

  return;
} // end of PID_setUi() function


//! \brief     Runs the parallel form of the PID controller
//! \param[in] handle      The PID controller handle
//! \param[in] refValue    The reference value to the controller
//! \param[in] fbackValue  The feedback value to the controller
//! \param[in] ffwdValue   The feedforward value to the controller
//! \param[in] pOutValue   The pointer to the controller output value
static inline void PID_run_parallel(PID_Handle handle,const float_t refValue,const float_t fbackValue,
                                    const float_t ffwdValue,float_t *pOutValue)
{
  PID_Obj *obj = (PID_Obj *)handle;


  float_t Kp = PID_getKp(handle);
  float_t Ki = PID_getKi(handle);
  float_t Kd = PID_getKd(handle);
  float_t Up;
  float_t Ui = PID_getUi(handle);
  float_t Ud_tmp,Ud;
  float_t outMax = PID_getOutMax(handle);
  float_t outMin = PID_getOutMin(handle);

  obj->error = refValue - fbackValue;

  Up = Kp * obj->error;                                    			 // Compute the proportional output
  Ui = MATH_sat(Ui + (Ki * obj->error),obj->uiOutMax,obj->uiOutMin);      // Compute the integral output

  Ud_tmp = Kd * obj->error;                                 			// Compute the derivative term
  Ud = FILTER_FO_run(obj->derFilterHandle,Ud_tmp);

  PID_setUi(handle,Ui);
  PID_setRefValue(handle,refValue);
  PID_setFbackValue(handle,fbackValue);
  PID_setFfwdValue(handle,ffwdValue);

  *pOutValue = MATH_sat(Up + Ui + Ud + ffwdValue,outMax,outMin);  // Saturate the output

  return;
} // end of PID_run_parallel() function


//! \brief     Runs the series form of the PID controller
//! \param[in] handle      The PID controller handle
//! \param[in] refValue    The reference value to the controller
//! \param[in] fbackValue  The feedback value to the controller
//! \param[in] ffwdValue   The feedforward value to the controller
//! \param[in] pOutValue   The pointer to the controller output value
static inline void PID_run_series(PID_Handle handle,const float_t refValue,const float_t fbackValue,
                                  const float_t ffwdValue,float_t *pOutValue)
{
  PID_Obj *obj = (PID_Obj *)handle;


  float_t Kp = PID_getKp(handle);
  float_t Ki = PID_getKi(handle);
  float_t Kd = PID_getKd(handle);
  float_t Up;
  float_t Ui = PID_getUi(handle);
  float_t Ud_tmp,Ud;
  float_t outMax = PID_getOutMax(handle);
  float_t outMin = PID_getOutMin(handle);


  obj->error = refValue - fbackValue;

  Up = Kp * obj->error;                                     // Compute the proportional output
  Ui = MATH_sat(Ui + (Ki * Up),outMax,outMin);         // Compute the integral output with saturation

  Ud_tmp = Kd * Ui;                                    // Compute the derivative term
  Ud = FILTER_FO_run(obj->derFilterHandle,Ud_tmp);

  PID_setUi(handle,Ui);
  PID_setRefValue(handle,refValue);
  PID_setFbackValue(handle,fbackValue);
  PID_setFfwdValue(handle,ffwdValue);

  *pOutValue = MATH_sat(Up + Ui + Ud + ffwdValue,outMax,outMin);  // Saturate the output

  return;
} // end of PID_run_series() function


#ifdef __cplusplus
}
#endif // extern "C"

//@} // ingroup
#endif //end of _PID_H_ definition

