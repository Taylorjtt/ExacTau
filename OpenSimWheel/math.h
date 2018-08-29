/* --COPYRIGHT--,BSD
 * Copyright (c) 2012, Texas Instruments Incorporated
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
#ifndef _MATH_H_
#define _MATH_H_

//! \file   modules/math/src/float/math.h
//! \brief  Contains the public interface to the 
//!         math (MATH) module routines
//!
//! (C) Copyright 2011, Texas Instruments, Inc.


// **************************************************************************
// the includes

#include "Peripherial Drivers/types.h"

//!
//!
//! \defgroup MATH MATH
//!
//@{

// Include the algorithm overview defined in modules/<module>/docs/doxygen/doxygen.h
//! \defgroup MATH_OVERVIEW 


#ifdef __cplusplus
extern "C" {
#endif


// **************************************************************************
// the defines

//! \brief Defines conversion scale factor from N*m to lb*in
//!
#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))
#define MATH_Nm_TO_lbin_SF        ((float_t)(8.8507457913))

//! \brief Defines 2/3
//!
#define MATH_TWO_OVER_THREE       ((float_t)(0.66666666666666666666666666666667))

//! \brief Defines 1/3
//!
#define MATH_ONE_OVER_THREE       ((float_t)(0.33333333333333333333333333333333))

//! \brief Defines 1/(pi)
//!
#define MATH_ONE_OVER_PI          ((float_t)(0.318309886183791))

//! \brief Defines 1/sqrt(3)
//!
#define MATH_ONE_OVER_SQRT_THREE  ((float_t)(0.57735026918962576450914878050196))

//! \brief Defines 1/(4*pi)
//!
#define MATH_ONE_OVER_FOUR_PI     ((float_t)(0.07957747154594767))

//! \brief Defines 1/(2*pi)
//!
#define MATH_ONE_OVER_TWO_PI     ((float_t) (0.1591549430918954))

//! \brief Defines pi
//!
#define	MATH_PI                   ((float_t)(3.1415926535897932384626433832795))

//! \brief Defines pi per unit
//!
#define	MATH_PI_PU                ((float_t)(0.5))

//! \brief Defines 2*pi
//!
#define	MATH_TWO_PI               ((float_t)(6.283185307179586))

//! \brief Defines 2*pi per unit 
//!
#define	MATH_TWO_PI_PU            ((float_t)(1.0)

//! \brief Defines 4*pi
//!
#define	MATH_FOUR_PI               ((float_t)(12.56637061435917)

//! \brief Defines 4*pi per unit
//!
#define	MATH_FOUR_PI_PU            ((float_t)(2.0))

//! \brief Defines pi/2
//!
#define	MATH_PI_OVER_TWO           ((float_t)(1.570796326794897))

//! \brief Defines pi/2 per unit
//!
#define	MATH_PI_OVER_TWO_PU        ((float_t)(0.25))

//! \brief Defines pi/4
//!
#define	MATH_PI_OVER_FOUR          ((float_t)(0.785398163397448))

//! \brief Defines pi/4 per unit
//!
#define	MATH_PI_OVER_FOUR_PU        ((float_t)(0.125)))


// **************************************************************************
// the typedefs

//! \brief Defines a two element vector
//!
typedef struct _MATH_vec2_
{

  float_t  value[2];

} MATH_vec2;


//! \brief Defines a three element vector
//!
typedef struct _MATH_vec3_
{

  float_t  value[3];

} MATH_vec3;
static int modulo(int x,int N){
    return (x % N + N) %N;
}

// **************************************************************************
// the function prototypes


//! \brief     Finds the absolute value 
//! \param[in] in   The input value
//! \return    The absolute value

static inline float_t MATH_abs(const float_t in)
{
  float_t out = in;


  if(in < 0.0)
    {
      out = -in;
    }

  return(out);
} // end of MATH_abs() function


//! \brief     Increments an angle value and handles wrap-around
//! \param[in] angle_rad       The angle value, rad
//! \param[in] angleDelta_rad  The angle increment value, rad
//! \return    The incremented angle value, rad
static inline float_t MATH_incrAngle(const float_t angle_rad,const float_t angleDelta_rad)
{
  float_t angleNew_rad;


  // increment the angle
  angleNew_rad = angle_rad + angleDelta_rad;
     

  // check for limits
  if(angleNew_rad > MATH_PI)
    {
      angleNew_rad -= MATH_TWO_PI;
    }
  else if(angleNew_rad < (-MATH_PI))
    {
      angleNew_rad += MATH_TWO_PI;
    }

  return(angleNew_rad);
} // end of MATH_incrAngle() function


//! \brief     Saturates the input value between the minimum and maximum values
//! \param[in] in   The input value
//! \param[in] max  The maximum value allowed
//! \param[in] min  The minimum value allowed
//! \return    The saturated value
static inline float_t MATH_sat(const float_t in,const float_t max,const float_t min)
{
  float_t out = in;


  if(in < min)
    {
      out = min;
    }
  else if(in > max)
    {
      out = max;
    }

  return(out);
} // end of MATH_sat() function


#ifdef __cplusplus
}
#endif // extern "C"

//@} // ingroup
#endif // end of _MATH_H_ definition


