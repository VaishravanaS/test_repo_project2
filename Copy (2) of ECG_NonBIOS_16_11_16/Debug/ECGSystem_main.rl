L 1 "../src/ECGSystem_main.c"
N /******************************************************************************
N**File Name			: ECGSystem_main.c
N**File Description	:
N**Created By		:
N**Creation Date		:
N**Last modified date:
N**Remarks			:
N*******************************************************************************/
N#include "Respiration_Module.h"
L 1 "../inc/Respiration_Module.h" 1
N /******************************************************************************
N**File Name			: Respiration_Module.h
N**File Description	:
N**Author    		: Vuon1
N**Creation Date		: 05-Aug-2016
N**Last modified date:
N**Remarks			:
N*******************************************************************************/
N
N#ifndef INC_RESPIRATION_MODULE_H_
N#define INC_RESPIRATION_MODULE_H_
N
N
N#define BUFFER			10
N#define ACCURACY 		10
N
Nint Calculate_Respiration(long LPF_2_hz_out1);
N
N#endif /* INC_RESPIRATION_MODULE_H_ */
L 10 "../src/ECGSystem_main.c" 2
N#include "ECGDemoNonBios.h"     
L 1 "../inc/ECGDemoNonBios.h" 1
N/******************************************************************************
N**File Name			: ECGDemoNonBios.h
N**File Description	:Definitions used for ECGSystem_main, common variables, buffer and function declaration
N**Created By		:
N**Creation Date		:
N**Last modified date:
N**Remarks			:
N*******************************************************************************/
N
N#ifndef _ECGDEMONONBIOS_H
N#define _ECGDEMONONBIOS_H
N
N
N#include<math.h>
L 1 "C:/ti/ccsv6/tools/compiler/c5500_4.4.1/include/math.h" 1
N/****************************************************************************/
N/*  math.h           v4.4.1                                                 */
N/*                                                                          */
N/* Copyright (c) 1997-2012 Texas Instruments Incorporated                   */
N/* http://www.ti.com/                                                       */
N/*                                                                          */
N/*  Redistribution and  use in source  and binary forms, with  or without   */
N/*  modification,  are permitted provided  that the  following conditions   */
N/*  are met:                                                                */
N/*                                                                          */
N/*     Redistributions  of source  code must  retain the  above copyright   */
N/*     notice, this list of conditions and the following disclaimer.        */
N/*                                                                          */
N/*     Redistributions in binary form  must reproduce the above copyright   */
N/*     notice, this  list of conditions  and the following  disclaimer in   */
N/*     the  documentation  and/or   other  materials  provided  with  the   */
N/*     distribution.                                                        */
N/*                                                                          */
N/*     Neither the  name of Texas Instruments Incorporated  nor the names   */
N/*     of its  contributors may  be used to  endorse or  promote products   */
N/*     derived  from   this  software  without   specific  prior  written   */
N/*     permission.                                                          */
N/*                                                                          */
N/*  THIS SOFTWARE  IS PROVIDED BY THE COPYRIGHT  HOLDERS AND CONTRIBUTORS   */
N/*  "AS IS"  AND ANY  EXPRESS OR IMPLIED  WARRANTIES, INCLUDING,  BUT NOT   */
N/*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR   */
N/*  A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT   */
N/*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,   */
N/*  SPECIAL,  EXEMPLARY,  OR CONSEQUENTIAL  DAMAGES  (INCLUDING, BUT  NOT   */
N/*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,   */
N/*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY   */
N/*  THEORY OF  LIABILITY, WHETHER IN CONTRACT, STRICT  LIABILITY, OR TORT   */
N/*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE   */
N/*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.    */
N/*                                                                          */
N/****************************************************************************/
N
N#ifndef _TI_ENHANCED_MATH_H
N#define _TI_ENHANCED_MATH_H
N#endif
N
N#ifndef __math__
N#define __math__
N
N#ifndef EDOM
N#define EDOM   1
N#endif
N
N#ifndef ERANGE
N#define ERANGE 2
N#endif
N
N#include <float.h>
L 1 "C:/ti/ccsv6/tools/compiler/c5500_4.4.1/include/float.h" 1
N/********************************************************************/
N/* float.h  v4.4.1                                                  */
N/*                                                                  */
N/* Copyright (c) 1996-2012 Texas Instruments Incorporated           */
N/* http://www.ti.com/                                               */
N/*                                                                  */
N/*  Redistribution and  use in source  and binary forms, with  or without */
N/*  modification,  are permitted provided  that the  following conditions */
N/*  are met:                                                        */
N/*                                                                  */
N/*     Redistributions  of source  code must  retain the  above copyright */
N/*     notice, this list of conditions and the following disclaimer. */
N/*                                                                  */
N/*     Redistributions in binary form  must reproduce the above copyright */
N/*     notice, this  list of conditions  and the following  disclaimer in */
N/*     the  documentation  and/or   other  materials  provided  with  the */
N/*     distribution.                                                */
N/*                                                                  */
N/*     Neither the  name of Texas Instruments Incorporated  nor the names */
N/*     of its  contributors may  be used to  endorse or  promote products */
N/*     derived  from   this  software  without   specific  prior  written */
N/*     permission.                                                  */
N/*                                                                  */
N/*  THIS SOFTWARE  IS PROVIDED BY THE COPYRIGHT  HOLDERS AND CONTRIBUTORS */
N/*  "AS IS"  AND ANY  EXPRESS OR IMPLIED  WARRANTIES, INCLUDING,  BUT NOT */
N/*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR */
N/*  A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT */
N/*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, */
N/*  SPECIAL,  EXEMPLARY,  OR CONSEQUENTIAL  DAMAGES  (INCLUDING, BUT  NOT */
N/*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, */
N/*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY */
N/*  THEORY OF  LIABILITY, WHETHER IN CONTRACT, STRICT  LIABILITY, OR TORT */
N/*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE */
N/*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */
N/*                                                                  */
N/*    KEY:  FLT_     - APPLIES TO TYPE FLOAT                        */
N/*          DBL_     - APPLIES TO TYPE DOUBLE                       */
N/*          LDBL_    - APPLIES TO TYPE LONG DOUBLE                  */
N/********************************************************************/
N#ifndef _FLOAT
N#define _FLOAT
N
N#define FLT_RADIX                         2   /* RADIX OF EXPONENT         */
N#define FLT_ROUNDS                        1   /* ROUND TO NEAREST          */
N
N#define FLT_DIG                           6   /* DECIMAL PRECISION         */
N#define FLT_MANT_DIG                     24   /* BITS IN MANTISSA          */
N#define FLT_MIN_EXP                    -125   /* SMALLEST EXPONENT         */
N#define FLT_MAX_EXP                     128   /* LARGEST EXPONENT          */
N#define FLT_EPSILON        1.192092896E-07F   /* SMALLEST X WHERE 1+X != 1 */
N#define FLT_MIN            1.175494351E-38F   /* SMALLEST POSITIVE VALUE   */
N#define FLT_MAX            3.402823466E+38F   /* LARGEST POSITIVE VALUE    */
N#define FLT_MIN_10_EXP                  -37   /* MIN POWER OF 10           */
N#define FLT_MAX_10_EXP                   38   /* MAX POWER OF 10           */
N
N#define DBL_DIG                           6   /* DECIMAL PRECISION         */
N#define DBL_MANT_DIG                     24   /* BITS IN MANTISSA          */
N#define DBL_MIN_EXP                    -125   /* SMALLEST EXPONENT         */
N#define DBL_MAX_EXP                     128   /* LARGEST EXPONENT          */
N#define DBL_EPSILON        1.192092896E-07F   /* SMALLEST X WHERE 1+X != 1 */
N#define DBL_MIN            1.175494351E-38F   /* SMALLEST POSITIVE VALUE   */
N#define DBL_MAX            3.402823466E+38F   /* LARGEST POSITIVE VALUE    */
N#define DBL_MIN_10_EXP                  -37   /* MIN POWER OF 10           */
N#define DBL_MAX_10_EXP                   38   /* MAX POWER OF 10           */
N
N#define LDBL_DIG                          6   /* DECIMAL PRECISION         */
N#define LDBL_MANT_DIG                    24   /* BITS IN MANTISSA          */
N#define LDBL_MIN_EXP                   -125   /* SMALLEST EXPONENT         */
N#define LDBL_MAX_EXP                    128   /* LARGEST EXPONENT          */
N#define LDBL_EPSILON       1.192092896E-07F   /* SMALLEST X WHERE 1+X != 1 */
N#define LDBL_MIN           1.175494351E-38F   /* SMALLEST POSITIVE VALUE   */
N#define LDBL_MAX           3.402823466E+38F   /* LARGEST POSITIVE VALUE    */
N#define LDBL_MIN_10_EXP                 -37   /* MIN POWER OF 10           */
N#define LDBL_MAX_10_EXP                  38   /* MAX POWER OF 10           */
N
N#endif
L 54 "C:/ti/ccsv6/tools/compiler/c5500_4.4.1/include/math.h" 2
N#define HUGE_VAL DBL_MAX
N#define HUGE_VALL LDBL_MAX
N
N#include <access.h>
L 1 "C:/ti/ccsv6/tools/compiler/c5500_4.4.1/include/access.h" 1
N/****************************************************************************/
N/*  access.h         v4.4.1                                                 */
N/*                                                                          */
N/* Copyright (c) 1997-2012 Texas Instruments Incorporated                   */
N/* http://www.ti.com/                                                       */
N/*                                                                          */
N/*  Redistribution and  use in source  and binary forms, with  or without   */
N/*  modification,  are permitted provided  that the  following conditions   */
N/*  are met:                                                                */
N/*                                                                          */
N/*     Redistributions  of source  code must  retain the  above copyright   */
N/*     notice, this list of conditions and the following disclaimer.        */
N/*                                                                          */
N/*     Redistributions in binary form  must reproduce the above copyright   */
N/*     notice, this  list of conditions  and the following  disclaimer in   */
N/*     the  documentation  and/or   other  materials  provided  with  the   */
N/*     distribution.                                                        */
N/*                                                                          */
N/*     Neither the  name of Texas Instruments Incorporated  nor the names   */
N/*     of its  contributors may  be used to  endorse or  promote products   */
N/*     derived  from   this  software  without   specific  prior  written   */
N/*     permission.                                                          */
N/*                                                                          */
N/*  THIS SOFTWARE  IS PROVIDED BY THE COPYRIGHT  HOLDERS AND CONTRIBUTORS   */
N/*  "AS IS"  AND ANY  EXPRESS OR IMPLIED  WARRANTIES, INCLUDING,  BUT NOT   */
N/*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR   */
N/*  A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT   */
N/*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,   */
N/*  SPECIAL,  EXEMPLARY,  OR CONSEQUENTIAL  DAMAGES  (INCLUDING, BUT  NOT   */
N/*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,   */
N/*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY   */
N/*  THEORY OF  LIABILITY, WHETHER IN CONTRACT, STRICT  LIABILITY, OR TORT   */
N/*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE   */
N/*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.    */
N/*                                                                          */
N/****************************************************************************/
N
N#ifndef __EXTERN
N
N   #undef __EXTERN
N   #undef __INLINE
N   #undef __STATIC
N
N      #undef _CODE_ACCESS
N      #undef _DATA_ACCESS
N
N      #if defined(_FAR_RTS)
X      #if 0L
S         #define __EXTERN __far extern
S         #define __STATIC static __far
S         #define _CODE_ACCESS __far
S         #define _DATA_ACCESS __far
S 
N      #else
N         #ifdef __cplusplus
S           #define __EXTERN extern "C"
N         #else
N           #define __EXTERN extern
N         #endif
N         #define __STATIC static
N         #define _CODE_ACCESS
N         #define _DATA_ACCESS
N      #endif
N
N      #if defined(_INLINE)
X      #if 0L
S         #define __INLINE static __inline
N      #else
N         #define __INLINE __EXTERN
N      #endif
N#endif
N
N
L 58 "C:/ti/ccsv6/tools/compiler/c5500_4.4.1/include/math.h" 2
N#include <elfnames.h>
L 1 "C:/ti/ccsv6/tools/compiler/c5500_4.4.1/include/elfnames.h" 1
N/****************************************************************************/
N/*  elfnames.h          v4.4.1                                              */
N/*                                                                          */
N/* Copyright (c) 1997-2012 Texas Instruments Incorporated                   */
N/* http://www.ti.com/                                                       */
N/*                                                                          */
N/*  Redistribution and  use in source  and binary forms, with  or without   */
N/*  modification,  are permitted provided  that the  following conditions   */
N/*  are met:                                                                */
N/*                                                                          */
N/*     Redistributions  of source  code must  retain the  above copyright   */
N/*     notice, this list of conditions and the following disclaimer.        */
N/*                                                                          */
N/*     Redistributions in binary form  must reproduce the above copyright   */
N/*     notice, this  list of conditions  and the following  disclaimer in   */
N/*     the  documentation  and/or   other  materials  provided  with  the   */
N/*     distribution.                                                        */
N/*                                                                          */
N/*     Neither the  name of Texas Instruments Incorporated  nor the names   */
N/*     of its  contributors may  be used to  endorse or  promote products   */
N/*     derived  from   this  software  without   specific  prior  written   */
N/*     permission.                                                          */
N/*                                                                          */
N/*  THIS SOFTWARE  IS PROVIDED BY THE COPYRIGHT  HOLDERS AND CONTRIBUTORS   */
N/*  "AS IS"  AND ANY  EXPRESS OR IMPLIED  WARRANTIES, INCLUDING,  BUT NOT   */
N/*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR   */
N/*  A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT   */
N/*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,   */
N/*  SPECIAL,  EXEMPLARY,  OR CONSEQUENTIAL  DAMAGES  (INCLUDING, BUT  NOT   */
N/*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,   */
N/*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY   */
N/*  THEORY OF  LIABILITY, WHETHER IN CONTRACT, STRICT  LIABILITY, OR TORT   */
N/*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE   */
N/*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.    */
N/*                                                                          */
N/****************************************************************************/
N
N#ifndef __elfnames__
N#define __elfnames__
N
N#if defined(__TI_EABI__)
X#if 0L
S
S
S#define _divd __TI_P(divd)
S#define _divf __TI_P(divf)
S#define _absd __TI_P(absd)
S#define _absf __TI_P(absf)
S#define _addd __TI_P(addd)
S#define _addf __TI_P(addf)
S#define _cmpd __TI_P(cmpd)
S#define _eqld __TI_P(eqld)
S#define _neqd __TI_P(neqd)
S#define _lssd __TI_P(lssd)
S#define _gtrd __TI_P(gtrd)
S#define _leqd __TI_P(leqd)
S#define _geqd __TI_P(geqd)
S#define _cmpf __TI_P(cmpf)
S#define _eqlf __TI_P(eqlf)
S#define _neqf __TI_P(neqf)
S#define _lssf __TI_P(lssf)
S#define _gtrf __TI_P(gtrf)
S#define _leqf __TI_P(leqf)
S#define _geqf __TI_P(geqf)
S
S#define _nround __TI_P(nround)
S#define _roundf __TI_P(roundf)
S#define _roundl __TI_P(roundl)
S
S#define _cvtdf __TI_P(cvtdf)
S#define _cvtfd __TI_P(cvtfd)
S
S#define _fixdi          __TI_P(fixdi)
S#define _fixdli         __TI_P(fixdli)
S#define _fixdlli        __TI_P(fixdlli)
S#define _fixdu          __TI_P(fixdu)
S#define _fixdul         __TI_P(fixdul)
S#define _fixdull        __TI_P(fixdull)
S#define _fixfi          __TI_P(fixfi)
S#define _fixfli         __TI_P(fixfli)
S#define _fixflli        __TI_P(fixflli)
S#define _fixfu          __TI_P(fixfu)
S#define _fixful         __TI_P(fixful)
S#define _fixfull        __TI_P(fixfull)
S#define _fltid          __TI_P(fltid)
S#define _fltif          __TI_P(fltif)
S#define _fltlid         __TI_P(fltlid)
S#define _fltlif         __TI_P(fltlif)
S#define _fltllid        __TI_P(fltllid)
S#define _fltllif        __TI_P(fltllif)
S#define _fltud          __TI_P(fltud)
S#define _fltuf          __TI_P(fltuf)
S#define _fltuld         __TI_P(fltuld)
S#define _fltulf         __TI_P(fltulf)
S#define _fltulld        __TI_P(fltulld)
S#define _fltullf        __TI_P(fltullf)
S
S#define __fpclassify    __TI_P(fpclassify)
S#define __fpclassifyf   __TI_P(fpclassifyf)
S
S#define __frcdivd       __TI_P(frcdivd)
S#define __frcdivf       __TI_P(frcdivf)
S
S#define _frcmpyd_div    __TI_P(frcmpyd_div)
S#define _frcmpyf_div    __TI_P(frcmpyf_div)
S
S#define __isfinite      __TI_P(isfinite)
S#define __isfinitef     __TI_P(isfinitef)
S#define __isinf         __TI_P(isinf)
S#define __isinff        __TI_P(isinff)
S#define __isnan         __TI_P(isnan)
S#define __isnanf        __TI_P(isnanf)
S#define __isnormal      __TI_P(isnormal)
S#define __isnormalf     __TI_P(isnormalf)
S
S#define _mpyd __TI_P(mpyd)
S#define _mpyf __TI_P(mpyf)
S#define _negd __TI_P(negd)
S#define _negf __TI_P(negf)
S#define _subd __TI_P(subd)
S#define _subf __TI_P(subf)
S
S#define _trunc  __TI_P(trunc)
S#define _truncf __TI_P(truncf)
S#define _truncl __TI_P(truncl)
S
N#endif /* defined(__TI_EABI__) */
N
N#endif /* __elfnames__ */
L 59 "C:/ti/ccsv6/tools/compiler/c5500_4.4.1/include/math.h" 2
N
N#ifdef __cplusplus
S//----------------------------------------------------------------------------
S// <cmath> IS RECOMMENDED OVER <math.h>.  <math.h> IS PROVIDED FOR 
S// COMPATIBILITY WITH C AND THIS USAGE IS DEPRECATED IN C++
S//----------------------------------------------------------------------------
Sextern "C" namespace std {
N#endif
N
N__EXTERN double sqrt (double x);
Xextern double sqrt (double x);
N__EXTERN double exp  (double x);
Xextern double exp  (double x);
N__EXTERN double log  (double x);
Xextern double log  (double x);
N__EXTERN double log10(double x);
Xextern double log10(double x);
N__EXTERN double pow  (double x, double y);
Xextern double pow  (double x, double y);
N__EXTERN double sin  (double x);
Xextern double sin  (double x);
N__EXTERN double cos  (double x);
Xextern double cos  (double x);
N__EXTERN double tan  (double x);
Xextern double tan  (double x);
N__EXTERN double asin (double x);
Xextern double asin (double x);
N__EXTERN double acos (double x);
Xextern double acos (double x);
N__EXTERN double atan (double x);
Xextern double atan (double x);
N__EXTERN double atan2(double y, double x);
Xextern double atan2(double y, double x);
N__EXTERN double sinh (double x);
Xextern double sinh (double x);
N__EXTERN double cosh (double x);
Xextern double cosh (double x);
N__EXTERN double tanh (double x);
Xextern double tanh (double x);
N
N__INLINE double ceil (double x);
Xextern double ceil (double x);
N__INLINE double floor(double x);
Xextern double floor(double x);
N
N__EXTERN double fabs (double x);
Xextern double fabs (double x);
N
N__EXTERN double ldexp(double x, int n);
Xextern double ldexp(double x, int n);
N__EXTERN double frexp(double x, int *exp);
Xextern double frexp(double x, int *exp);
N__EXTERN double modf (double x, double *ip);
Xextern double modf (double x, double *ip);
N__EXTERN double fmod (double x, double y);
Xextern double fmod (double x, double y);
N
N/* An inline version of fmod that works for limited domain only */
N/* See comments in implementation below */
N__INLINE double _FMOD(double x, double y);
Xextern double _FMOD(double x, double y);
N
N/* these present in many linked images, so we'll tell you about them. */
N__EXTERN double _nround(double x); /* round-to-nearest */
Xextern double _nround(double x);  
N__EXTERN double _trunc(double x); /* truncate towards 0 */
Xextern double _trunc(double x);  
N
N#ifdef __cplusplus
S} /* extern "C" namespace std */
N#endif /* __cplusplus */
N
N/* the ANSI-optional *f and *l routines */
N#include <mathf.h>
L 1 "C:/ti/ccsv6/tools/compiler/c5500_4.4.1/include/mathf.h" 1
N/****************************************************************************/
N/*  mathf.h          v4.4.1                                                 */
N/*                                                                          */
N/* Copyright (c) 1997-2012 Texas Instruments Incorporated                   */
N/* http://www.ti.com/                                                       */
N/*                                                                          */
N/*  Redistribution and  use in source  and binary forms, with  or without   */
N/*  modification,  are permitted provided  that the  following conditions   */
N/*  are met:                                                                */
N/*                                                                          */
N/*     Redistributions  of source  code must  retain the  above copyright   */
N/*     notice, this list of conditions and the following disclaimer.        */
N/*                                                                          */
N/*     Redistributions in binary form  must reproduce the above copyright   */
N/*     notice, this  list of conditions  and the following  disclaimer in   */
N/*     the  documentation  and/or   other  materials  provided  with  the   */
N/*     distribution.                                                        */
N/*                                                                          */
N/*     Neither the  name of Texas Instruments Incorporated  nor the names   */
N/*     of its  contributors may  be used to  endorse or  promote products   */
N/*     derived  from   this  software  without   specific  prior  written   */
N/*     permission.                                                          */
N/*                                                                          */
N/*  THIS SOFTWARE  IS PROVIDED BY THE COPYRIGHT  HOLDERS AND CONTRIBUTORS   */
N/*  "AS IS"  AND ANY  EXPRESS OR IMPLIED  WARRANTIES, INCLUDING,  BUT NOT   */
N/*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR   */
N/*  A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT   */
N/*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,   */
N/*  SPECIAL,  EXEMPLARY,  OR CONSEQUENTIAL  DAMAGES  (INCLUDING, BUT  NOT   */
N/*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,   */
N/*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY   */
N/*  THEORY OF  LIABILITY, WHETHER IN CONTRACT, STRICT  LIABILITY, OR TORT   */
N/*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE   */
N/*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.    */
N/*                                                                          */
N/****************************************************************************/
N
N#ifndef _TI_ENHANCED_MATH_H
S#define _TI_ENHANCED_MATH_H
N#endif
N
N#ifndef __mathf__
N#define __mathf__
N
N#ifndef EDOM
S   #define EDOM   1
N#endif
N
N#ifndef ERANGE
S   #define ERANGE 2
N#endif
N
N#include <float.h>
N
N#if (FLT_DIG == DBL_DIG) /* float == double */
X#if (6 == 6)  
N
N#ifndef __math__
S#include <math.h>
N#endif
N
N#if !defined(DNKLIB)
X#if !0L
N#define sqrtf    sqrt
N#define expf     exp
N#define logf     log
N#define log10f   log10
N#define powf     pow
N#define sinf     sin
N#define cosf     cos
N#define tanf     tan
N#define asinf    asin
N#define acosf    acos
N#define atanf    atan
N#define atan2f   atan2
N#define sinhf    sinh
N#define coshf    cosh
N#define tanhf    tanh
N
N#define ceilf    ceil
N#define floorf   floor
N
N#define fabsf    fabs
N
N#define ldexpf   ldexp
N#define frexpf   frexp
N#define fmodf    fmod
N
N#ifdef __cplusplus
Snamespace std {
N#endif /* __cplusplus */
N
N/*modff()requires an actual function because the pointer arg cannot
N   be implicitly converted.  */
N__INLINE float modff (float x, float *ip);
Xextern float modff (float x, float *ip);
N#ifdef _INLINE
S__INLINE float modff (float x, float *ip)
S{
S   return modf(x, (double *)ip); 
S}
N#endif
N
N#ifdef __cplusplus
S} /* namespace std */      
N#endif
N
N#endif /* DNKLIB */
N
N#ifdef _TI_ENHANCED_MATH_H
N
N#define HUGE_VALF FLT_MAX
N#if !defined(DNKLIB)
X#if !0L
N#define rsqrtf   rsqrt
N#define exp2f    exp2
N#define exp10f   exp10
N#define log2f    log2
N#define powif    powi
N#define cotf     cot
N#define acotf    acot
N#define acot2f   acot2
N#define cothf    coth
N#define asinhf   asinh
N#define acoshf   acosh
N#define atanhf   atanh
N#define acothf   acoth
N#define truncf   _trunc
N#define roundf   _nround
N#define __isnormalf __isnormal
N#define __isinff   __isinf
N#define __isnanf   __isnan
N#define __isfinitef __isfinite
N#define __fpclassifyf __fpclassify
N#endif /* DNKLIB */
N#endif /* _TI_ENHANCED_MATH_H */
N
N#else /* float != double */
S#ifdef __cplusplus
S//----------------------------------------------------------------------------
S// <cmathf> IS RECOMMENDED OVER <mathf.h>.  <mathf.h> IS PROVIDED FOR 
S// COMPATIBILITY WITH C AND THIS USAGE IS DEPRECATED IN C++
S//----------------------------------------------------------------------------
Sextern "C" namespace std {
S#endif /* !__cplusplus */
S
S#include <access.h>
S
S__EXTERN float sqrtf (float x);
S__EXTERN float expf  (float x);
S__EXTERN float logf  (float x);
S__EXTERN float log10f(float x);
S__EXTERN float powf  (float x, float y);
S__EXTERN float sinf  (float x);
S__EXTERN float cosf  (float x);
S__EXTERN float tanf  (float x);
S__EXTERN float asinf (float x);
S__EXTERN float acosf (float x);
S__EXTERN float atanf (float x);
S__EXTERN float atan2f(float y, float x);
S__EXTERN float sinhf (float x);
S__EXTERN float coshf (float x);
S__EXTERN float tanhf (float x);
S
S__INLINE float ceilf (float x);
S__INLINE float floorf(float x);
S
S__EXTERN float fabsf (float x);
S
S__EXTERN float ldexpf(float x, int n);
S__EXTERN float frexpf(float x, int *exp);
S__EXTERN float modff (float x, float *ip);
S__EXTERN float fmodf (float x, float y);
S
S/* An inline version of fmodf that works for limited domain only */
S/* See comments in implementation below */
S__INLINE float _FMODF(float x, float y);
S
S/* these present in many linked images, so we'll tell you about them. */
S__EXTERN float _roundf(float x); /* round-to-nearest */
S__EXTERN float _truncf(float x); /* truncate towards 0 */
S
S#ifdef _TI_ENHANCED_MATH_H
S/* ------------------------------------------------- */
S/* Routines below are an addition to ANSI math.h     */
S/* Some (noted with "9x" in comment) will become ANSI*/
S/* once C9x is approved.                             */
S/* ------------------------------------------------- */
S
S#define HUGE_VALF FLT_MAX /* 9x */
S
S__EXTERN float rsqrtf(float x); /*   == 1/sqrtf(x) but *MUCH* faster         */
S__EXTERN float exp2f (float x); /*9x mathematically equiv to powf(2.0 ,x)    */
S__EXTERN float exp10f(float x); /*   mathematically equiv to powf(10.0,x)    */
S__EXTERN float log2f (float x); /*9x mathematically equiv to logf(x)/logf(2.)*/
S
S__EXTERN float powif (float x, int i); /* equiv to powf(x,(float)i) */
S
S__EXTERN float cotf  (float x);
S__EXTERN float acotf (float x);
S__EXTERN float acot2f(float x, float y);
S
S__EXTERN float cothf (float x);
S
S__EXTERN float asinhf(float x); /* 9x */
S__EXTERN float acoshf(float x); /* 9x */
S__EXTERN float atanhf(float x); /* 9x */
S__EXTERN float acothf(float x);
S
S#ifndef __INLINE_ISINF__
S#define __INLINE_ISINF__ 0
S#endif
S
S#if __INLINE_ISINF__
S__INLINE int __isinff(float x);
S#else
S__EXTERN int __isinff(float x);
S#endif
S
S__INLINE int __isnanf(float x);
S__INLINE int __isfinitef(float x);
S__INLINE int __isnormalf(float x);
S__EXTERN int __fpclassifyf(float x);
S
S#define isinf(x) (sizeof(x) == sizeof(double) ? __isinf(x) : \
S                  sizeof(x) == sizeof(float) ? __isinff(x) : __isinfl(x))
X#define isinf(x) (sizeof(x) == sizeof(double) ? __isinf(x) :                   sizeof(x) == sizeof(float) ? __isinff(x) : __isinfl(x))
S
S#define isnan(x) (sizeof(x) == sizeof(double) ? __isnan(x) : \
S                  sizeof(x) == sizeof(float) ? __isnanf(x) : __isnanl(x))
X#define isnan(x) (sizeof(x) == sizeof(double) ? __isnan(x) :                   sizeof(x) == sizeof(float) ? __isnanf(x) : __isnanl(x))
S
S#define isfinite(x) (sizeof(x) == sizeof(double) ? __isfinite(x) : \
S                     sizeof(x) == sizeof(float) ? __isfinitef(x) : \
S                     __isfinitel(x))
X#define isfinite(x) (sizeof(x) == sizeof(double) ? __isfinite(x) :                      sizeof(x) == sizeof(float) ? __isfinitef(x) :                      __isfinitel(x))
S
S#define isnormal(x) (sizeof(x) == sizeof(double) ? __isnormal(x) : \
S                     sizeof(x) == sizeof(float) ? __isnormalf(x) : \
S                     __isnormall(x))
X#define isnormal(x) (sizeof(x) == sizeof(double) ? __isnormal(x) :                      sizeof(x) == sizeof(float) ? __isnormalf(x) :                      __isnormall(x))
S
S#define fpclassify(x) (sizeof(x) == sizeof(double) ? __fpclassify(x) : \
S                       sizeof(x) == sizeof(float) ? __fpclassifyf(x) : \
S                       __fpclassifyl(x))
X#define fpclassify(x) (sizeof(x) == sizeof(double) ? __fpclassify(x) :                        sizeof(x) == sizeof(float) ? __fpclassifyf(x) :                        __fpclassifyl(x))
S
S#define roundf _roundf /* 9x round-to-nearest   */
S#define truncf _truncf /* 9x truncate towards 0 */
S
S#endif /* defined(_TI_ENHANCED_MATH_H) */
S
S
S#ifdef _INLINE
S/****************************************************************************/
S/*  Inline versions of floorf, ceilf, fmodf                                 */
S/****************************************************************************/
Sstatic __inline float floorf(float x) 
S{
S   float y; 
S   return (modff(x, &y) < 0 ? y - 1 : y);
S}
S
Sstatic __inline float ceilf(float x)
S{
S   float y; 
S   return (modff(x, &y) > 0 ? y + 1 : y);
S}
S
S/* 
S   The implementation below does not work correctly for all cases.
S   Consider the case of fmod(Big, 3), for any Big > 2**(MANT_DIG+2).
S   The correct result is one of 0,1, or 2.
S   But the implementation below will *always* return 0 
S   because the quotient is only an approximation.
S*/
Sstatic __inline float _FMODF(float x, float y)
S{
S   float d = fabsf(x); 
S   if (d - fabsf(y) == d) return (0);
S   modff(x/y, &d);  
S   return (x - d * y);
S}
S
S#ifdef _TI_ENHANCED_MATH_H
S
S#if __INLINE_ISINF__
S#ifndef REAL_TO_REALNUM
S#error isinf can only be inlined in the compilation of the rts
S#endif
S
Sstatic __inline int __isinff(float x)
S{
S  realnum _x;
S  REAL_TO_REALNUM(x, _x);
S  return _x.exp == (REAL_EMAX + 1) && (_x.mantissa << 1) == 0;
S}
S
S#endif /* __INLINE_ISINF__ */
S
Sstatic __inline int __isnanf(volatile float x)
S{
S  return x != x;
S}
S
Sstatic __inline int __isfinitef(float x)
S{
S  return (!__isinff(x) && !__isnanf(x));
S}
S
Sstatic __inline int __isnormalf(float x)
S{
S  return (__isfinitef(x) && x != 0.0);
S}
S
S#endif /* defined(_TI_ENHANCED_MATH_H) */
S
S#endif /* _INLINE */
S
S#include <unaccess.h>
S
S#ifdef __cplusplus
S} /* extern "C" namespace std */
S#endif /* __cplusplus */
S
N#endif /*  float == double */
N
N#endif /* __mathf__ */
N
N#if defined(__cplusplus) && !defined(_CPP_STYLE_HEADER) && (FLT_DIG != DBL_DIG)
X#if 0L && !0L && (6 != 6)
Susing std::sqrtf;
Susing std::expf;
Susing std::logf;
Susing std::log10f;
Susing std::powf;
Susing std::sinf;
Susing std::cosf;
Susing std::tanf;
Susing std::asinf;
Susing std::acosf;
Susing std::atanf;
Susing std::atan2f;
Susing std::sinhf;
Susing std::coshf;
Susing std::tanhf;
S
Susing std::ceilf;
Susing std::floorf;
S
Susing std::fabsf;
S
Susing std::ldexpf;
Susing std::frexpf;
Susing std::modff;
Susing std::fmodf;
S
Susing std::_FMODF;
S
S#ifdef _TI_ENHANCED_MATH_H
Susing std::rsqrtf; /*   == 1/sqrtf(x) but *MUCH* faster         */
Susing std::exp2f;  /*9x mathematically equiv to powf(2.0 ,x)    */
Susing std::exp10f; /*   mathematically equiv to powf(10.0,x)    */
Susing std::log2f;  /*9x mathematically equiv to logf(x)/logf(2.)*/
S
Susing std::powif;  /* equiv to powf(x,(float)i) */
S
Susing std::cotf; 
Susing std::acotf;
Susing std::acot2f;
S
Susing std::cothf;
S
Susing std::asinhf; /* 9x */
Susing std::acoshf; /* 9x */
Susing std::atanhf; /* 9x */
Susing std::acothf;
S#endif /* _TI_ENHANCED_MATH_H */
S
N#endif /* ! _CPP_STYLE_HEADER */
N
N#if defined(__cplusplus) && (FLT_DIG != DBL_DIG) && defined(_TI_ENHANCED_MATH_H)
X#if 0L && (6 != 6) && 1L
Susing std::__isnanf;
Susing std::__isinff;
Susing std::__isfinitef;
Susing std::__isnormalf;
Susing std::__fpclassifyf;
Susing std::_roundf; /* round-to-nearest */
Susing std::_truncf; /* truncate towards 0 */
N#endif /* __cplusplus && FLT_DIG != DBL_DIG && _TI_ENHANCED_MATH_H */
L 108 "C:/ti/ccsv6/tools/compiler/c5500_4.4.1/include/math.h" 2
N#include <mathl.h>
L 1 "C:/ti/ccsv6/tools/compiler/c5500_4.4.1/include/mathl.h" 1
N/****************************************************************************/
N/*  mathl.h          v4.4.1                                                 */
N/*                                                                          */
N/* Copyright (c) 1997-2012 Texas Instruments Incorporated                   */
N/* http://www.ti.com/                                                       */
N/*                                                                          */
N/*  Redistribution and  use in source  and binary forms, with  or without   */
N/*  modification,  are permitted provided  that the  following conditions   */
N/*  are met:                                                                */
N/*                                                                          */
N/*     Redistributions  of source  code must  retain the  above copyright   */
N/*     notice, this list of conditions and the following disclaimer.        */
N/*                                                                          */
N/*     Redistributions in binary form  must reproduce the above copyright   */
N/*     notice, this  list of conditions  and the following  disclaimer in   */
N/*     the  documentation  and/or   other  materials  provided  with  the   */
N/*     distribution.                                                        */
N/*                                                                          */
N/*     Neither the  name of Texas Instruments Incorporated  nor the names   */
N/*     of its  contributors may  be used to  endorse or  promote products   */
N/*     derived  from   this  software  without   specific  prior  written   */
N/*     permission.                                                          */
N/*                                                                          */
N/*  THIS SOFTWARE  IS PROVIDED BY THE COPYRIGHT  HOLDERS AND CONTRIBUTORS   */
N/*  "AS IS"  AND ANY  EXPRESS OR IMPLIED  WARRANTIES, INCLUDING,  BUT NOT   */
N/*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR   */
N/*  A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT   */
N/*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,   */
N/*  SPECIAL,  EXEMPLARY,  OR CONSEQUENTIAL  DAMAGES  (INCLUDING, BUT  NOT   */
N/*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,   */
N/*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY   */
N/*  THEORY OF  LIABILITY, WHETHER IN CONTRACT, STRICT  LIABILITY, OR TORT   */
N/*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE   */
N/*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.    */
N/*                                                                          */
N/****************************************************************************/
N
N#ifndef _TI_ENHANCED_MATH_H
S#define _TI_ENHANCED_MATH_H
N#endif
N
N#ifndef __mathl__
N#define __mathl__
N
N#ifndef EDOM
S   #define EDOM   1
N#endif
N
N#ifndef ERANGE
S   #define ERANGE 2
N#endif
N
N#include <float.h>
N#include <access.h>
N
N#if (LDBL_DIG == DBL_DIG) /* long double == double*/
X#if (6 == 6)  
N
N#ifndef __math__
S#include <math.h>
N#endif
N
N#if !defined(DNKLIB)
X#if !0L
N#define logl     log
N#define log10l   log10
N#define sinl     sin
N#define cosl     cos
N#define sinhl    sinh
N#define coshl    cosh
N#endif
N
N#define sqrtl    sqrt
N#define expl     exp
N#define powl     pow
N#define tanl     tan
N#define asinl    asin
N#define acosl    acos
N#define atanl    atan
N#define atan2l   atan2
N#define tanhl    tanh
N
N#define ceill    ceil
N#define floorl   floor
N
N#define fabsl    fabs
N
N#define ldexpl   ldexp
N#define frexpl   frexp
N#define fmodl    fmod
N
N#ifdef __cplusplus
Snamespace std {
N#endif /* __cplusplus */
N
N/* modfl() requires an actual function because the pointer arg cannot
N   be implicitly converted.  */
N__INLINE long double modfl (long double x, long double *ip);
Xextern long double modfl (long double x, long double *ip);
N#ifdef _INLINE
S__INLINE long double modfl (long double x, long double *ip)
S{
S   return modf(x, (double *)ip); 
S}
N#endif
N
N#ifdef __cplusplus
S} /* namespace std */      
N#endif
N
N#ifdef _TI_ENHANCED_MATH_H
N
N#define HUGE_VALL LDBL_MAX
N
N#define rsqrtl   rsqrt
N#define exp2l    exp2
N#define exp10l   exp10
N#define log2l    log2
N#define powil    powi
N#define cotl     cot
N#define acotl    acot
N#define acot2l   acot2
N#define cothl    coth
N#define asinhl   asinh
N#define acoshl   acosh
N#define atanhl   atanh
N#define acothl   acoth
N#define truncl   _trunc
N#define roundl   _nround
N#define __isnormall __isnormal
N#define __isinfl   __isinf
N#define __isnanl   __isnan
N#define __isfinitel __isfinite
N#define __fpclassifyl __fpclassify
N
N#endif /* _TI_ENHANCED_MATH_H */
N
N#else /* long double != double */
S#ifdef __cplusplus
S//----------------------------------------------------------------------------
S// <cmathl> IS RECOMMENDED OVER <mathl.h>.  <mathl.h> IS PROVIDED FOR 
S// COMPATIBILITY WITH C AND THIS USAGE IS DEPRECATED IN C++
S//----------------------------------------------------------------------------
Sextern "C" namespace std {
S#endif /* !__cplusplus */
S
S__EXTERN long double sqrtl (long double x);
S__EXTERN long double expl  (long double x);
S__EXTERN long double logl  (long double x);
S__EXTERN long double log10l(long double x);
S__EXTERN long double powl  (long double x, long double y);
S__EXTERN long double sinl  (long double x);
S__EXTERN long double cosl  (long double x);
S__EXTERN long double tanl  (long double x);
S__EXTERN long double asinl (long double x);
S__EXTERN long double acosl (long double x);
S__EXTERN long double atanl (long double x);
S__EXTERN long double atan2l(long double y, long double x);
S__EXTERN long double sinhl (long double x);
S__EXTERN long double coshl (long double x);
S__EXTERN long double tanhl (long double x);
S
S__INLINE long double ceill (long double x);
S__INLINE long double floorl(long double x);
S
S__EXTERN long double fabsl (long double x);
S
S__EXTERN long double ldexpl(long double x, int n);
S__EXTERN long double frexpl(long double x, int *exp);
S__EXTERN long double modfl (long double x, long double *ip);
S__EXTERN long double fmodl (long double x, long double y);
S
S/* An inline version of fmod that works for limited domain only */
S/* See comments in implementation below */
Sstatic __inline long double _FMODL(long double x, long double y);
S
S__EXTERN long double _roundl(long double x); /* round-to-nearest */
S__EXTERN long double _truncl(long double x); /* truncate towards 0 */
S
S#ifdef _TI_ENHANCED_MATH_H
S/* ------------------------------------------------- */
S/* Routines below are an addition to ANSI math.h     */
S/* Some (noted with "9x" in comment) will become ANSI*/
S/* once C9x is approved.                             */
S/* ------------------------------------------------- */
S
S#define HUGE_VALL LDBL_MAX /* ## */
S
S__EXTERN long double rsqrtl(long double x); /*   1/sqrtl(x) but *MUCH* faster*/
S__EXTERN long double exp2l (long double x); /*9x math equiv to powl(2.0 ,x)  */
S__EXTERN long double exp10l(long double x); /*   math equiv to powl(10.0,x)  */
S__EXTERN long double log2l (long double x);/*9x math equiv to logl(x)/logl(2)*/
S
S__EXTERN long double powil (long double x, int i); /* == powl(x,(long double)i)*/
S
S__EXTERN long double cotl  (long double x);
S__EXTERN long double acotl (long double x);
S__EXTERN long double acot2l(long double x, long double y);
S
S__EXTERN long double cothl (long double x);
S
S__EXTERN long double asinhl(long double x); /* 9x */
S__EXTERN long double acoshl(long double x); /* 9x */
S__EXTERN long double atanhl(long double x); /* 9x */
S__EXTERN long double acothl(long double x);
S
S#ifndef __INLINE_ISINF__
S#define __INLINE_ISINF__ 0
S#endif
S
S#if __INLINE_ISINF__
S__INLINE int __isinfl(long double x);
S#else
S__EXTERN int __isinfl(long double x);
S#endif
S
S__INLINE int __isnanl(long double x);
S__INLINE int __isfinitel(long double x);
S__INLINE int __isnormall(long double x);
S__EXTERN int __fpclassifyl(long double x);
S
S#define isinf(x) (sizeof(x) == sizeof(double) ? __isinf(x) : \
S                  sizeof(x) == sizeof(float) ? __isinff(x) : __isinfl(x))
X#define isinf(x) (sizeof(x) == sizeof(double) ? __isinf(x) :                   sizeof(x) == sizeof(float) ? __isinff(x) : __isinfl(x))
S
S#define isnan(x) (sizeof(x) == sizeof(double) ? __isnan(x) : \
S                  sizeof(x) == sizeof(float) ? __isnanf(x) : __isnanl(x))
X#define isnan(x) (sizeof(x) == sizeof(double) ? __isnan(x) :                   sizeof(x) == sizeof(float) ? __isnanf(x) : __isnanl(x))
S
S#define isfinite(x) (sizeof(x) == sizeof(double) ? __isfinite(x) : \
S                     sizeof(x) == sizeof(float) ? __isfinitef(x) : \
S                     __isfinitel(x))
X#define isfinite(x) (sizeof(x) == sizeof(double) ? __isfinite(x) :                      sizeof(x) == sizeof(float) ? __isfinitef(x) :                      __isfinitel(x))
S
S#define isnormal(x) (sizeof(x) == sizeof(double) ? __isnormal(x) : \
S                     sizeof(x) == sizeof(float) ? __isnormalf(x) : \
S                     __isnormall(x))
X#define isnormal(x) (sizeof(x) == sizeof(double) ? __isnormal(x) :                      sizeof(x) == sizeof(float) ? __isnormalf(x) :                      __isnormall(x))
S
S#define fpclassify(x) (sizeof(x) == sizeof(double) ? __fpclassify(x) : \
S                       sizeof(x) == sizeof(float) ? __fpclassifyf(x) : \
S                       __fpclassifyl(x))
X#define fpclassify(x) (sizeof(x) == sizeof(double) ? __fpclassify(x) :                        sizeof(x) == sizeof(float) ? __fpclassifyf(x) :                        __fpclassifyl(x))
S
S#define roundl _roundl /* 9x round-to-nearest   */
S#define truncl _truncl /* 9x truncate towards 0 */
S
S#endif /* defined(_TI_ENHANCED_MATH_H) */
S
S
S#ifdef _INLINE
S/****************************************************************************/
S/*  Inline versions of floorl, ceill, fmodl                                 */
S/****************************************************************************/
Sstatic inline long double floorl(long double x) 
S{
S   long double y; 
S   return (modfl(x, &y) < 0 ? y - 1 : y);
S}
S
Sstatic inline long double ceill(long double x)
S{
S   long double y; 
S   return (modfl(x, &y) > 0 ? y + 1 : y);
S}
S
S/* 
S   The implementation below does not work correctly for all cases.
S   Consider the case of fmod(Big, 3), for any Big > 2**(MANT_DIG+2).
S   The correct result is one of 0,1, or 2.
S   But the implementation below will *always* return 0 
S   because the quotient is only an approximation.
S*/
Sstatic inline long double _FMODL(long double x, long double y)
S{
S   long double d = fabsl(x); 
S   if (d - fabsl(y) == d) return (0);
S   modfl(x/y, &d);  
S   return (x - d * y);
S}
S
S#ifdef _TI_ENHANCED_MATH_H
S
S#if __INLINE_ISINF__
S#ifndef REAL_TO_REALNUM
S#error isinf can only be inlined in the compilation of the rts
S#endif
S
Sstatic __inline int __isinfl(long double x)
S{
S  realnum _x;
S  REAL_TO_REALNUM(x, _x);
S  return _x.exp == (REAL_EMAX + 1) && (_x.mantissa << 1) == 0;
S}
S
S#endif /* __INLINE_ISINF__ */
S
Sstatic __inline int __isnanl(volatile long double x)
S{
S  return x != x;
S}
S
Sstatic __inline int __isfinitel(long double x)
S{
S  return (!__isinfl(x) && !__isnanl(x));
S}
S
Sstatic __inline int __isnormall(long double x)
S{
S  return (__isfinitel(x) && x != 0.0);
S}
S
S#endif /* defined(_TI_ENHANCED_MATH_H) */
S
S#endif /* defined(_INLINE) */
S
S#include <unaccess.h>
S#ifdef __cplusplus
S} /* extern "C" namespace std */
S#endif /* __cplusplus */
S
N#endif /* long double == double */
N
N#endif /* __mathl__ */
N
N#if defined(__cplusplus) && !defined(_CPP_STYLE_HEADER) && (LDBL_DIG != DBL_DIG)
X#if 0L && !0L && (6 != 6)
Susing std::sqrtl;
Susing std::expl ;
Susing std::logl ;
Susing std::log10l;
Susing std::powl;
Susing std::sinl;
Susing std::cosl;
Susing std::tanl;
Susing std::asinl;
Susing std::acosl;
Susing std::atanl;
Susing std::atan2l;
Susing std::sinhl;
Susing std::coshl;
Susing std::tanhl;
S
Susing std::ceill;
Susing std::floorl;
S
Susing std::fabsl;
S
Susing std::ldexpl;
Susing std::frexpl;
Susing std::modfl;
Susing std::fmodl;
S
Susing std::_FMODL;
S
S#ifdef _TI_ENHANCED_MATH_H
Susing std::rsqrtl; /*   1/sqrtl(x) but *MUCH* faster*/
Susing std::exp2l;  /*9x math equiv to powl(2.0 ,x)  */
Susing std::exp10l; /*   math equiv to powl(10.0,x)  */
Susing std::log2l;  /*9x math equiv to logl(x)/logl(2)*/
S
Susing std::powil; /* == powl(x,(long double)i)*/
S
Susing std::cotl;
Susing std::acotl;
Susing std::acot2l;
S
Susing std::cothl;
S
Susing std::asinhl; /* 9x */
Susing std::acoshl; /* 9x */
Susing std::atanhl; /* 9x */
Susing std::acothl;
S#endif /* _TI_ENHANCED_MATH_H */
S
N#endif /* ! _CPP_STYLE_HEADER */
N
N#if defined(__cplusplus) && (LDBL_DIG != DBL_DIG) && defined(_TI_ENHANCED_MATH_H)
X#if 0L && (6 != 6) && 1L
Susing std::__isnanl;
Susing std::__isinfl;
Susing std::__isfinitel;
Susing std::__isnormall;
Susing std::__fpclassifyl;
Susing std::_roundl; /* round-to-nearest */
Susing std::_truncl; /* truncate towards 0 */
N#endif /* __cplusplus && LDBL_DIG != DBL_DIG && _TI_ENHANCED_MATH_H */
L 109 "C:/ti/ccsv6/tools/compiler/c5500_4.4.1/include/math.h" 2
N
N#include <access.h>
N
N#ifdef __cplusplus
Sextern "C" namespace std {
N#endif
N
N#ifdef _TI_ENHANCED_MATH_H
N/* ------------------------------------------------- */
N/* Routines below are an addition to ANSI math.h     */
N/* Some (noted with "9x" in comment) will become ANSI*/
N/* once C9x is approved.                             */
N/* ------------------------------------------------- */
N
N__EXTERN double rsqrt(double x); /*   == 1/sqrt(x) but *MUCH* faster         */
Xextern double rsqrt(double x);  
N__EXTERN double exp2 (double x); /*9x mathematically equiv to pow(2.0 ,x)    */
Xextern double exp2 (double x);  
N__EXTERN double exp10(double x); /*   mathematically equiv to pow(10.0,x)    */
Xextern double exp10(double x);  
N__EXTERN double log2 (double x); /*9x mathematically equiv to log(x)/log(2.0)*/
Xextern double log2 (double x);  
N
N__EXTERN double powi(double x, int i); /* equiv to pow(x,(double)i) */
Xextern double powi(double x, int i);  
N
N__EXTERN double cot  (double x);
Xextern double cot  (double x);
N__EXTERN double acot (double x);
Xextern double acot (double x);
N__EXTERN double acot2(double x, double y);
Xextern double acot2(double x, double y);
N
N__EXTERN double coth (double x);
Xextern double coth (double x);
N
N__EXTERN double asinh(double x); /* 9x */
Xextern double asinh(double x);  
N__EXTERN double acosh(double x); /* 9x */
Xextern double acosh(double x);  
N__EXTERN double atanh(double x); /* 9x */
Xextern double atanh(double x);  
N__EXTERN double acoth(double x);
Xextern double acoth(double x);
N
N#ifndef __INLINE_ISINF__
N#define __INLINE_ISINF__ 0
N#endif
N
N#if __INLINE_ISINF__
X#if 0
S__INLINE int __isinf(double x);
N#else
N__EXTERN int __isinf(double x);
Xextern int __isinf(double x);
N#endif
N
N__INLINE int __isnan(volatile double x);
Xextern int __isnan(volatile double x);
N__INLINE int __isfinite(double x);
Xextern int __isfinite(double x);
N__INLINE int __isnormal(double x);
Xextern int __isnormal(double x);
N__EXTERN int __fpclassify(double x);
Xextern int __fpclassify(double x);
N
N#define isinf(x) (sizeof(x) == sizeof(double) ? __isinf(x) : \
N                  sizeof(x) == sizeof(float) ? __isinff(x) : __isinfl(x))
X#define isinf(x) (sizeof(x) == sizeof(double) ? __isinf(x) :                   sizeof(x) == sizeof(float) ? __isinff(x) : __isinfl(x))
N
N#define isnan(x) (sizeof(x) == sizeof(double) ? __isnan(x) : \
N                  sizeof(x) == sizeof(float) ? __isnanf(x) : __isnanl(x))
X#define isnan(x) (sizeof(x) == sizeof(double) ? __isnan(x) :                   sizeof(x) == sizeof(float) ? __isnanf(x) : __isnanl(x))
N
N#define isfinite(x) (sizeof(x) == sizeof(double) ? __isfinite(x) : \
N                     sizeof(x) == sizeof(float) ? __isfinitef(x) : \
N                     __isfinitel(x))
X#define isfinite(x) (sizeof(x) == sizeof(double) ? __isfinite(x) :                      sizeof(x) == sizeof(float) ? __isfinitef(x) :                      __isfinitel(x))
N
N#define isnormal(x) (sizeof(x) == sizeof(double) ? __isnormal(x) : \
N                     sizeof(x) == sizeof(float) ? __isnormalf(x) : \
N                     __isnormall(x))
X#define isnormal(x) (sizeof(x) == sizeof(double) ? __isnormal(x) :                      sizeof(x) == sizeof(float) ? __isnormalf(x) :                      __isnormall(x))
N
N#define fpclassify(x) (sizeof(x) == sizeof(double) ? __fpclassify(x) : \
N                       sizeof(x) == sizeof(float) ? __fpclassifyf(x) : \
N                       __fpclassifyl(x))
X#define fpclassify(x) (sizeof(x) == sizeof(double) ? __fpclassify(x) :                        sizeof(x) == sizeof(float) ? __fpclassifyf(x) :                        __fpclassifyl(x))
N
N#define round _nround /* 9x round-to-nearest   */
N#define trunc _trunc /* 9x truncate towards 0 */
N
N/*Definitions of classification macros used in fp_classify 
N  We do not support subnormal numbers yet, but the classification exists for
N  when they are supported */
N
N#define FP_INFINITE  1
N#define FP_NAN       2
N#define FP_NORMAL    3
N#define FP_ZERO      4
N#define FP_SUBNORMAL 5
N
N#endif /* defined(_TI_ENHANCED_MATH_H) */
N
N#ifdef __cplusplus
S} /* extern "C" namespace std */
S
N#endif /* __cplusplus */
N
N
N#ifdef _INLINE
S/****************************************************************************/
S/*  Inline versions of floor, ceil, fmod                                    */
S/****************************************************************************/
S
S#ifdef __cplusplus
Snamespace std {
S#endif
S
Sstatic __inline double floor(double x) 
S{
S   double y; 
S   return (modf(x, &y) < 0 ? y - 1 : y);
S}
S
Sstatic __inline double ceil(double x)
S{
S   double y; 
S   return (modf(x, &y) > 0 ? y + 1 : y);
S}
S
S/* 
S   The implementation below does not work correctly for all cases.
S   Consider the case of fmod(Big, 3), for any Big > 2**(MANT_DIG+2).
S   The correct result is one of 0,1, or 2.
S   But the implementation below will *always* return 0 
S   because the quotient is only an approximation.
S*/
Sstatic __inline double _FMOD(double x, double y)
S{
S   double d = fabs(x); 
S   if (d - fabs(y) == d) return (0);
S   modf(x/y, &d);  
S   return (x - d * y);
S}
S
S#ifdef _TI_ENHANCED_MATH_H
S
S#if __INLINE_ISINF__
S#ifndef REAL_TO_REALNUM
S#error isinf can only be inlined in the compilation of the rts
S#endif
S
Sstatic __inline int __isinf(double x)
S{
S  realnum _x;
S  REAL_TO_REALNUM(x, _x);
S  return _x.exp == (REAL_EMAX + 1) && (_x.mantissa << 1) == 0;
S}
S
S#endif /* __INLINE_ISINF___ */
S
S#pragma diag_suppress 681
Sstatic __inline int __isnan(volatile double x)
S{
S  return x != x;
S}
S#pragma diag_default 681
S
Sstatic __inline int __isfinite(double x)
S{
S  return (!__isinf(x) && !__isnan(x));
S}
S
Sstatic __inline int __isnormal(double x)
S{
S  return (__isfinite(x) && x != 0.0);
S}
S
S#endif /* defined(_TI_ENHANCED_MATH_H) */
S
S#ifdef __cplusplus
S} /* namespace std */
S#endif /* __cplusplus */
S
N#endif /* _INLINE */
N
N/*******************************************************************************/
N/* CQ35082 : Overloaded version of math functions for float and long double    */
N/*           removed from here, and include in cmath instead (see Section 26.5 */
N/*           of C++ standard for details). Thus cpp_inline_math.h is now       */
N/*           included in cmath .                                               */
N/*******************************************************************************/
N#include <unaccess.h>
L 1 "C:/ti/ccsv6/tools/compiler/c5500_4.4.1/include/unaccess.h" 1
N/****************************************************************************/
N/*  UNACCESS.H v4.4.1                                                       */
N/*                                                                          */
N/* Copyright (c) 2000-2012 Texas Instruments Incorporated                   */
N/* http://www.ti.com/                                                       */
N/*                                                                          */
N/*  Redistribution and  use in source  and binary forms, with  or without   */
N/*  modification,  are permitted provided  that the  following conditions   */
N/*  are met:                                                                */
N/*                                                                          */
N/*     Redistributions  of source  code must  retain the  above copyright   */
N/*     notice, this list of conditions and the following disclaimer.        */
N/*                                                                          */
N/*     Redistributions in binary form  must reproduce the above copyright   */
N/*     notice, this  list of conditions  and the following  disclaimer in   */
N/*     the  documentation  and/or   other  materials  provided  with  the   */
N/*     distribution.                                                        */
N/*                                                                          */
N/*     Neither the  name of Texas Instruments Incorporated  nor the names   */
N/*     of its  contributors may  be used to  endorse or  promote products   */
N/*     derived  from   this  software  without   specific  prior  written   */
N/*     permission.                                                          */
N/*                                                                          */
N/*  THIS SOFTWARE  IS PROVIDED BY THE COPYRIGHT  HOLDERS AND CONTRIBUTORS   */
N/*  "AS IS"  AND ANY  EXPRESS OR IMPLIED  WARRANTIES, INCLUDING,  BUT NOT   */
N/*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR   */
N/*  A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT   */
N/*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,   */
N/*  SPECIAL,  EXEMPLARY,  OR CONSEQUENTIAL  DAMAGES  (INCLUDING, BUT  NOT   */
N/*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,   */
N/*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY   */
N/*  THEORY OF  LIABILITY, WHETHER IN CONTRACT, STRICT  LIABILITY, OR TORT   */
N/*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE   */
N/*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.    */
N/*                                                                          */
N/****************************************************************************/
N/* unaccess.h: Empty by default */
L 279 "C:/ti/ccsv6/tools/compiler/c5500_4.4.1/include/math.h" 2
N
N#endif /* __math__ */
N
N#if defined(__cplusplus) && !defined(_CPP_STYLE_HEADER)
X#if 0L && !0L
Susing std::sqrt; 
Susing std::exp; 
Susing std::log; 
Susing std::log10; 
Susing std::pow; 
Susing std::sin; 
Susing std::cos; 
Susing std::tan; 
Susing std::asin; 
Susing std::acos;
Susing std::atan;
Susing std::atan2;
Susing std::sinh;
Susing std::cosh;
Susing std::tanh;
Susing std::ceil;
Susing std::floor;
Susing std::fabs;
Susing std::ldexp;
Susing std::frexp;
Susing std::modf;
Susing std::fmod;
S
S#ifdef _TI_ENHANCED_MATH_H
Susing std::rsqrt;
Susing std::exp2;
Susing std::exp10;
Susing std::log2;
Susing std::powi;
Susing std::cot;
Susing std::acot;
Susing std::acot2;
Susing std::coth;
Susing std::asinh;
Susing std::acosh;
Susing std::atanh;
Susing std::acoth;
S#endif /* _TI_ENHANCED_MATH_H */
S
N#endif /* _CPP_STYLE_HEADER */
N
N#if defined(__cplusplus) && defined(_TI_ENHANCED_MATH_H)
X#if 0L && 1L
Susing std::__isnan;
Susing std::__isinf;
Susing std::__isfinite;
Susing std::__isnormal;
Susing std::__fpclassify;
Susing std::_nround; /* round-to-nearest */
Susing std::_trunc;
N#endif /* __cplusplus && _TI_ENHANCED_MATH_H */
L 15 "../inc/ECGDemoNonBios.h" 2
N#include<file.h>
L 1 "C:/ti/ccsv6/tools/compiler/c5500_4.4.1/include/file.h" 1
N/*****************************************************************************/
N/*  FILE.H v4.4.1                                                            */
N/*                                                                           */
N/* Copyright (c) 1995-2012 Texas Instruments Incorporated                    */
N/* http://www.ti.com/                                                        */
N/*                                                                           */
N/*  Redistribution and  use in source  and binary forms, with  or without    */
N/*  modification,  are permitted provided  that the  following conditions    */
N/*  are met:                                                                 */
N/*                                                                           */
N/*     Redistributions  of source  code must  retain the  above copyright    */
N/*     notice, this list of conditions and the following disclaimer.         */
N/*                                                                           */
N/*     Redistributions in binary form  must reproduce the above copyright    */
N/*     notice, this  list of conditions  and the following  disclaimer in    */
N/*     the  documentation  and/or   other  materials  provided  with  the    */
N/*     distribution.                                                         */
N/*                                                                           */
N/*     Neither the  name of Texas Instruments Incorporated  nor the names    */
N/*     of its  contributors may  be used to  endorse or  promote products    */
N/*     derived  from   this  software  without   specific  prior  written    */
N/*     permission.                                                           */
N/*                                                                           */
N/*  THIS SOFTWARE  IS PROVIDED BY THE COPYRIGHT  HOLDERS AND CONTRIBUTORS    */
N/*  "AS IS"  AND ANY  EXPRESS OR IMPLIED  WARRANTIES, INCLUDING,  BUT NOT    */
N/*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR    */
N/*  A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT    */
N/*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,    */
N/*  SPECIAL,  EXEMPLARY,  OR CONSEQUENTIAL  DAMAGES  (INCLUDING, BUT  NOT    */
N/*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,    */
N/*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY    */
N/*  THEORY OF  LIABILITY, WHETHER IN CONTRACT, STRICT  LIABILITY, OR TORT    */
N/*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE    */
N/*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.     */
N/*                                                                           */
N/*****************************************************************************/
N
N/*****************************************************************************/
N/* Macros and declarations used in lowlevel I/O functions.                   */
N/*****************************************************************************/
N#ifndef _FILE
N#define _FILE
N
N#include <linkage.h>
L 1 "C:/ti/ccsv6/tools/compiler/c5500_4.4.1/include/linkage.h" 1
N/*****************************************************************************/
N/* linkage.h   v4.4.1                                                        */
N/*                                                                           */
N/* Copyright (c) 1998-2012 Texas Instruments Incorporated                    */
N/* http://www.ti.com/                                                        */
N/*                                                                           */
N/*  Redistribution and  use in source  and binary forms, with  or without    */
N/*  modification,  are permitted provided  that the  following conditions    */
N/*  are met:                                                                 */
N/*                                                                           */
N/*     Redistributions  of source  code must  retain the  above copyright    */
N/*     notice, this list of conditions and the following disclaimer.         */
N/*                                                                           */
N/*     Redistributions in binary form  must reproduce the above copyright    */
N/*     notice, this  list of conditions  and the following  disclaimer in    */
N/*     the  documentation  and/or   other  materials  provided  with  the    */
N/*     distribution.                                                         */
N/*                                                                           */
N/*     Neither the  name of Texas Instruments Incorporated  nor the names    */
N/*     of its  contributors may  be used to  endorse or  promote products    */
N/*     derived  from   this  software  without   specific  prior  written    */
N/*     permission.                                                           */
N/*                                                                           */
N/*  THIS SOFTWARE  IS PROVIDED BY THE COPYRIGHT  HOLDERS AND CONTRIBUTORS    */
N/*  "AS IS"  AND ANY  EXPRESS OR IMPLIED  WARRANTIES, INCLUDING,  BUT NOT    */
N/*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR    */
N/*  A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT    */
N/*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,    */
N/*  SPECIAL,  EXEMPLARY,  OR CONSEQUENTIAL  DAMAGES  (INCLUDING, BUT  NOT    */
N/*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,    */
N/*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY    */
N/*  THEORY OF  LIABILITY, WHETHER IN CONTRACT, STRICT  LIABILITY, OR TORT    */
N/*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE    */
N/*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.     */
N/*                                                                           */
N/*****************************************************************************/
N
N#ifndef _LINKAGE
N#define _LINKAGE
N
N/* No modifiers are needed to access code or data */
N
N#define _CODE_ACCESS
N#define _DATA_ACCESS
N#define _DATA_ACCESS_NEAR
N
N/*--------------------------------------------------------------------------*/
N/* Define _IDECL ==> how inline functions are declared                      */
N/*--------------------------------------------------------------------------*/
N#ifdef _INLINE
S#define _IDECL static __inline
S#define _IDEFN static __inline
N#else
N#define _IDECL _CODE_ACCESS
N#define _IDEFN _CODE_ACCESS
N#endif
N
N#endif /* ifndef _LINKAGE */
L 45 "C:/ti/ccsv6/tools/compiler/c5500_4.4.1/include/file.h" 2
N
N/*---------------------------------------------------------------------------*/
N/* constants for file manipulations                                          */
N/*---------------------------------------------------------------------------*/
N#define  O_RDONLY    (0x0000) /* open for reading      */
N#define  O_WRONLY    (0x0001) /* open for writing      */
N#define  O_RDWR      (0x0002) /* open for read & write */
N#define  O_APPEND    (0x0008) /* append on each write  */
N#define  O_CREAT     (0x0200) /* open with file create */
N#define  O_TRUNC     (0x0400) /* open with truncation  */
N#define  O_BINARY    (0x8000) /* open in binary mode   */
N
N/*---------------------------------------------------------------------------*/
N/* lowlevel I/O declarations                                                 */
N/*---------------------------------------------------------------------------*/
N#ifdef __cplusplus
S#define _DECL extern "C"
N#else /* ! __cplusplus */
N#define _DECL extern
N#endif
N
N#ifndef _OFF_T
N#define _OFF_T
Ntypedef long off_t;
N#endif /* _OFF_T */
N
N#ifndef SEEK_SET
N#define SEEK_SET  (0x0000)
N#endif /* SEEK_SET */
N#ifndef SEEK_CUR
N#define SEEK_CUR  (0x0001)
N#endif /*SEEK_CUR */
N#ifndef SEEK_END
N#define SEEK_END  (0x0002)
N#endif /* SEEK_END */
N
N_DECL _CODE_ACCESS int   open(const char *path, unsigned flags, int mode);
Xextern  int   open(const char *path, unsigned flags, int mode);
N_DECL _CODE_ACCESS int   read(int fildes, char *bufptr, unsigned cnt);
Xextern  int   read(int fildes, char *bufptr, unsigned cnt);
N_DECL _CODE_ACCESS int   write(int fildes, const char *bufptr, unsigned cnt);
Xextern  int   write(int fildes, const char *bufptr, unsigned cnt);
N_DECL _CODE_ACCESS off_t lseek(int fildes, off_t offset, int origin);
Xextern  off_t lseek(int fildes, off_t offset, int origin);
N_DECL _CODE_ACCESS int   close(int fildes);
Xextern  int   close(int fildes);
N_DECL _CODE_ACCESS int   unlink(const char *path);
Xextern  int   unlink(const char *path);
N_DECL _CODE_ACCESS int   rename(const char *old_name, const char *new_name);
Xextern  int   rename(const char *old_name, const char *new_name);
N
N_DECL _CODE_ACCESS int add_device(
Xextern  int add_device(
N    char     *name,			           
N    unsigned  flags,
N    int      (*dopen)(const char *path, unsigned flags, int llv_fd),
N    int      (*dclose)(int dev_fd),
N    int      (*dread)(int dev_fd, char *buf, unsigned count),
N    int      (*dwrite)(int dev_fd, const char *buf, unsigned count),
N    off_t    (*dlseek)(int dev_fd, off_t offset, int origin),
N    int      (*dunlink)(const char *path),
N    int      (*drename)(const char *old_name, const char *new_name));
N
N/*---------------------------------------------------------------------------*/
N/* _NSTREAM defines the max number of files you can have open with fopen().  */
N/* Since the standard streams(stdin/stdout/stderr) use 3 of them by default, */
N/* (_NSTREAM - 3) will be available to users.                                */
N/*---------------------------------------------------------------------------*/
N#define _NSTREAM         10                  
N
N#define _NDEVICE         3                   /* Size of device table        */
N
N#define _SSA      (0x0000)             /* Single Stream allowed       */
N#define _BUSY     (0x0001)             /* Device busy                 */
N#define _MSA      (0x0002)             /* Multiple Streams Allowed    */
N
N#undef _DECL
N
N#endif /* _FILE */
N
L 16 "../inc/ECGDemoNonBios.h" 2
N#include<stdio.h>
L 1 "C:/ti/ccsv6/tools/compiler/c5500_4.4.1/include/stdio.h" 1
N/*****************************************************************************/
N/* STDIO.H v4.4.1                                                            */
N/*                                                                           */
N/* Copyright (c) 1993-2012 Texas Instruments Incorporated                    */
N/* http://www.ti.com/                                                        */
N/*                                                                           */
N/*  Redistribution and  use in source  and binary forms, with  or without    */
N/*  modification,  are permitted provided  that the  following conditions    */
N/*  are met:                                                                 */
N/*                                                                           */
N/*     Redistributions  of source  code must  retain the  above copyright    */
N/*     notice, this list of conditions and the following disclaimer.         */
N/*                                                                           */
N/*     Redistributions in binary form  must reproduce the above copyright    */
N/*     notice, this  list of conditions  and the following  disclaimer in    */
N/*     the  documentation  and/or   other  materials  provided  with  the    */
N/*     distribution.                                                         */
N/*                                                                           */
N/*     Neither the  name of Texas Instruments Incorporated  nor the names    */
N/*     of its  contributors may  be used to  endorse or  promote products    */
N/*     derived  from   this  software  without   specific  prior  written    */
N/*     permission.                                                           */
N/*                                                                           */
N/*  THIS SOFTWARE  IS PROVIDED BY THE COPYRIGHT  HOLDERS AND CONTRIBUTORS    */
N/*  "AS IS"  AND ANY  EXPRESS OR IMPLIED  WARRANTIES, INCLUDING,  BUT NOT    */
N/*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR    */
N/*  A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT    */
N/*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,    */
N/*  SPECIAL,  EXEMPLARY,  OR CONSEQUENTIAL  DAMAGES  (INCLUDING, BUT  NOT    */
N/*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,    */
N/*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY    */
N/*  THEORY OF  LIABILITY, WHETHER IN CONTRACT, STRICT  LIABILITY, OR TORT    */
N/*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE    */
N/*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.     */
N/*                                                                           */
N/*****************************************************************************/
N#ifndef _STDIO 
N#define _STDIO
N
N#include <linkage.h>
N#include <stdarg.h>
L 1 "C:/ti/ccsv6/tools/compiler/c5500_4.4.1/include/stdarg.h" 1
N/****************************************************************************/
N/* stdarg.h v4.4.1                                                          */
N/*                                                                          */
N/* Copyright (c) 1996-2012 Texas Instruments Incorporated                   */
N/* http://www.ti.com/                                                       */
N/*                                                                          */
N/*  Redistribution and  use in source  and binary forms, with  or without   */
N/*  modification,  are permitted provided  that the  following conditions   */
N/*  are met:                                                                */
N/*                                                                          */
N/*     Redistributions  of source  code must  retain the  above copyright   */
N/*     notice, this list of conditions and the following disclaimer.        */
N/*                                                                          */
N/*     Redistributions in binary form  must reproduce the above copyright   */
N/*     notice, this  list of conditions  and the following  disclaimer in   */
N/*     the  documentation  and/or   other  materials  provided  with  the   */
N/*     distribution.                                                        */
N/*                                                                          */
N/*     Neither the  name of Texas Instruments Incorporated  nor the names   */
N/*     of its  contributors may  be used to  endorse or  promote products   */
N/*     derived  from   this  software  without   specific  prior  written   */
N/*     permission.                                                          */
N/*                                                                          */
N/*  THIS SOFTWARE  IS PROVIDED BY THE COPYRIGHT  HOLDERS AND CONTRIBUTORS   */
N/*  "AS IS"  AND ANY  EXPRESS OR IMPLIED  WARRANTIES, INCLUDING,  BUT NOT   */
N/*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR   */
N/*  A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT   */
N/*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,   */
N/*  SPECIAL,  EXEMPLARY,  OR CONSEQUENTIAL  DAMAGES  (INCLUDING, BUT  NOT   */
N/*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,   */
N/*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY   */
N/*  THEORY OF  LIABILITY, WHETHER IN CONTRACT, STRICT  LIABILITY, OR TORT   */
N/*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE   */
N/*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.    */
N/*                                                                          */
N/****************************************************************************/
N
N#ifndef _STDARG
N#define _STDARG
N
N#ifdef __cplusplus
Snamespace std {
N#endif /* __cplusplus */
N
Ntypedef char *va_list;
N
N#ifdef __cplusplus
S} /* namespace std */
N#endif
N
N#if !defined(__TMS320C55X_PLUS_BYTE__)
X#if !0L
N#define va_start(ap, parmN)                     \
N    _Pragma("diag_suppress 238")                \
N 	(ap = ((char *)__va_parmadr(parmN)) + 	\
N	((__va_argref(parmN) ? sizeof(&parmN) : sizeof(parmN))-sizeof(int))) \
N    _Pragma("diag_default 238")
X#define va_start(ap, parmN)                         _Pragma("diag_suppress 238")                 	(ap = ((char *)__va_parmadr(parmN)) + 		((__va_argref(parmN) ? sizeof(&parmN) : sizeof(parmN))-sizeof(int)))     _Pragma("diag_default 238")
N#else
S#define va_start(ap, parmN)                                \
S    _Pragma("diag_suppress 238")                           \
S	(ap = ((char *)__va_parmadr(parmN)) +              \
S	((__va_argref(parmN) ? sizeof(&parmN) :            \
S	    (sizeof(parmN) < 2 ? 2 : sizeof(parmN)))))     \
S    _Pragma("diag_default 238")
X#define va_start(ap, parmN)                                    _Pragma("diag_suppress 238")                           	(ap = ((char *)__va_parmadr(parmN)) +              	((__va_argref(parmN) ? sizeof(&parmN) :            	    (sizeof(parmN) < 2 ? 2 : sizeof(parmN)))))         _Pragma("diag_default 238")
N#endif
N
N#define va_end(ap) ((void)0)
N
N/****************************************************************************/
N/* VA_ARG - Return pointer to the next argument                             */
N/*                                                                          */
N/* Conventions:                                                             */
N/*   1) An argument of size greater then one word is aligned on an even     */
N/*      word boundary.                                                      */
N/*   2) Argument pointer points to the last word of the previous argument   */
N/*      and is updated to point to the last word of the current argument.   */
N/*                                                                          */
N/* When an argument is passed by-reference (indicated by __va_argref being  */
N/* true) the actual argument passed is a pointer and will be treated much   */
N/* the same as a "void *" argument.                                         */
N/*                                                                          */
N/* The first expression of the outer comma expression adds in any necessary */
N/* stack alignment per convention 1) above.                                 */
N/*                                                                          */
N/* The second expression of the outer comma expression increments the       */
N/* argument pointer to point to the last word of the argument and produces  */
N/* the address to use for the argument.                                     */
N/****************************************************************************/
N
N#if !defined(__TMS320C55X_PLUS_BYTE__)
X#if !0L
N#define va_arg(ap, type)  (						     \
N    _Pragma("diag_suppress 238")					     \
N    __va_argref(type) ?							     \
N      ((ap) += (sizeof(void *) > 1 && ( ! ((long)(ap) & 1)))) :		     \
N      ((ap) += (sizeof(type  ) > 1 && ( ! ((long)(ap) & 1)))),		     \
N    __va_argref(type) ?							     \
N      ((ap) += sizeof(void *), (** (type **) ((ap) - (sizeof(void *)-1)))) : \
N      ((ap) += sizeof(type  ), (*  (type *)  ((ap) - (sizeof(type  )-1))))   \
N    _Pragma("diag_default 238")						     \
N    )
X#define va_arg(ap, type)  (						         _Pragma("diag_suppress 238")					         __va_argref(type) ?							           ((ap) += (sizeof(void *) > 1 && ( ! ((long)(ap) & 1)))) :		           ((ap) += (sizeof(type  ) > 1 && ( ! ((long)(ap) & 1)))),		         __va_argref(type) ?							           ((ap) += sizeof(void *), (** (type **) ((ap) - (sizeof(void *)-1)))) :       ((ap) += sizeof(type  ), (*  (type *)  ((ap) - (sizeof(type  )-1))))       _Pragma("diag_default 238")						         )
N#else
S
S/*****************************************************************************/
S/* objects of size "char" are widened to "int" before being passed to a	     */
S/* variadic function.  ints are aligned to 2 bytes, longs and long longs     */
S/* to 4. aggregates are handled as pointers, which in huge model are 4	     */
S/* bytes.								     */
S/*****************************************************************************/
S/* Note: ap is left pointing to the *beginning* of where the next arg might  */
S/* occur								     */
S/*****************************************************************************/
S#define va_arg(ap, type)  (						      \
S    _Pragma("diag_suppress 238")					      \
S    __va_argref(type) ?							      \
S      ((ap) = (char *)(((long)ap + 3) & ~3)) :                                \
S      ((ap) = (char *)                                                        \
S	      ((sizeof(type) >= sizeof(long)) ? (((long)ap + 3) & ~3) :       \
S	       (                                ((long)ap + 1) & ~1))),       \
S    __va_argref(type) ?							      \
S      ((ap) += 4, (** (type **) ((ap) - 4))) :                                \
S      ((ap) += ((sizeof(type) < 2) ? 2 : sizeof(type)),                       \
S       *(type*)((ap) - ((sizeof(type) < 2 ? 2 : sizeof(type)))))              \
S    _Pragma("diag_default 238")						      \
S    )
X#define va_arg(ap, type)  (						          _Pragma("diag_suppress 238")					          __va_argref(type) ?							            ((ap) = (char *)(((long)ap + 3) & ~3)) :                                      ((ap) = (char *)                                                        	      ((sizeof(type) >= sizeof(long)) ? (((long)ap + 3) & ~3) :       	       (                                ((long)ap + 1) & ~1))),           __va_argref(type) ?							            ((ap) += 4, (** (type **) ((ap) - 4))) :                                      ((ap) += ((sizeof(type) < 2) ? 2 : sizeof(type)),                              *(type*)((ap) - ((sizeof(type) < 2 ? 2 : sizeof(type)))))                  _Pragma("diag_default 238")						          )
S
N#endif
N
N#endif /* _STDARG */
N
N#if defined(__cplusplus) && !defined(_CPP_STYLE_HEADER)
X#if 0L && !0L
Susing std::va_list;
N#endif /* _CPP_STYLE_HEADER */
N
L 42 "C:/ti/ccsv6/tools/compiler/c5500_4.4.1/include/stdio.h" 2
N
N/*---------------------------------------------------------------------------*/
N/* Attributes are only available in relaxed ANSI mode.                       */
N/*---------------------------------------------------------------------------*/
N#ifndef __ATTRIBUTE
N#if __TI_STRICT_ANSI_MODE__
X#if 1
N#define __ATTRIBUTE(attr)
N#else
S#define __ATTRIBUTE(attr) __attribute__(attr)
N#endif
N#endif
N
N
N#ifdef __cplusplus
S//----------------------------------------------------------------------------
S// <cstdio> IS RECOMMENDED OVER <stdio.h>.  <stdio.h> IS PROVIDED FOR
S// COMPATIBILITY WITH C AND THIS USAGE IS DEPRECATED IN C++
S//----------------------------------------------------------------------------
Sextern "C" namespace std {
N#endif
N
N/****************************************************************************/
N/* TYPES THAT ANSI REQUIRES TO BE DEFINED                                   */
N/****************************************************************************/
N#ifndef _SIZE_T
N#define _SIZE_T
Ntypedef __SIZE_T_TYPE__ size_t;
Xtypedef unsigned size_t;
N#endif
N
Ntypedef struct {
N      int fd;                    /* File descriptor */
N      unsigned char* buf;        /* Pointer to start of buffer */
N      unsigned char* pos;        /* Position in buffer */
N      unsigned char* bufend;     /* Pointer to end of buffer */
N      unsigned char* buff_stop;  /* Pointer to last read char in buffer */
N      unsigned int   flags;      /* File status flags (see below) */
N} FILE;
N
N#ifndef _FPOS_T
N#define _FPOS_T
Ntypedef long fpos_t;
N#endif /* _FPOS_T */
N
N/****************************************************************************/
N/* DEVICE AND STREAM RELATED MACROS                                         */
N/****************************************************************************/
N/****************************************************************************/
N/* MACROS THAT DEFINE AND USE FILE STATUS FLAGS                             */
N/****************************************************************************/
N
N#define _IOFBF       0x0001
N#define _IOLBF       0x0002
N#define _IONBF       0x0004
N#define _BUFFALOC    0x0008
N#define _MODER       0x0010
N#define _MODEW       0x0020
N#define _MODERW      0x0040
N#define _MODEA       0x0080
N#define _MODEBIN     0x0100
N#define _STATEOF     0x0200
N#define _STATERR     0x0400
N#define _UNGETC      0x0800
N#define _TMPFILE     0x1000
N
N#define _SET(_fp, _b)      (((_fp)->flags) |= (_b))
N#define _UNSET(_fp, _b)    (((_fp)->flags) &= ~(_b))
N#define _STCHK(_fp, _b)    (((_fp)->flags) & (_b))
N#define _BUFFMODE(_fp)     (((_fp)->flags) & (_IOFBF | _IOLBF | _IONBF))
N#define _ACCMODE(_fp)      (((_fp)->flags) & (_MODER | _MODEW))
N
N/****************************************************************************/
N/* MACROS THAT ANSI REQUIRES TO BE DEFINED                                  */
N/****************************************************************************/
N#define BUFSIZ          256 
N
N#define FOPEN_MAX       _NFILE
N#define FILENAME_MAX    256  
N#define TMP_MAX         65535
N
N#define stdin     (&_ftable[0])      
N#define stdout    (&_ftable[1])
N#define stderr    (&_ftable[2])
N
N#define L_tmpnam  _LTMPNAM
N
N
N#define SEEK_SET  (0x0000)
N#define SEEK_CUR  (0x0001)
N#define SEEK_END  (0x0002)
N
N#ifndef NULL
N#define NULL 0
N#endif
N
N#ifndef EOF
N#define EOF    (-1)
N#endif
N
N/******** END OF ANSI MACROS ************************************************/
N
N#define P_tmpdir        ""                   /* Path for temp files         */
N
N/****************************************************************************/
N/* DEVICE AND STREAM RELATED DATA STRUCTURES AND MACROS                     */
N/****************************************************************************/
N#define _NFILE           10                   /* Max number of files open   */
N#define _LTMPNAM         16                   /* Length of temp name        */
N
Nextern _DATA_ACCESS FILE _ftable[_NFILE];
Xextern  FILE _ftable[10];
Nextern _DATA_ACCESS char _tmpnams[_NFILE][_LTMPNAM];
Xextern  char _tmpnams[10][16];
N
N/****************************************************************************/
N/*   FUNCTION DEFINITIONS  - ANSI                                           */
N/****************************************************************************/
N/****************************************************************************/
N/* OPERATIONS ON FILES                                                      */
N/****************************************************************************/
Nextern _CODE_ACCESS int     remove(const char *_file);
Xextern  int     remove(const char *_file);
Nextern _CODE_ACCESS int     rename(const char *_old, const char *_new);
Xextern  int     rename(const char *_old, const char *_new);
Nextern _CODE_ACCESS FILE   *tmpfile(void);
Xextern  FILE   *tmpfile(void);
Nextern _CODE_ACCESS char   *tmpnam(char *_s);
Xextern  char   *tmpnam(char *_s);
N
N/****************************************************************************/
N/* FILE ACCESS FUNCTIONS                                                    */
N/****************************************************************************/
Nextern _CODE_ACCESS int     fclose(FILE *_fp); 
Xextern  int     fclose(FILE *_fp); 
Nextern _CODE_ACCESS FILE   *fopen(const char *_fname, const char *_mode);
Xextern  FILE   *fopen(const char *_fname, const char *_mode);
Nextern _CODE_ACCESS FILE   *freopen(const char *_fname, const char *_mode,
Xextern  FILE   *freopen(const char *_fname, const char *_mode,
N			            register FILE *_fp);
Nextern _CODE_ACCESS void    setbuf(register FILE *_fp, char *_buf);
Xextern  void    setbuf(register FILE *_fp, char *_buf);
Nextern _CODE_ACCESS int     setvbuf(register FILE *_fp, register char *_buf, 
Xextern  int     setvbuf(register FILE *_fp, register char *_buf, 
N			            register int _type, register size_t _size);
Nextern _CODE_ACCESS int     fflush(register FILE *_fp); 
Xextern  int     fflush(register FILE *_fp); 
N
N/****************************************************************************/
N/* FORMATTED INPUT/OUTPUT FUNCTIONS                                         */
N/****************************************************************************/
Nextern _CODE_ACCESS int fprintf(FILE *_fp, const char *_format, ...)
Xextern  int fprintf(FILE *_fp, const char *_format, ...)
N               __ATTRIBUTE ((__format__ (__printf__, 2, 3)));
X               ;
Nextern _CODE_ACCESS int fscanf(FILE *_fp, const char *_fmt, ...)
Xextern  int fscanf(FILE *_fp, const char *_fmt, ...)
N               __ATTRIBUTE ((__format__ (__scanf__, 2, 3)));
X               ;
Nextern _CODE_ACCESS int printf(const char *_format, ...)
Xextern  int printf(const char *_format, ...)
N               __ATTRIBUTE ((__format__ (__printf__, 1, 2)));
X               ;
Nextern _CODE_ACCESS int scanf(const char *_fmt, ...)
Xextern  int scanf(const char *_fmt, ...)
N               __ATTRIBUTE ((__format__ (__scanf__, 1, 2)));
X               ;
Nextern _CODE_ACCESS int sprintf(char *_string, const char *_format, ...)
Xextern  int sprintf(char *_string, const char *_format, ...)
N               __ATTRIBUTE ((__format__ (__printf__, 2, 3)));
X               ;
Nextern _CODE_ACCESS int snprintf(char *_string, size_t _n, 
Xextern  int snprintf(char *_string, size_t _n, 
N				 const char *_format, ...)
N               __ATTRIBUTE ((__format__ (__printf__, 3, 4)));
X               ;
Nextern _CODE_ACCESS int sscanf(const char *_str, const char *_fmt, ...)
Xextern  int sscanf(const char *_str, const char *_fmt, ...)
N               __ATTRIBUTE ((__format__ (__scanf__, 2, 3)));
X               ;
Nextern _CODE_ACCESS int vfprintf(FILE *_fp, const char *_format, va_list _ap)
Xextern  int vfprintf(FILE *_fp, const char *_format, va_list _ap)
N               __ATTRIBUTE ((__format__ (__printf__, 2, 0)));
X               ;
Nextern _CODE_ACCESS int vprintf(const char *_format, va_list _ap)
Xextern  int vprintf(const char *_format, va_list _ap)
N               __ATTRIBUTE ((__format__ (__printf__, 1, 0)));
X               ;
Nextern _CODE_ACCESS int vsprintf(char *_string, const char *_format,
Xextern  int vsprintf(char *_string, const char *_format,
N				 va_list _ap)
N               __ATTRIBUTE ((__format__ (__printf__, 2, 0)));
X               ;
Nextern _CODE_ACCESS int vsnprintf(char *_string, size_t _n, 
Xextern  int vsnprintf(char *_string, size_t _n, 
N				  const char *_format, va_list _ap)
N               __ATTRIBUTE ((__format__ (__printf__, 3, 0)));
X               ;
N
N/****************************************************************************/
N/* CHARACTER INPUT/OUTPUT FUNCTIONS                                         */
N/****************************************************************************/
Nextern _CODE_ACCESS int     fgetc(register FILE *_fp);
Xextern  int     fgetc(register FILE *_fp);
Nextern _CODE_ACCESS char   *fgets(char *_ptr, register int _size,
Xextern  char   *fgets(char *_ptr, register int _size,
N				  register FILE *_fp);
Nextern _CODE_ACCESS int     fputc(int _c, register FILE *_fp);
Xextern  int     fputc(int _c, register FILE *_fp);
Nextern _CODE_ACCESS int     fputs(const char *_ptr, register FILE *_fp);
Xextern  int     fputs(const char *_ptr, register FILE *_fp);
Nextern _CODE_ACCESS int     getc(FILE *_p);
Xextern  int     getc(FILE *_p);
Nextern _CODE_ACCESS int     getchar(void);
Xextern  int     getchar(void);
Nextern _CODE_ACCESS char   *gets(char *_ptr); 
Xextern  char   *gets(char *_ptr); 
Nextern _CODE_ACCESS int     putc(int _x, FILE *_fp);
Xextern  int     putc(int _x, FILE *_fp);
Nextern _CODE_ACCESS int     putchar(int _x);
Xextern  int     putchar(int _x);
Nextern _CODE_ACCESS int     puts(const char *_ptr); 
Xextern  int     puts(const char *_ptr); 
Nextern _CODE_ACCESS int     ungetc(int _c, register FILE *_fp);
Xextern  int     ungetc(int _c, register FILE *_fp);
N
N/****************************************************************************/
N/* DIRECT INPUT/OUTPUT FUNCTIONS                                            */
N/****************************************************************************/
Nextern _CODE_ACCESS size_t  fread(void *_ptr, size_t _size, size_t _count,
Xextern  size_t  fread(void *_ptr, size_t _size, size_t _count,
N				  FILE *_fp);
Nextern _CODE_ACCESS size_t  fwrite(const void *_ptr, size_t _size,
Xextern  size_t  fwrite(const void *_ptr, size_t _size,
N				   size_t _count, register FILE *_fp); 
N
N/****************************************************************************/
N/* FILE POSITIONING FUNCTIONS                                               */
N/****************************************************************************/
Nextern _CODE_ACCESS int     fgetpos(FILE *_fp, fpos_t *_pos);
Xextern  int     fgetpos(FILE *_fp, fpos_t *_pos);
Nextern _CODE_ACCESS int     fseek(register FILE *_fp, long _offset,
Xextern  int     fseek(register FILE *_fp, long _offset,
N				  int _ptrname);
Nextern _CODE_ACCESS int     fsetpos(FILE *_fp, const fpos_t *_pos);
Xextern  int     fsetpos(FILE *_fp, const fpos_t *_pos);
Nextern _CODE_ACCESS long    ftell(FILE *_fp);
Xextern  long    ftell(FILE *_fp);
Nextern _CODE_ACCESS void    rewind(register FILE *_fp); 
Xextern  void    rewind(register FILE *_fp); 
N
N/****************************************************************************/
N/* ERROR-HANDLING FUNCTIONS                                                 */
N/****************************************************************************/
Nextern _CODE_ACCESS void    clearerr(FILE *_fp);
Xextern  void    clearerr(FILE *_fp);
Nextern _CODE_ACCESS int     feof(FILE *_fp);
Xextern  int     feof(FILE *_fp);
Nextern _CODE_ACCESS int     ferror(FILE *_fp);
Xextern  int     ferror(FILE *_fp);
Nextern _CODE_ACCESS void    perror(const char *_s);
Xextern  void    perror(const char *_s);
N
N#define _getchar()      getc(stdin)
N#define _putchar(_x)    putc((_x), stdout)
N#define _clearerr(_fp)   ((void) ((_fp)->flags &= ~(_STATERR | _STATEOF)))
N
N#define _ferror(_x)     ((_x)->flags & _STATERR)
N
N#define _remove(_fl)    (unlink((_fl)))
N
N#ifdef __cplusplus
S} /* extern "C" namespace std */
N#endif  /* __cplusplus */
N
N#endif  /* #ifndef _STDIO */
N
N#if defined(__cplusplus) && !defined(_CPP_STYLE_HEADER)
X#if 0L && !0L
Susing std::size_t;
Susing std::FILE;
Susing std::fpos_t;
Susing std::_ftable;
Susing std::_tmpnams;
Susing std::remove;
Susing std::rename;
Susing std::tmpfile;
Susing std::tmpnam;
Susing std::fclose;
Susing std::fopen;
Susing std::freopen;
Susing std::setbuf;
Susing std::setvbuf;
Susing std::fflush;
Susing std::fprintf;
Susing std::fscanf;
Susing std::printf;
Susing std::scanf;
Susing std::sprintf;
Susing std::snprintf;
Susing std::sscanf;
Susing std::vfprintf;
Susing std::vprintf;
Susing std::vsprintf;
Susing std::vsnprintf;
Susing std::fgetc;
Susing std::fgets;
Susing std::fputc;
Susing std::fputs;
Susing std::getc;
Susing std::getchar;
Susing std::gets;
Susing std::putc;
Susing std::putchar;
Susing std::puts;
Susing std::ungetc;
Susing std::fread;
Susing std::fwrite;
Susing std::fgetpos;
Susing std::fseek;
Susing std::fsetpos;
Susing std::ftell;
Susing std::rewind;
Susing std::clearerr;
Susing std::feof;
Susing std::ferror;
Susing std::perror;
S
N#endif  /* _CPP_STYLE_HEADER */
N
N
L 17 "../inc/ECGDemoNonBios.h" 2
N#include<std.h>
L 1 "C:/ti/bios_5_42_01_09/packages/ti/bios/include/std.h" 1
N/*
N *  Copyright 2012 by Texas Instruments Incorporated.
N *  @(#) DSP/BIOS_Kernel 5,2,5,44 09-06-2012 (cuda-u44)
N */
N/*
N *  ======== std.h ========
N *
N */
N
N#ifndef STD_
N
N#include "tistdtypes.h"
L 1 "C:\ti\bios_5_42_01_09\packages\ti\bios\include\tistdtypes.h" 1
N/*
N *  Copyright 2012 by Texas Instruments Incorporated.
N *  @(#) DSP/BIOS_Kernel 5,2,5,44 09-06-2012 (cuda-u44)
N */
N/*
N *  ======== tistdtypes.h ========
N *
N */
N
N
N/*
N *  These types are also defined by other TI components.  They are bracketed
N *  with _TI_STD_TYPES to avoid warnings for duplicate definition.  The
N *  The Uint16, etc. definitions were provided in early 2.x versions of CSL
N *  that did not have _TI_STD_TYPES protection.
N *
N *  You may get warnings about duplicate type definitions when using this
N *  header file with 2.x CSL.  You can use the '-pds303' compiler option to
N *  suppress these warnings.
N */
N#ifndef _TI_STD_TYPES
N#define _TI_STD_TYPES
N
N/*
N * Aliases for standard C types
N */
Ntypedef int                     Int;
Ntypedef unsigned                Uns;
Ntypedef char                    Char;
N
N/* pointer to null-terminated character sequence */
Ntypedef char                    *String;
N
Ntypedef void                    *Ptr;           /* pointer to arbitrary type */
N
Ntypedef unsigned short          Bool;           /* boolean */
N
N/*
N * Uint8, Uint16, Uint32, etc are defined to be "smallest unit of
N * available storage that is large enough to hold unsigned or integer
N * of specified size".
N */
N
N/* Handle the 6x ISA */
N#if defined(_TMS320C6X)
X#if 0L
S    /* Unsigned integer definitions (32bit, 16bit, 8bit) follow... */
S    typedef unsigned int        Uint32;
S    typedef unsigned short      Uint16;
S    typedef unsigned char       Uint8;
S
S    /* Signed integer definitions (32bit, 16bit, 8bit) follow... */
S    typedef int                 Int32;
S    typedef short               Int16;
S    typedef char                Int8;
S
S/* Handle the 54x, 55x and 28x ISAs */
N#elif defined(_TMS320C5XX) || defined(__TMS320C55X__)
X#elif 0L || 1L
N    /* Unsigned integer definitions (32bit, 16bit, 8bit) follow... */
N    typedef unsigned long       Uint32;
N    typedef unsigned short      Uint16;
N    typedef unsigned char       Uint8;
N
N    /* Signed integer definitions (32bit, 16bit, 8bit) follow... */
N    typedef long                Int32;
N    typedef short               Int16;
N    typedef char                Int8;
N
N#elif defined(_TMS320C28X)
S    /* Unsigned integer definitions (32bit, 16bit, 8bit) follow... */
S    typedef unsigned long       Uint32;
S    typedef unsigned int        Uint16;
S    typedef unsigned char       Uint8;
S
S    /* Signed integer definitions (32bit, 16bit, 8bit) follow... */
S    typedef long                Int32;
S    typedef int                 Int16;
S    typedef char                Int8;
S#else
S    /* Other ISAs not supported */
S    #error <tistdtypes.h> is not supported for this target
N#endif  /* defined(_6x_) */
N
N#endif  /* _TI_STD_TYPES */
L 13 "C:/ti/bios_5_42_01_09/packages/ti/bios/include/std.h" 2
N#include "common_includes.h"
L 1 "../common_inc/common_includes.h" 1
N/*
N * common_includes_vai.h
N *
N *  Created on: 14-Jul-2016
N *      Author: Vuon1
N */
N
N#ifndef COMMON_INC_COMMON_INCLUDES_H_
N#define COMMON_INC_COMMON_INCLUDES_H_
N #ifndef TRUE
N #define TRUE  (1U)
N #define FALSE (0U)
N #endif
N
N
N
N
N
N#endif /* COMMON_INC_COMMON_INCLUDES_VAI_H_ */
L 14 "C:/ti/bios_5_42_01_09/packages/ti/bios/include/std.h" 2
N/*
N *  #include <tistdtypes.h> must be before '#define STD_' to be compatible
N *  with older versions of <tistdtypes.h> file which had conditional logic
N *  based on STD_.  This protects against possible include path inconsistencies
N *  where user might find older <tistdtypes.h> before BIOS's <tistdtypes.h>.
N */
N
N#define STD_
N
N
N#ifdef _TMS320C28X
S#define _28_ 1
S#ifdef __LARGE_MODEL__
S#define _28L_ 1
S#define _28l_ 1         /* deprecated, recommend use _28L_ */
S#endif // __LARGE_MODEL__
N#endif // _TMS320C28X
N
N#if defined(__TMS320C28XX_FPU32__) || defined(__TMS320C28XX_FPU64__)
X#if 0L || 0L
S#define _28FP_ 1
N#endif
N
N#ifdef _TMS320C5XX
S#define _54_ 1
N#endif
N
N#ifdef __TMS320C55X__ 
N#define _55_ 1
N#ifdef __LARGE_MODEL__
N#define _55L_ 1
N#define _55l_ 1         /* deprecated, recommend use _55L_ */
N#endif
N#ifdef __HUGE_MODEL__
S#define _55H_ 1
S#define _VCORE3_ 1
N#endif
N#endif
N
N#ifdef __TMS320C55X_PLUS_BYTE__
S#define _55_  1
S#define _55H_ 1
S#define _55Pb_ 1
S#define _55P_ 1
S#define _VCORE3_ 1
N#endif // __TMS320C55X_PLUS_BYTE__ 
N
N//#ifdef __TMS320C55X_PLUS_WORD__
N//#define _55_  1
N//#define       _55H_ 1
N//#define       _55Pw_ 1
N//#define       _55P_ 1
N//#define _VCORE3_ 1
N//#endif // __TMS320C55X_PLUS_WORD__
N
N#ifdef _TMS320C6200
S#define _62_ 1
S#define _6x_ 1
N#endif
N
N#ifdef _TMS320C6400
S#define _64_ 1
S#define _6x_ 1
N#endif
N
N#ifdef _TMS320C6400_PLUS
S#define _64P_ 1
S#define _6x_ 1
N#endif
N
N#ifdef _TMS320C6700
S#define _67_ 1
S#define _6x_ 1
N#endif
N
N#ifdef _TMS320C6700_PLUS
S#define _67P_ 1
S#define _6x_ 1
N#endif
N
N#ifdef _TMS320C6740
S#define _674_ 1
S#define _6x_ 1
N#endif
N
N/*
N *  ======== _TI_ ========
N *  _TI_ is defined for all TI targets
N */
N#if defined(_54_) || defined(_55_) || defined (_6x_) || defined (_28_)
X#if 0L || 1L || 0L || 0L
N#define _TI_    1
N#endif
N
N/*
N *  ======== _FLOAT_ ========
N *  _FLOAT_ is defined for all targets that natively support floating point
N */
N#if defined(_67_) || defined(_67P_) || defined(_674_)
X#if 0L || 0L || 0L
S#define _FLOAT_ 1
N#else
N#define _FIXED_ 1
N#endif
N
N
N/*
N *  8, 16, 32-bit type definitions
N *
N *  Sm* - 8-bit type
N *  Md* - 16-bit type
N *  Lg* - 32-bit type
N *
N *  *Int - signed type
N *  *Uns - unsigned type
N *  *Bits - unsigned type (bit-maps)
N */
Ntypedef char SmInt;             /* SMSIZE-bit signed integer */
Ntypedef short MdInt;            /* MDSIZE-bit signed integer */
N#if defined(_6x_)
X#if 0L
Stypedef int LgInt;              /* LGSIZE-bit signed integer */
N#else
Ntypedef long LgInt;             /* LGSIZE-bit signed integer */
N#endif
N
Ntypedef unsigned char SmUns;    /* SMSIZE-bit unsigned integer */
Ntypedef unsigned short MdUns;   /* MDSIZE-bit unsigned integer */
N#if defined(_6x_)
X#if 0L
Stypedef unsigned LgUns;         /* LGSIZE-bit unsigned integer */
N#else
Ntypedef unsigned long LgUns;    /* LGSIZE-bit unsigned integer */
N#endif
N
Ntypedef unsigned char SmBits;   /* SMSIZE-bit bit string */
Ntypedef unsigned short MdBits;  /* MDSIZE-bit bit string */
N#if defined(_6x_)
X#if 0L
Stypedef unsigned LgBits;        /* LGSIZE-bit bit string */
N#else
Ntypedef unsigned long LgBits;   /* LGSIZE-bit bit string */
N#endif
N
N/* avoid conflicting with xdc/std.h */
N#ifndef xdc_std__include
N
Ntypedef long int Long;
Ntypedef short int Short;
Ntypedef SmBits Byte;            /* smallest unit of addressable store */
N
N#define Void void
N
N/* Arg should be size of Ptr */
N#if defined(_54_) || defined(_6x_)
X#if 0L || 0L
Stypedef Int Arg;
N#elif defined(_55_) || defined(_28_)
X#elif 1L || 0L
Ntypedef void *Arg;
N#else
S/* Other ISAs not supported */
S#error <std.h> types not supported for this target
N#endif
N
Ntypedef Int (*Fxn)();           /* generic function type */
N
Ntypedef float Float;
N
N#ifndef NULL
S#define NULL 0
N#endif
N
N
N
N#endif /* xdc_std__include */
N
N/*
N * These macros are used to cast 'Arg' types to 'Int' or 'Ptr'.
N * These macros were added for the 55x since Arg is not the same
N * size as Int and Ptr in 55x large model.
N */
N#if defined(_28L_) || defined(_55L_) || defined(_55H_)
X#if 0L || 1L || 0L
N#define ArgToInt(A)     ((Int)((long)(A) & 0xffff))
N#define ArgToPtr(A)     ((Ptr)(A))
N#else
S#define ArgToInt(A)     ((Int)(A))
S#define ArgToPtr(A)     ((Ptr)(A))
N#endif
N
N#endif /* STD_ */
L 18 "../inc/ECGDemoNonBios.h" 2
N#include "psp_common.h"
L 1 "..\inc\psp_common.h" 1
N/******************************************************************************
N**File Name			:psp_common.h
N**File Description	:Generic interface definitions
N**Created By		:
N**Creation Date		:
N**Last modified date:
N**Remarks			:
N*******************************************************************************/
N
N#ifndef _PSP_COMMON_H_
N#define _PSP_COMMON_H_
N
N#include <tistdtypes.h>
L 1 "../common_inc/tistdtypes.h" 1
N/*****************************************************************************
N * File Name : tistdtypes.h 
N *                                                                              
N * Brief	 : These types are also defined by other TI components.  They are bracketed
N * with _TI_STD_TYPES to avoid warnings for duplicate definition.
N *
N * Copyright (C) 2009 -2010 Texas Instruments Incorporated - http://www.ti.com/ 
N *
N * 
N *  Redistribution and use in source and binary forms, with or without 
N *  modification, are permitted provided that the following conditions 
N *  are met:
N *
N *    Redistributions of source code must retain the above copyright 
N *    notice, this list of conditions and the following disclaimer.
N *
N *    Redistributions in binary form must reproduce the above copyright
N *    notice, this list of conditions and the following disclaimer in the 
N *    documentation and/or other materials provided with the   
N *    distribution.
N *
N *    Neither the name of Texas Instruments Incorporated nor the names of
N *    its contributors may be used to endorse or promote products derived
N *    from this software without specific prior written permission.
N *
N *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
N *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
N *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
N *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
N *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
N *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
N *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
N *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
N *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
N *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
N *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
N *
N******************************************************************************/
N#ifndef _TI_STD_TYPES
S#define _TI_STD_TYPES
S
S/*
S * This '#ifndef STD_' is needed to protect from duplicate definitions
S * of Int, Uns, etc. in DSP/BIOS v4.x (e.g. 4.90, 4.80) since these versions
S * of DSP/BIOS did not contain the '#ifndef_TI_STD_TYPES' logic.
S */
S#ifndef STD_
S
S/*
S * Aliases for standard C types
S */
Stypedef int                 Int;
Stypedef unsigned            Uns;
Stypedef char                Char;
S
S/* pointer to null-terminated character sequence */
Stypedef char                *String;
S                            
Stypedef void                *Ptr;       /* pointer to arbitrary type */
S                            
Stypedef unsigned short      Bool;       /* boolean */
S
S#endif /* STD_ */
S
S/*
S * Uint8, Uint16, Uint32, etc are defined to be "smallest unit of
S * available storage that is large enough to hold unsigned or integer
S * of specified size".
S */
S #ifndef TRUE
S  #define TRUE  1
S  #define FALSE 0
S#endif
S
S
S/* Handle the 6x ISA */
S#if defined(_TMS320C6X)
S    /* Unsigned integer definitions (32bit, 16bit, 8bit) follow... */
S    typedef unsigned int    Uint32;
S    typedef unsigned short  Uint16;
S    typedef unsigned char   Uint8;
S
S    /* Signed integer definitions (32bit, 16bit, 8bit) follow... */
S    typedef int             Int32;
S    typedef short           Int16;
S    typedef char            Int8;
S                            
S/* Handle the 54x, 55x and 28x ISAs */
S#elif defined(_TMS320C5XX) || defined(__TMS320C55X__) || defined(_TMS320C28X)
S    /* Unsigned integer definitions (32bit, 16bit, 8bit) follow... */
S    typedef unsigned long   Uint32;
S    typedef unsigned short  Uint16;
S    typedef unsigned char   Uint8;
S
S    /* Signed integer definitions (32bit, 16bit, 8bit) follow... */
S    typedef long            Int32;
S    typedef short           Int16;
S    typedef char            Int8;
S
S#else
S    /* Other ISAs not supported */
S    #error <tistdtypes.h> is not supported for this target
S#endif  /* defined(_6x_) */
S
N#endif  /* _TI_STD_TYPES */
L 14 "..\inc\psp_common.h" 2
N
N#define PSP_SOK                 (0)
N#define PSP_SINPROGRESS         (1)
N#define PSP_E_DRIVER_INIT       (-1)
N#define PSP_E_NO_MEMORY         (-2)
N#define PSP_E_RESOURCES         (-3)
N#define PSP_E_INVAL_STATE       (-4)
N#define PSP_E_INVAL_PARAM       (-5)
N#define PSP_E_NOT_SUPPORTED     (-6)
N#define PSP_E_IO_CANCEL_FAIL    (-7)
N#define PSP_E_FIFO_NOT_ENABLED  (-8)
N#define PSP_E_INVALID_MODE      (-9)
N#define PSP_E_INVALID_CMD       (-10)
N#define PSP_E_TIMEOUT           (-11)
N
N#ifndef True
N#define True    (TRUE)
N#define False   (FALSE)
N#endif
N
N#ifndef NULL
S#define NULL 0
N#endif
N
N/* Type Macros */
N#define PAL_False       ((Bool)0)
N#define PAL_True        ((Bool)1)
N
N/* General Macros */
N#define PAL_MAX(a,b)    ((a) > (b) ? (a) : (b))
N#define PAL_MIN(a,b)    ((a) < (b) ? (a) : (b))
N
N/* Array Dimension */
N#define PAL_DIM(array)  ((sizeof(array))/(sizeof(array[0])))
N
N/* Endianness */
N
N#define PAL_MK_UINT16(high8,low8)                               \
N    ((Uint16)( ((Uint16)(high8) << 8) | (Uint16)(low8) ))
X#define PAL_MK_UINT16(high8,low8)                                   ((Uint16)( ((Uint16)(high8) << 8) | (Uint16)(low8) ))
N
N#define PAL_UINT16_LOW8(a)                                      \
N    ((Uint8)((a) & 0x00FF))
X#define PAL_UINT16_LOW8(a)                                          ((Uint8)((a) & 0x00FF))
N
N#define PAL_UINT16_HIGH8(a)                                     \
N    ((Uint8)(((a) >> 8) & 0x00FF))
X#define PAL_UINT16_HIGH8(a)                                         ((Uint8)(((a) >> 8) & 0x00FF))
N
N#define PAL_MK_UINT32(high16,low16)                             \
N    ((Uint32)( ((Uint32)(high16) << 16) | (Uint32)(low16) ))
X#define PAL_MK_UINT32(high16,low16)                                 ((Uint32)( ((Uint32)(high16) << 16) | (Uint32)(low16) ))
N
N#define PAL_MK_UINT32_FROM8S(high8,med_high8,med_low8,low8)     \
N    PAL_MK_UINT32(PAL_MK_UINT16(high8,med_high8), PAL_MK_UINT16(med_low8, low8))
X#define PAL_MK_UINT32_FROM8S(high8,med_high8,med_low8,low8)         PAL_MK_UINT32(PAL_MK_UINT16(high8,med_high8), PAL_MK_UINT16(med_low8, low8))
N
N#define PAL_UINT32_LOW16(u32)                                   \
N    ((Uint16)((u32) & 0xFFFF))
X#define PAL_UINT32_LOW16(u32)                                       ((Uint16)((u32) & 0xFFFF))
N
N#define PAL_UINT32_HIGH16(u32)                                  \
N    ((Uint16)(((u32) >> 16) & 0xFFFF))
X#define PAL_UINT32_HIGH16(u32)                                      ((Uint16)(((u32) >> 16) & 0xFFFF))
N
N#define PAL_UINT32_LOW8(u32)                                    \
N    ((Uint8)((u32) & 0x00FF))
X#define PAL_UINT32_LOW8(u32)                                        ((Uint8)((u32) & 0x00FF))
N
N#define PAL_UINT32_MED_LOW8(u32)                                \
N    ((Uint8)(((u32) >> 8) & 0xFF))
X#define PAL_UINT32_MED_LOW8(u32)                                    ((Uint8)(((u32) >> 8) & 0xFF))
N
N#define PAL_UINT32_MED_HIGH8(u32)                               \
N    ((Uint8)(((u32) >> 16) & 0xFF))
X#define PAL_UINT32_MED_HIGH8(u32)                                   ((Uint8)(((u32) >> 16) & 0xFF))
N
N#define PAL_UINT32_HIGH8(u32)                                   \
N    ((Uint8)(((u32) >> 24) & 0xFF))
X#define PAL_UINT32_HIGH8(u32)                                       ((Uint8)(((u32) >> 24) & 0xFF))
N
N#define PAL_SWAP_UINT16(w)      \
N    (PAL_MK_UINT16(PAL_UINT16_LOW8(w), PAL_UINT16_HIGH8(w)))
X#define PAL_SWAP_UINT16(w)          (PAL_MK_UINT16(PAL_UINT16_LOW8(w), PAL_UINT16_HIGH8(w)))
N
N#define PAL_SWAP_UINT32(u32)                \
N    (PAL_MK_UINT32_FROM8S(                  \
N        PAL_UINT32_LOW8(u32),               \
N        PAL_UINT32_MED_LOW8(u32),           \
N        PAL_UINT32_MED_HIGH8(u32),          \
N        PAL_UINT32_HIGH8(u32)))
X#define PAL_SWAP_UINT32(u32)                    (PAL_MK_UINT32_FROM8S(                          PAL_UINT32_LOW8(u32),                       PAL_UINT32_MED_LOW8(u32),                   PAL_UINT32_MED_HIGH8(u32),                  PAL_UINT32_HIGH8(u32)))
N
N/** Endian Utility Macros
N * PAL_UINT16_LE(w) converts a Little-Endian 16bit word to current endian word
N * PAL_UINT16_BE(w) converts a Big-Endian 16bit word to current endian word
N * PAL_UINT32_LE(d) converts a Little-Endian 32bit dword to current endian dword
N * PAL_UINT32_BE(d) converts a Big-Endian 32bit dword to current endian dword
N */
N
N#ifdef PAL_NATIVE_ENDIAN_BIG
S/* Native CPU accesses to memory locations are big-endian style */
S#define PAL_UINT16_LE(w)    PAL_SWAP_UINT16(w)
S#define PAL_UINT16_BE(w)    (w)
S#define PAL_UINT32_LE(d)    PAL_SWAP_UINT32(d)
S#define PAL_UINT32_BE(d)    (d)
S
N#else
N/* Native CPU accesses to memory locations are little-endian style */
N#define PAL_UINT16_LE(w)    (w)
N#define PAL_UINT16_BE(w)    PAL_SWAP_UINT16(w)
N#define PAL_UINT32_LE(d)    (d)
N#define PAL_UINT32_BE(d)    PAL_SWAP_UINT32(d)
N
N#endif /* Endian switch */
N
N/**
N * \defgroup PALErrorCodes PAL Error Codes
N *
N * PAL Error code bit fields follow a standard format. This format is used by
N * all PAL components, Services and Device Drivers.
N *
N * \Note: IMP: This was 32-bit in the original implementation. Was reduced to
N *             16-bit for use on 16-bit machines.
N *
N * The following bitfield diagram depicts the PAL error code format:
N * \n
N * |<----------------16----------------->|
N * \n
N * |1(A)| 2(B) | 2(C) | 3(D)  |   8(E)   |
N * - A - MSB - Set if Error / 0 if Success
N * - B - (SEVERITY) Error level - 0=Informational, 1=Warning, 2=Minor, 3=Major, 4=Critical
N * - C - (SRC) PSP Architecture Component - 0=Reserved, 1=CSL, 2=Driver, 3=PAL, 4=SRV etc
N * - D - (QUAL) Device specific - eg Instance Id of DDC, PAL component (e.g. OSMEM etc)
N * - E - Error number - based upon implementation.
N */
N
N/*@{*/
N
N/** Error severity levels  */
N#define PAL_INFO                (0)
N#define PAL_WARNING             (1)
N#define PAL_MINOR_ERROR         (2)
N#define PAL_MAJOR_ERROR         (3)
N#define PAL_CRITICAL_ERROR      (4)
N
N/** PAL Error Sources (PSP Architectural Components) */
N#define PAL_ERROR_SRC_CSL       (0)
N#define PAL_ERROR_SRC_DRV       (1)
N#define PAL_ERROR_SRC_PAL       (2)
N#define PAL_ERROR_SRC_SRV       (3)
N
N#define PAL_ERROR_FLAG          (0x8000)    /**< PAL Error occured sentinel flag */
N
N/** Successful Return Code for PAL_Result */
N
N#define PAL_SOK                 (0x0)
N#define PAL_SINPROGRESS         (0x1)
N
N
N/**
N * \note PAL Error bit manipulation masks and shift values
N * Adjusted for the new 16-bit error codes.
N */
N#define PAL_ERROR_SEVERITY_SHIFT    (13)
N#define PAL_ERROR_SEVERITY_MASK     (0x6000)
N
N#define PAL_ERROR_SRC_SHIFT         (11)
N#define PAL_ERROR_SRC_MASK          (0x1800)
N
N#define PAL_ERROR_QUAL_SHIFT        (8)
N#define PAL_ERROR_QUAL_MASK         (0x0700)
N
N#define PAL_ERROR_NUM_SHIFT         (0)
N#define PAL_ERROR_NUM_MASK          (0xFF)
N
N/**
N * \brief PAL_ERROR() macro composes a final 16-bit error code per the
N * above described format. It inputs the severity, source of error,
N * source qualifier and the specific error number of interest
N *
N * \Note: Adjusted for 16-bit machines.
N *
N * \sa PAL_ERROR_CSLSTATUS()
N */
N#define PAL_ERROR(severity, src, qual, num) \
N    (PAL_ERROR_FLAG                                                       | \
N    (PAL_ERROR_SEVERITY_MASK & ( (severity) << PAL_ERROR_SEVERITY_SHIFT)) | \
N    (PAL_ERROR_SRC_MASK      & ( (src)      << PAL_ERROR_SRC_SHIFT))      | \
N    (PAL_ERROR_QUAL_MASK     & ( (qual)     << PAL_ERROR_QUAL_SHIFT))     | \
N    (PAL_ERROR_NUM_MASK      & ( (num)      << PAL_ERROR_NUM_SHIFT)))
X#define PAL_ERROR(severity, src, qual, num)     (PAL_ERROR_FLAG                                                       |     (PAL_ERROR_SEVERITY_MASK & ( (severity) << PAL_ERROR_SEVERITY_SHIFT)) |     (PAL_ERROR_SRC_MASK      & ( (src)      << PAL_ERROR_SRC_SHIFT))      |     (PAL_ERROR_QUAL_MASK     & ( (qual)     << PAL_ERROR_QUAL_SHIFT))     |     (PAL_ERROR_NUM_MASK      & ( (num)      << PAL_ERROR_NUM_SHIFT)))
N
N/**
N * \brief The following 4 macros allow to extract relevant portions of
N * the 32bit PAL error code.
N */
N#define PAL_ERROR_NUM(code)         (( (code) & PAL_ERROR_NUM_MASK) >> PAL_ERROR_NUM_SHIFT)
N#define PAL_ERROR_QUAL(code)        (( (code) & PAL_ERROR_QUAL_MASK) >> PAL_ERROR_QUAL_SHIFT)
N#define PAL_ERROR_SRC(code)         (( (code) & PAL_ERROR_SRC_MASK) >> PAL_ERROR_SRC_SHIFT)
N#define PAL_ERROR_SEVERITY(code)    (( (code) & PAL_ERROR_SEVERITY_MASK) >> PAL_ERROR_SEVERITY_SHIFT)
N
N/**
N * \brief   PAL_ERROR_CSLSTATUS() macros constructs a fully embodied
N *      32bit PAL error code, given a 16bit CSL_Status number
N * \note    CSL Errors are always ascribed severity level MAJOR.
N */
N#define PAL_ERROR_CSLSTATUS(cslerr) \
N    PAL_ERROR(PAL_MAJOR_ERROR, PAL_ERROR_SRC_CSL, 0, ( (cslerr) & 0x7F))
X#define PAL_ERROR_CSLSTATUS(cslerr)     PAL_ERROR(PAL_MAJOR_ERROR, PAL_ERROR_SRC_CSL, 0, ( (cslerr) & 0x7F))
N
N/*@}*/
N/*@}*/
N
N
N
N/* IEEE 754 single-precision floating point*/
Ntypedef float   float32_t;
Ntypedef int     PSP_Result;
Ntypedef void    *PSP_Handle;
Ntypedef unsigned int    Uint;   /**< Unsigned base integer quantity */
N
Ntypedef void (*PSP_AppCallback)(void *cbkContext, char *buf, int xferSize);
N/**
N * \defgroup PALDefines PAL Defines
N *
N *  PAL Generic Defines - used by all modules using PAL services
N *  (including Drivers)
N */
N/*@{*/
N
N/* PAL Result - return value of a function  */
N typedef Int             PAL_Result;
N
N
N /**
N * \defgroup Enumerations for driver operating modes
N *
N *  Driver operation modes enumberation
N */
N/*@{*/
Ntypedef enum
N{
N    /** Polled operation mode */
N    PSP_OPMODE_POLLED       = 0,
N    /** Interrupt mode of operation */
N    PSP_OPMODE_INTERRUPT    = 1,
N    /** DMA Mode of operation, DMA interrupts will be used for DMA completion */
N    PSP_OPMODE_DMAINTERRUPT = 2
N}PSP_OpMode;
N
N
N /**
N * \defgroup Driver states
N *
N *  Enumerations indicating the state of the drivers
N */
N/*@{*/
Ntypedef enum
N{
N    /** Indicates that the driver is in deleted state */
N    PSP_DRIVER_STATE_DELETED,
N    /** Indicates that the driver is in created state */
N    PSP_DRIVER_STATE_CREATED,
N    /** Indicates that the driver has been initialized */
N    PSP_DRIVER_STATE_INITIALIZED,
N    /** Indicates that the driver has been opened */
N    PSP_DRIVER_STATE_OPENED,
N    /** Indicates that the driver is closed */
N    PSP_DRIVER_STATE_CLOSED,
N    /** Indicates that the driver has veen de-initialized */
N    PSP_DRIVER_STATE_DEINITIALIZED,
N    /** Indicates that the driver has powered down the device */
N    PSP_DRIVER_STATE_POWERED_DOWN
N} PSP_DriverState;
N
N#endif  /* _PSP_COMMON_H_ */
N/*EOF*/
L 19 "../inc/ECGDemoNonBios.h" 2
N#include "cpu_clock_init.h"
L 1 "..\inc\cpu_clock_init.h" 1
N/******************************************************************************
N**File Name			: cpu_clock_init.h
N**File Description	:header file - contains macros for bits in PLL clk registers.
N**Created By		:
N**Creation Date		:
N**Last modified date:
N**Remarks			:
N*******************************************************************************/
N#ifndef CPU_CLOCK_INIT_H_
N#define CPU_CLOCK_INIT_H_
N
N/* Clock configuration MSW */
N#define SYS_PLL_BYPASS                      (0xFFFE)
N/**< PLL Bypass mode mask                                                   */
N#define SYS_PLL_SEL                         (0x0001)
N/**< PLL selection mask                                                     */
N
N/* pll control LSW */
N#define PLL_MULT_MASK                       (0x03FF)
N/**< PLL multiplier mask                                                    */
N
N/* pll control MSW */
N#define PLL_DIV_MASK                        (0x0FFF)
N/**< PLL Divisor mask                                                       */
N
N#define PLL_PWR_DWN                         (0xEFFF)
N/**< PLL powerdown mask                                                     */
N#define PLL_ERROR_INCORRECT_INPUT           (-1)
N/**< PLL update error                                                       */
N#define PLL_MAX_VP_FOR_100MHz               (779)
N/**< Max VP value for 100 MHz clock frequency                               */
N#define PLL_RP_BYPASS_ENABLE_MASK           (0x8000)
N/**< RP Bypass enable Mask                                                  */
N#define PLL_OP_DIV_BYPASS_MASK              (0x0100)
N/**< O/P Div bypass Mask                                                    */
N#define PLL_OP_DIV2_BYPASS_MASK             (0x0200)
N/**< O/P Div_2 bypass Mask                                                  */
N#define PLL_RP_BYPASS_ENABLE_SHIFT          (15)
N/**< RP Bypass enable shift                                                 */
N#define PLL_OP_DIV_BYPASS_SHIFT             (8)
N/**< O/P Div bypass shift                                                   */
N#define PLL_OP_DIV2_BYPASS_SHIFT            (9)
N/**< O/P Div_2 bypass shift                                                 */
N#define PLL_OP_DIV_MASK                     (0x003F)
N/**< O/P Div Mask                                                           */
N#define PLL_VS_MASK                         (0x0003)
N/**< VS Mask                                                                */
N#define PLL_VS_SHIFT                        (12)
N/**< VS Shift                                                               */
N#define INPUT_CLK                           (12000)
N/**< Input clock - 12 MHz                                                   */
N
N#define SYS_CLK_NUMERATOR(MULT,VS)          ((INPUT_CLK)*((((MULT)<<2)+(VS))+4))
N/**< Numerator of System clock formula                                      */
N
N#define SYS_CLK_MAX                         (100)
N/**< Max system clock - 100 MHz                                             */
N
N#define CLK_CONFIG_MSW_ADDR                 ((ioport volatile unsigned*)0x1C1F)
N/**< Clock configuration register MSW                                       */
N#define PLL_CNTL_LSW_ADDR                   ((ioport volatile unsigned*)0x1C20)
N/**< PLL control register LSW                                               */
N#define PLL_CNTL_MSW_ADDR                   ((ioport volatile unsigned*)0x1C21)
N/**< PLL control register MSW                                               */
N#define PLL_OP_CTRL_REG_ADDR                ((ioport volatile unsigned*)0x1C23)
N/**< PLL output control register                                            */
N
N/**
N *  \brief Initialize the PLL
N *
N *  Sets the CPU clock by modifying the PLL control and clock
N *  configuration registers. Enable the PLL and wait for locking.
N *
N *  \note:
N *   - The DSP maximum operating frequency is 100MHz @ 1.3V.
N *   - The input to the VCO has to fall between 30KHz and 170KHz.
N *
N *  The PLL input clock supports 32KHz to 40MHz input frequency, but the
N *  reference divider must ensure that the input to the Phase Detector
N *  falls between 30KHz and 170KHz.
N *
N *  Refer to the formula in section 9.8.1.4.1, on page 53 of Corazon spec v1.11
N *  on how system clock is generated.
N *
N *
N * The API returns error, if the systemClock output exceeds 100 MHz. Else, the
N * parameters are allowed and set.
N *
N * \param   pllmult [IN]    PLL Multiplier value
N * \param   plldiv  [IN]    PLL Divisor value
N *
N * \return Success, if the system clock doesn't go past 100MHz for pllmult and plldiv,
N *         failure otherwise.
N */
NPSP_Result CpuClockInit(int pllmult ,int plldiv);
N
N#endif
N
N /*EOF*/
L 20 "../inc/ECGDemoNonBios.h" 2
N#include "dda_spi.h"
L 1 "..\inc\dda_spi.h" 1
N/******************************************************************************
N**File Name			: dda_spi.h
N**File Description	:header file for SPI driver adaptation
N**Created By		:
N**Creation Date		:
N**Last modified date:
N**Remarks			:
N*******************************************************************************/
N
N
N#ifndef _DDA_SPI_H_
N#define _DDA_SPI_H_
N
N#include "ddc_spi.h"
L 1 "..\inc\ddc_spi.h" 1
N/******************************************************************************
N**File Name			: ddc_spi.h
N**File Description	:header file for SPI implementation
N**Created By		:
N**Creation Date		:
N**Last modified date:
N**Remarks			:
N*******************************************************************************/
N
N
N#ifndef _DDC_SPI_H_
N#define _DDC_SPI_H_
N
N#include "llc_spi.h"
L 1 "..\inc\llc_spi.h" 1
N/******************************************************************************
N**File Name			: llc_spi.h
N**File Description	:header file for SPI LLC implementation
N**Created By		:
N**Creation Date		:
N**Last modified date:
N**Remarks			:
N*******************************************************************************/
N
N
N#ifndef _LLC_SPI_H_
N#define _LLC_SPI_H_
N
N#include <tistdtypes.h>
N#include <psp_common.h>
N#include "cslr_spi_001.h"
L 1 "../common_inc/cslr_spi_001.h" 1
N/*****************************************************************************
N * File Name : cslr_spi_001.h 
N *
N * Brief	 : Define SPI register structure
N *
N * Copyright (C) 2009 -2010 Texas Instruments Incorporated - http://www.ti.com/ 
N * 
N * 
N *  Redistribution and use in source and binary forms, with or without 
N *  modification, are permitted provided that the following conditions 
N *  are met:
N *
N *    Redistributions of source code must retain the above copyright 
N *    notice, this list of conditions and the following disclaimer.
N *
N *    Redistributions in binary form must reproduce the above copyright
N *    notice, this list of conditions and the following disclaimer in the 
N *    documentation and/or other materials provided with the   
N *    distribution.
N *
N *    Neither the name of Texas Instruments Incorporated nor the names of
N *    its contributors may be used to endorse or promote products derived
N *    from this software without specific prior written permission.
N *
N *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
N *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
N *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
N *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
N *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
N *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
N *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
N *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
N *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
N *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
N *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
N *
N******************************************************************************/
N#ifndef _CSLR__SPI_1_H_
N#define _CSLR__SPI_1_H_
N
N#include <cslr.h>
L 1 "../common_inc/cslr.h" 1
N/*****************************************************************************
N * File Name : cslr.h 
N *
N * Brief	 : Register layer central -- contains field-manipulation macro definitions
N *
N * Copyright (C) 2009 -2010 Texas Instruments Incorporated - http://www.ti.com/ 
N * 
N * 
N *  Redistribution and use in source and binary forms, with or without 
N *  modification, are permitted provided that the following conditions 
N *  are met:
N *
N *    Redistributions of source code must retain the above copyright 
N *    notice, this list of conditions and the following disclaimer.
N *
N *    Redistributions in binary form must reproduce the above copyright
N *    notice, this list of conditions and the following disclaimer in the 
N *    documentation and/or other materials provided with the   
N *    distribution.
N *
N *    Neither the name of Texas Instruments Incorporated nor the names of
N *    its contributors may be used to endorse or promote products derived
N *    from this software without specific prior written permission.
N *
N *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
N *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
N *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
N *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
N *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
N *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
N *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
N *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
N *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
N *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
N *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
N *
N******************************************************************************/
N
N/* Register layer central -- contains field-manipulation macro definitions */
N
N#ifndef _CSLR_H_
N#define _CSLR_H_
N
N/* the "expression" macros */
N
N/* the Field MaKe macro */
N#define CSL_FMK(PER_REG_FIELD, val)                                         \
N    (((val) << CSL_##PER_REG_FIELD##_SHIFT) & CSL_##PER_REG_FIELD##_MASK)
X#define CSL_FMK(PER_REG_FIELD, val)                                             (((val) << CSL_##PER_REG_FIELD##_SHIFT) & CSL_##PER_REG_FIELD##_MASK)
N
N/* the Field EXTract macro */
N#define CSL_FEXT(reg, PER_REG_FIELD)                                        \
N    (((reg) & CSL_##PER_REG_FIELD##_MASK) >> CSL_##PER_REG_FIELD##_SHIFT)
X#define CSL_FEXT(reg, PER_REG_FIELD)                                            (((reg) & CSL_##PER_REG_FIELD##_MASK) >> CSL_##PER_REG_FIELD##_SHIFT)
N
N/* the Field INSert macro */
N#define CSL_FINS(reg, PER_REG_FIELD, val)                                   \
N    ((reg) = ((reg) & ~CSL_##PER_REG_FIELD##_MASK)                          \
N    | CSL_FMK(PER_REG_FIELD, val))
X#define CSL_FINS(reg, PER_REG_FIELD, val)                                       ((reg) = ((reg) & ~CSL_##PER_REG_FIELD##_MASK)                              | CSL_FMK(PER_REG_FIELD, val))
N
N
N/* the "token" macros */
N
N/* the Field MaKe (Token) macro */
N#define CSL_FMKT(PER_REG_FIELD, TOKEN)                                      \
N    CSL_FMK(PER_REG_FIELD, CSL_##PER_REG_FIELD##_##TOKEN)
X#define CSL_FMKT(PER_REG_FIELD, TOKEN)                                          CSL_FMK(PER_REG_FIELD, CSL_##PER_REG_FIELD##_##TOKEN)
N
N/* the Field INSert (Token) macro */
N#define CSL_FINST(reg, PER_REG_FIELD, TOKEN)                                \
N    CSL_FINS((reg), PER_REG_FIELD, CSL_##PER_REG_FIELD##_##TOKEN)
X#define CSL_FINST(reg, PER_REG_FIELD, TOKEN)                                    CSL_FINS((reg), PER_REG_FIELD, CSL_##PER_REG_FIELD##_##TOKEN)
N
N
N/* the "raw" macros */
N
N/* the Field MaKe (Raw) macro */
N#define CSL_FMKR(msb, lsb, val)                                             \
N    (((val) & ((1 << ((msb) - (lsb) + 1)) - 1)) << (lsb))
X#define CSL_FMKR(msb, lsb, val)                                                 (((val) & ((1 << ((msb) - (lsb) + 1)) - 1)) << (lsb))
N
N/* the Field EXTract (Raw) macro */
N#define CSL_FEXTR(reg, msb, lsb)                                            \
N    (((reg) >> (lsb)) & ((1 << ((msb) - (lsb) + 1)) - 1))
X#define CSL_FEXTR(reg, msb, lsb)                                                (((reg) >> (lsb)) & ((1 << ((msb) - (lsb) + 1)) - 1))
N
N/* the Field INSert (Raw) macro */
N#define CSL_FINSR(reg, msb, lsb, val)                                       \
N    ((reg) = ((reg) &~ (((1 << ((msb) - (lsb) + 1)) - 1) << (lsb)))         \
N    | CSL_FMKR(msb, lsb, val))
X#define CSL_FINSR(reg, msb, lsb, val)                                           ((reg) = ((reg) &~ (((1 << ((msb) - (lsb) + 1)) - 1) << (lsb)))             | CSL_FMKR(msb, lsb, val))
N
N#define CSL_Write_Reg(reg, val) (reg = val)
N
N#define CSL_Read_Reg(reg, val) (val = reg)
N    
N#endif /* _CSLR_H_ */
N
L 42 "../common_inc/cslr_spi_001.h" 2
N
N/* Minimum unit = 2 bytes */
N
N/**************************************************************************\
N* Register Overlay Structure
X
N\**************************************************************************/
Ntypedef struct  {
N    volatile Uint16 SPICC1;
N    volatile Uint16 SPICC2;
N    volatile Uint16 SPIDC1;
N    volatile Uint16 SPIDC2;
N    volatile Uint16 SPICR1;
N    volatile Uint16 SPICR2;
N    volatile Uint16 SPISR1;
N    volatile Uint16 SPISR2;
N    volatile Uint16 SPIDR1;
N    volatile Uint16 SPIDR2;
N} CSL_SpiRegs;
N
N/**************************************************************************\
N* Field Definition Macros
X
N\**************************************************************************/
N
N/* SPICC1 */
N
N#define CSL_SPI_SPICC1_DCLK_DIV_MASK (0xFFFFu)
N#define CSL_SPI_SPICC1_DCLK_DIV_SHIFT (0x0000u)
N#define CSL_SPI_SPICC1_DCLK_DIV_RESETVAL (0x0000u)
N
N#define CSL_SPI_SPICC1_RESETVAL (0x0000u)
N
N/* SPICC2 */
N
N#define CSL_SPI_SPICC2_CLKEN_MASK (0x8000u)
N#define CSL_SPI_SPICC2_CLKEN_SHIFT (0x000Fu)
N#define CSL_SPI_SPICC2_CLKEN_RESETVAL (0x0000u)
N
N
N#define CSL_SPI_SPICC2_RESETVAL (0x0000u)
N
N/* SPIDC1 */
N
N
N#define CSL_SPI_SPIDC1_DD1_MASK (0x1800u)
N#define CSL_SPI_SPIDC1_DD1_SHIFT (0x000Bu)
N#define CSL_SPI_SPIDC1_DD1_RESETVAL (0x0000u)
N
N#define CSL_SPI_SPIDC1_CKPH1_MASK (0x0400u)
N#define CSL_SPI_SPIDC1_CKPH1_SHIFT (0x000Au)
N#define CSL_SPI_SPIDC1_CKPH1_RESETVAL (0x0000u)
N
N#define CSL_SPI_SPIDC1_CSP1_MASK (0x0200u)
N#define CSL_SPI_SPIDC1_CSP1_SHIFT (0x0009u)
N#define CSL_SPI_SPIDC1_CSP1_RESETVAL (0x0000u)
N
N#define CSL_SPI_SPIDC1_CKP1_MASK (0x0100u)
N#define CSL_SPI_SPIDC1_CKP1_SHIFT (0x0008u)
N#define CSL_SPI_SPIDC1_CKP1_RESETVAL (0x0000u)
N
N
N#define CSL_SPI_SPIDC1_DD0_MASK (0x0018u)
N#define CSL_SPI_SPIDC1_DD0_SHIFT (0x0003u)
N#define CSL_SPI_SPIDC1_DD0_RESETVAL (0x0000u)
N
N#define CSL_SPI_SPIDC1_CKPH0_MASK (0x0004u)
N#define CSL_SPI_SPIDC1_CKPH0_SHIFT (0x0002u)
N#define CSL_SPI_SPIDC1_CKPH0_RESETVAL (0x0000u)
N
N#define CSL_SPI_SPIDC1_CSP0_MASK (0x0002u)
N#define CSL_SPI_SPIDC1_CSP0_SHIFT (0x0001u)
N#define CSL_SPI_SPIDC1_CSP0_RESETVAL (0x0000u)
N
N#define CSL_SPI_SPIDC1_CKP0_MASK (0x0001u)
N#define CSL_SPI_SPIDC1_CKP0_SHIFT (0x0000u)
N#define CSL_SPI_SPIDC1_CKP0_RESETVAL (0x0000u)
N
N#define CSL_SPI_SPIDC1_RESETVAL (0x0000u)
N
N/* SPIDC2 */
N
N#define CSL_SPI_SPIDC2_LPBK_MASK (0x8000u)
N#define CSL_SPI_SPIDC2_LPBK_SHIFT (0x000Fu)
N#define CSL_SPI_SPIDC2_LPBK_RESETVAL (0x0000u)
N
N
N#define CSL_SPI_SPIDC2_DD3_MASK (0x1800u)
N#define CSL_SPI_SPIDC2_DD3_SHIFT (0x000Bu)
N#define CSL_SPI_SPIDC2_DD3_RESETVAL (0x0000u)
N
N#define CSL_SPI_SPIDC2_CKPH3_MASK (0x0400u)
N#define CSL_SPI_SPIDC2_CKPH3_SHIFT (0x000Au)
N#define CSL_SPI_SPIDC2_CKPH3_RESETVAL (0x0000u)
N
N#define CSL_SPI_SPIDC2_CSP3_MASK (0x0200u)
N#define CSL_SPI_SPIDC2_CSP3_SHIFT (0x0009u)
N#define CSL_SPI_SPIDC2_CSP3_RESETVAL (0x0000u)
N
N#define CSL_SPI_SPIDC2_CKP3_MASK (0x0100u)
N#define CSL_SPI_SPIDC2_CKP3_SHIFT (0x0008u)
N#define CSL_SPI_SPIDC2_CKP3_RESETVAL (0x0000u)
N
N
N#define CSL_SPI_SPIDC2_DD2_MASK (0x0018u)
N#define CSL_SPI_SPIDC2_DD2_SHIFT (0x0003u)
N#define CSL_SPI_SPIDC2_DD2_RESETVAL (0x0000u)
N
N#define CSL_SPI_SPIDC2_CKPH2_MASK (0x0004u)
N#define CSL_SPI_SPIDC2_CKPH2_SHIFT (0x0002u)
N#define CSL_SPI_SPIDC2_CKPH2_RESETVAL (0x0000u)
N
N#define CSL_SPI_SPIDC2_CSP2_MASK (0x0002u)
N#define CSL_SPI_SPIDC2_CSP2_SHIFT (0x0001u)
N#define CSL_SPI_SPIDC2_CSP2_RESETVAL (0x0000u)
N
N#define CSL_SPI_SPIDC2_CKP2_MASK (0x0001u)
N#define CSL_SPI_SPIDC2_CKP2_SHIFT (0x0000u)
N#define CSL_SPI_SPIDC2_CKP2_RESETVAL (0x0000u)
N
N#define CSL_SPI_SPIDC2_RESETVAL (0x0000u)
N
N/* SPICR1 */
N
N
N#define CSL_SPI_SPICR1_WIRQ_MASK (0x4000u)
N#define CSL_SPI_SPICR1_WIRQ_SHIFT (0x000Eu)
N#define CSL_SPI_SPICR1_WIRQ_RESETVAL (0x0000u)
N
N
N#define CSL_SPI_SPICR1_FLEN_MASK (0x0FFFu)
N#define CSL_SPI_SPICR1_FLEN_SHIFT (0x0000u)
N#define CSL_SPI_SPICR1_FLEN_RESETVAL (0x0000u)
N
N#define CSL_SPI_SPICR1_RESETVAL (0x0000u)
N
N/* SPICR2 */
N
N
N#define CSL_SPI_SPICR2_CSNUM_MASK (0x3000u)
N#define CSL_SPI_SPICR2_CSNUM_SHIFT (0x000Cu)
N#define CSL_SPI_SPICR2_CSNUM_RESETVAL (0x0000u)
N
N
N#define CSL_SPI_SPICR2_WLEN_MASK (0x00F8u)
N#define CSL_SPI_SPICR2_WLEN_SHIFT (0x0003u)
N#define CSL_SPI_SPICR2_WLEN_RESETVAL (0x0000u)
N
N
N#define CSL_SPI_SPICR2_CMD_MASK (0x0003u)
N#define CSL_SPI_SPICR2_CMD_SHIFT (0x0000u)
N#define CSL_SPI_SPICR2_CMD_RESETVAL (0x0000u)
N
N#define CSL_SPI_SPICR2_RESETVAL (0x0000u)
N
N/* SPISR1 */
N
N
N#define CSL_SPI_SPISR1_AE_MASK (0x0008u)
N#define CSL_SPI_SPISR1_AE_SHIFT (0x0003u)
N#define CSL_SPI_SPISR1_AE_RESETVAL (0x0000u)
N
N#define CSL_SPI_SPISR1_FC_MASK (0x0004u)
N#define CSL_SPI_SPISR1_FC_SHIFT (0x0002u)
N#define CSL_SPI_SPISR1_FC_RESETVAL (0x0000u)
N
N#define CSL_SPI_SPISR1_WC_MASK (0x0002u)
N#define CSL_SPI_SPISR1_WC_SHIFT (0x0001u)
N#define CSL_SPI_SPISR1_WC_RESETVAL (0x0000u)
N
N#define CSL_SPI_SPISR1_BSY_MASK (0x0001u)
N#define CSL_SPI_SPISR1_BSY_SHIFT (0x0000u)
N#define CSL_SPI_SPISR1_BSY_RESETVAL (0x0000u)
N
N#define CSL_SPI_SPISR1_RESETVAL (0x0000u)
N
N/* SPISR2 */
N
N
N#define CSL_SPI_SPISR2_WDCNT_MASK (0x1FFFu)
N#define CSL_SPI_SPISR2_WDCNT_SHIFT (0x0000u)
N#define CSL_SPI_SPISR2_WDCNT_RESETVAL (0x0000u)
N
N#define CSL_SPI_SPISR2_RESETVAL (0x0000u)
N
N/* SPIDR1 */
N
N#define CSL_SPI_SPIDR1_DATA_MASK (0xFFFFu)
N#define CSL_SPI_SPIDR1_DATA_SHIFT (0x0000u)
N#define CSL_SPI_SPIDR1_DATA_RESETVAL (0x0000u)
N
N#define CSL_SPI_SPIDR1_RESETVAL (0x0000u)
N
N/* SPIDR2 */
N
N#define CSL_SPI_SPIDR2_DATA_MASK (0xFFFFu)
N#define CSL_SPI_SPIDR2_DATA_SHIFT (0x0000u)
N#define CSL_SPI_SPIDR2_DATA_RESETVAL (0x0000u)
N
N#define CSL_SPI_SPIDR2_RESETVAL (0x0000u)
N
N#endif
N
L 17 "..\inc\llc_spi.h" 2
N
N#define SPI_STATUS_BUSY_MASK        (0x00000001)
N/**< spi status register busy bit mask                                      */
N#define SPI_STATUS_BUSY_SHIFT       (0x0000)
N/**< spi status register busy  bit shift                                    */
N#define SPI_STATUS_WC_MASK          (0x00000002)                            
N/**< spi status register word complete bit mask                             */
N#define SPI_STATUS_WC_SHIFT         (0x0001)                                
N/**< spi status register word complete bit shift                            */
N#define SPI_STATUS_FC_MASK          (0x00000004)                            
N/**< spi status register frame complete bit mask                            */
N#define SPI_STATUS_FC_SHIFT         (0x0002)                                
N/**< spi status register frame complete bit shift                           */
N#define SPI_STATUS_AE_MASK          (0x00000008)                            
N/**< spi status register address error bit mask                             */
N#define SPI_STATUS_AE_SHIFT         (0x0003)                                
N/**< spi status register address error bit shift                            */
N#define SPI_STATUS_WDCNT_MASK       (0x1FFF0000)                            
N/**< spi status register word count bit mask                                */
N#define SPI_STATUS_WDCNT_SHIFT      (0x0010)                                
N/**< spi status register word count bit shift                               */
N#define SPI_MAX_WDCNT               (4095)                                  
N/**< word count                                                             */
N#define SPI_MAX_FLEN                (4096)                                  
N/**< max frame lenght                                                       */
N#define SPI_FAILURE                 (0xFFFF)                                
N/**< Return values - Errors                                                 */
N
N#define SPI_STATUS_BUSY             (0x0001)
N#define SPI_WORD_COMPLETE           (0x0002)
N
N#define SPI_WORD_LEN_8              (8)
N#define SPI_WORD_LEN_16             (16)
N#define SPI_WORD_LEN_24             (24)
N#define SPI_WORD_LEN_32             (32)
N
N#define SPI_LOOP_BACK_ENABLE        (1)
N#define SPI_LOOP_BACK_DISABLE       (2)
N
N#define SPI_CLK_ENABLE              (1)
N#define SPI_CLK_DISABLE             (0)
N
N// Prescale value to change 100MHz to 7MHz for ADCCLK
N#define SPI_PRESCALE 				(13) /*7MHz(19) 5MHz//(216) 460KHz*/
N
N/**
N *  \brief spi slave data delay value
N *
N *  enum for spi slave data delay
N */
Ntypedef enum SPI_SlaveDataDly1
N{
N    SPI_SLV_DATA_DLY_0 = 0,
N    SPI_SLV_DATA_DLY_1,
N    SPI_SLV_DATA_DLY_2,
N    SPI_SLV_DATA_DLY_3,
N    SPI_SLV_DATA_DLY_UNKNOWN
N} SPI_SlaveDataDly;
N
N/**
N *  \brief spi slave clock polarity vlaue
N *
N *  enum for spi instance handle
N */
Ntypedef enum SPI_SlaveClkPolarity1
N{
N    SPI_LOW_AT_IDLE = 0,
N    SPI_HIGH_AT_IDLE,
N    SPI_UNKNOWN_AT_IDLE
N}SPI_SlaveClkPolarity;
N
N/**
N *  \brief spi slave clock phase value
N *
N *  enum for spi slave clock phase
N */
Ntypedef enum SPI_SlaveClkPh1
N{
N    SPI_SLV_PH_FALL_EDGE = 0,
N    SPI_SLV_PH_RISE_EDGE,
N    SPI_SLV_PH_UNKNOWN
N} SPI_SlaveClkPh;
N
N/**
N *  \brief spi slave select polarity value
N *
N *  enum for spi slave select polarity
N */
Ntypedef enum SPI_SlaveSelPol1
N{
N    SPI_SLV_ACTIVE_LOW = 0,
N    SPI_SLV_ACTIVE_HIGH,
N    SPI_CLK_POL_UNKNOWN
N} SPI_SlaveSelPol;
N
N/**
N *  \brief spi slave select value
N *
N *  enum for spi slave select
N */
Ntypedef enum SPI_SlaveSel1
N{
N    SPI_EEPROM = 0,
N    SPI_LCD,
N    SPI_SLAVE_RSVD,
N    SPI_SLAVE_RSVD1,
N    SPI_SLAVE_UNKNOWN
N} SPI_SlaveSel;
N
N/**
N *  \brief spi command value
N *
N *  enum for spi command
N */
Ntypedef enum SPI_Command1
N{
N    SPI_RSVD_CMD = 0,
N    SPI_READ,
N    SPI_WRITE,
N    SPI_RSVD1_CMD,
N    SPI_UNKNOWN_CMD
N} SPI_Command;
N
N/**
N *  \brief spi slave mode value
N *
N *  enum for spi slave mode
N */
Ntypedef enum SPI_HwMode1
N{
N    SPI_MODE_0 = 0,
N    SPI_MODE_1,
N    SPI_MODE_2,
N    SPI_MODE_3,
N    SPI_MODE_UNKNOWN
N} SPI_HwMode;
N
N/**
N *  \brief spi controller transaction status
N *
N *  enum for spi controller transation status
N */
Ntypedef enum SPI_TransactionStatus1
N{
N    SPI_BUSY = 0,
N    SPI_IDLE,
N    SPI_STATUS_UNKNOWN
N} SPI_TransactionStatus;
N
N/**
N *  \brief  LLC SPI initialization API
N *
N *  \param  clkDivisor      [IN]    clock divisor
N *  \param  frameLength     [IN]    Frame length
N *  \param  wordLength      [IN]    Word length
N *
N *  \return status of init
N */
NPSP_Result LLC_spiInit(
N                Uint32 clkDiv,
N                Uint16 frameLen,
N                Uint16 wordLen);
N
N/**
N *  \brief  enable spi clock
N *
N *  \return status of spi enable
N */
Nvoid LLC_SPI_Enable(void);
N
N/**
N *  \brief  disable spi clock
N *
N *  \return status of spi disable
N */
Nvoid LLC_SPI_Disable(void);
N
N/**
N *  \brief  set clock divisor
N *
N *  \return status of set clock
N */
Nvoid LLC_SPI_ClockSet(Uint32 spiClkRate);
N
N/**
N *  \brief  select a slave
N *
N *  \param SPI_SlaveSel spiSlaveSelect
N *
N *  \return status of slave select
N */
NPSP_Result LLC_SPI_SlaveSelect(SPI_SlaveSel spiSlaveSelect);
N
N/**
N *  \brief  select a slave
N *
N *  \return slave number
N */
NSPI_SlaveSel LLC_SPI_SlaveGet(void);
N
N/**
N *  \brief  select spi mode
N *
N *  \param SPI_HwMode spiModeSelect
N *
N *  \return status of mode select
N */
NPSP_Result LLC_SPI_ModeSelect(SPI_HwMode spiModeSelect);
N
N/**
N *  \brief  slave data delay select
N *
N *  \param SPI_SlaveDataDly spiDataDelay
N *
N *  \return status of slave data delay set
N */
NPSP_Result LLC_SPI_DataDelay(SPI_SlaveDataDly spiDataDelay);
N
N/**
N *  \brief  select a slave
N *
N *  \param Bool loopBackEnable
N *
N *  \return status loopbackmode select
N */
NPSP_Result LLC_SPI_LoopBackMode(Bool loopBackMode);
N
N/**
N *  \brief  get loopback mode
N *
N *  \return loopback mode
N */
NBool LLC_SPI_LoopBackGet(void);
N
N/**
N *  \brief  select slave polarity
N *
N *  \param SPI_SlaveSelPol slaveSelPol
N *
N *  \return status of slave polarity set
N */
NPSP_Result LLC_SPI_SlaveSelectPolSet(SPI_SlaveSelPol slaveSelPol);
N
N/**
N *  \brief  wordlength set
N *
N *  \param Uint16 wordLen
N *
N *  \return status of word length set
N */
NPSP_Result LLC_SPI_WLenSet(Uint16 wordLen);
N
N/**
N *  \brief  send SPI command
N *
N *  \param SPI_Command spiCmd
N *
N *  \return status of command set
N */
NPSP_Result LLC_SPI_CmdSet(SPI_Command spiCmd);
N
N/**
N *  \brief  framelength set
N *
N *  \param Uint16 frameLen
N *
N *  \return status of frame length set
N */
NPSP_Result LLC_SPI_FLenSet(Uint16 frameLen);
N
N/**
N *  \brief  read status register
N *
N *  \param Uint16 *spiStatus
N *
N */
Nvoid LLC_SPI_StatusRead(Uint16 *spiStatus);
N
N/**
N *  \brief  read byte
N *
N *  \return Uint16 byteRead
N */
NUint16 LLC_SPI_ByteRead(void);
N
N/**
N *  \brief  read word
N *
N *  \return Uint16 wordRead
N *
N */
NUint16 LLC_SPI_WordRead(void);
N
N/**
N *  \brief  read double word
N *
N *  \return Uint32 dWordRead
N *
N */
NUint32 LLC_SPI_DoubleWordRead(void);
N
N/**
N *  \brief  write byte
N *
N *  \param Uint16 byteWrite
N *
N */
Nvoid LLC_SPI_ByteWrite(Uint16 byteWrite);
N
N/**
N *  \brief  write word
N *
N *  \param Uint16 wordWrite
N *
N */
Nvoid LLC_SPI_WordWrite(Uint16 wordWrite);
N
N/**
N *  \brief  write double word
N *
N *  \param Uint32 dWordWrite
N *
N */
Nvoid LLC_SPI_DoubleWordWrite(Uint32 dWordWrite);
N
N/**
N *  \brief  enable word interrupt
N *
N */
Nvoid LLC_SPI_WordInterruptEnable(void);
N
N/**
N *  \brief  enable frame interrupt
N *
N */
Nvoid LLC_SPI_FrameInterruptEnable(void);
N
N/**
N *  \brief  get word interrupt state
N *
N */
Nvoid LLC_SPI_WordIntrRx_Get(void);
N
N/**
N *  \brief  get frame interrupt state
N *
N */
Nvoid LLC_SPI_FrameIntrRx_Get(void);
N
N/**
N  * \brief 
N  *         division of two integer number.
N  *         
N  * \param     Dividend     [IN]
N  * \param     Divisor     [IN]
N  *
N  * \return    Uint16
N  */
NUint16 spiDivFun(
N          Uint32 Dividend,
N          Uint32 Divisor  );
N
NPSP_Result LLC_SPI_WordLengthWrite(Uint32* InpBuf, Uint16 wordLength, Uint16 FrameLen);
NPSP_Result LLC_SPI_WordLengthWriteRead(Uint32* InpBuf, Uint32* CMDBuf, Uint16 wordLength, Uint16 FremLen, Uint16 CMDwordLength, Uint16 CMDFremLen);
NPSP_Result LLC_SPI_WordLengthRead(Uint32* InpBuf, Uint16 wordLength, Uint16 FremLen);
NPSP_Result LLC_SPI_CmdSet_WLenSet_SlaveSelect(SPI_Command spiCmd,Uint16 wordLen,SPI_SlaveSel spiSlaveSelect);
N#endif /* #ifndef  _LLC_SPI_H_ */
N
N/*EOF*/
L 15 "..\inc\ddc_spi.h" 2
N
N#ifndef NULL /* Move to a common place! */
S#define NULL 0
N#endif
N
N/**< definitions                                                     */
N#define SPI_OPEN_INSTANCES_MAX       (1)
N
N/**< ddc return values                                               */
N#define SPI_E_INV_OPER               (0xFF)
N/*#define SPI_WORD_LENGTH_IN_BITS    (0x08)*/
N#define SPI_CMD_LENGTH               (0x08)
N#define SPI_WRITE_1_WORD             (0x01)
N#define SPI_MAX_WORD_LEN             (0x32)
N
N/**
N *  \brief spi state value
N *
N *  enum for spi stae
N */
Ntypedef enum
N{
N    SPI_SLAVE_DELETED = 1,
N    SPI_SLAVE_CREATED,
N    SPI_SLAVE_OPENED,
N    SPI_SLAVE_CLOSED
N} SPI_InstanceState;
N
N/**
N *  \brief spi slave configuration
N *
N *  structure for spi slave config
N */
Ntypedef struct SPI_SlaveConfig_
N{
N    SPI_SlaveClkPolarity    slvClkPolr;
N    SPI_SlaveSelPol         csPolr;
N    SPI_SlaveClkPh          slvClkPhase;
N    SPI_SlaveDataDly        slvDataDly;
N    SPI_SlaveSel            slaveNo;
N} SPI_SlaveConfig;
N
N/**
N *  \brief spi controller state
N *
N *  struct for spi controller state and addr error
N */
Ntypedef struct SPI_CtrlrStatus_
N{
N    SPI_TransactionStatus   spiBusy;
N    Bool                    isAddrError;
N} SPI_CtrlrStatus;
N
N/**
N *  \brief spi transfer parameter
N *
N *  struct for spi transfer parameter
N */
Ntypedef struct SPI_TransParam_
N{
N    SPI_Command spiCmd;
N    Uint16      frameLen;
N    Uint16      wordCnt;
N    Uint16      wordLen;
N    Bool        isFrameComplete;
N    Bool        isWordComplete;
N} SPI_TransParam;
N
N/**
N *  \brief spi instance handle
N *
N *  struct for spi instance handle
N */
Ntypedef struct SPI_Handle_
N{
N    SPI_SlaveConfig     spiSlaveConfig;
N    SPI_CtrlrStatus     spiCtrlrStatus;
N    SPI_TransParam      spiTransParam;
N    Uint16              noOfOpen;
N    SPI_InstanceState   spiSlaveState;
N} SPI_Handle;
N
N/**
N *  \brief  DDC SPI initialization API
N *
N *  \param  Uint16 spiClkRate,
N *  \param  Uint16 frameLength,
N *  \param  Uint16 wordLength
N *
N *  \return status of init
N */
NPSP_Result DDC_SPI_Init(
N                    Uint32       spiClkRate,
N                    Uint16       frameLength,
N                    Uint16       wordLength);
N
N/**
N *  \brief  DDC SPI Deinitialization API
N *
N *  \param  void
N *
N *  \return status of deInit
N */
NPSP_Result DDC_SPI_DeInit(void);
N
N/**
N *  \brief  DDC SPI Instance open
N *
N *  \param  SPI_HwMode mode
N *  \param  SPI_SlaveConfig slvConfig
N *
N *  \return status of spi instance open
N */
NSPI_Handle* DDC_SPI_Open(
N                    SPI_HwMode      mode,
N                    SPI_SlaveConfig slvConfig);
N
N/**
N *  \brief  DDC SPI instance close
N *
N *  \param  SPI_Handle *spiHandle
N *
N *  \return status of spi instance close
N */
NUint16 DDC_SPI_Close(SPI_Handle *spiHandle);
N
N/**
N *  \brief  spi instance initialization
N *
N *  \return status of init
N */
Nvoid spiInitInstance(void);
N
N/**
N *  \brief  spi instance initialization
N *
N *  \return status of init
N */
NPSP_Result DDC_SpiDataTransaction(
N                         SPI_Handle *hSPI,
N                         Uint16     *transactionBuffer,
N                         Uint16     count,
N                         Uint16     readOrWrite);
N
N/**
N *  \brief  spi Read Data
N *
N *  \param  Uint16 *readBuf
N *  \param  Uint16 readCnt
N *  \param  Uint16 wLen
N *
N *  \return status of spi read
N */
NPSP_Result spiReadData(
N                    Uint16      *readBuf,
N                    Uint16      readCnt,
N                    Uint16      wLen);
N
N/**
N *  \brief  spi Write Data
N *
N *  \param  Uint16 *writeBuf
N *  \param  Uint16 writeCnt
N *  \param  Uint16 wLen
N *
N *  \return status of spi write
N */
NPSP_Result spiWriteData(
N                    Uint16      *writeBuf,
N                    Uint16      writeCnt,
N                    Uint16      wLen);
N
N/**
N *  \brief  spi Write Data
N *
N *  \param  Uint16 *cmdBuf
N *  \param  Uint16 cmdCnt
N *  \param  Uint16 cLen
N *  \param  Uint16 rdOrWr
N *
N *  \return status of spi cmd write
N */
NPSP_Result spiWriteCmd(
N                    Uint16      *cmdBuf,
N                    Uint16      cmdCnt,
N                    Uint16      cLen,
N                    Uint16      rdOrWr);
N
N/**
N *  \brief  DDC SPI slave configurations
N *
N *  \param  SPI_SlaveSel        slNo
N *  \param  SPI_SlaveDataDly    DD
N *  \param  SPI_SlaveSelPol     csPol
N *
N *  \return status of slave configurations
N */
NPSP_Result DDC_DeviceConfig(SPI_SlaveSel    slNo,
N                        SPI_SlaveDataDly    DD,
N                        SPI_SlaveSelPol     csPol);
N
N#endif /* #ifndef _DDC_SPI_H_ */
N/*EOF*/
L 15 "..\inc\dda_spi.h" 2
N
N/**
N *  \brief spi controller mode value
N *
N *  enum for spi controller mode
N */
Ntypedef enum
N{
N    SPI_MODE = 0,
N    SPI_MIBSPI_MODE,
N    SPI_UNKNOWN_MODE
N} DDA_SPI_CTRLMODE;
N
N/**
N *  \brief spi controller master/slave mode value
N *
N *  enum for spi controller master/slave mode
N */
Ntypedef enum
N{
N    SPI_MASTER = 0,
N    SPI_SLAVE,
N    SPI_UNKNOWN
N} DDA_SPI_MASTER_SLAVE;
N
N/**
N *  \brief spi driver mode value
N *
N *  enum for spi driver mode
N */
Ntypedef enum
N{
N    SPI_POLLED = 0,
N    SPI_INTERRUPT,
N    SPI_DMA_INTERRUPT,
N    SPI_UNKNOWN_OPMODE
N} DDA_SPI_OPMODE;
N
N/**
N *  \brief spi instance value
N *
N *  enum for spi instance
N */
Ntypedef enum
N{
N    SPI_SINGLE_INST = 0,
N    SPI_MULTI_INST,
N    SPI_UNKNOWN_INST
N} DDA_SPI_DRV_INST;
N
N/**
N *  \brief spi mode of transfer value
N *
N *  enum for spi mode of transfer
N */
Ntypedef enum
N{
N    SPI_SYNC = 0,
N    SPI_ASYNC,
N    SPI_UNKNOWN_TX_MODE
N} DDA_SPI_SYNC_ASYNC;
N
N/**
N *  \brief spi driver configuration
N *
N *  structure for spi driver configuration
N */
Ntypedef struct
N{
N    DDA_SPI_CTRLMODE        spiCtrlMode; /**< spi controllor mode */
N    DDA_SPI_MASTER_SLAVE    spiMaster; /**< spi master/slave */
N    DDA_SPI_OPMODE          opMode;/**< spi operation mode */
N    DDA_SPI_DRV_INST        instances;/**< spi driver instances */
N    DDA_SPI_SYNC_ASYNC      syncMode;/**< spi sync/async mode */
N} DDA_spiConfig;
N
N/**
N *  \brief  enable the SPI module
N *
N *  \return None.
N */
Nvoid spiPreInit(void);
N
N/**
N *  \brief Initialize SPI controller
N *
N *  \param  spiConfig   [IN]   spi config structure
N *  \param  spiClkRate  [IN]
N *  \param  frameLen    [IN]
N *  \param  wordLen [IN]
N *  \param  slaveNo [IN]
N *  \param  dataDly [IN]
N *  \param  csPol   [IN]
N *  \param  clkPh   [IN]
N *  \param  clkPol  [IN]
N *
N *  \return status of init
N */
NPSP_Result SPI_INIT(
N              DDA_spiConfig spiConfig,
N              Uint32 spiClkRate,
N              Uint16 frameLen,
N              Uint16 wordLen,
N              Uint16 slaveNo,
N              Uint16 dataDly,
N              Uint16 csPol,
N              Uint16 clkPh,
N              Uint16 clkPol);
N/**
N *  \brief  Read/Write N Bytes from/to SPI slave
N *
N *  \param  buffer  [IN]
N *  \param  count   [IN]
N *  \param  cmdBuf  [IN]
N *  \param  cmdCount    [IN]
N *  \param  readOrWrite [IN]
N *
N *  \return status of read/write
N */
NPSP_Result SPI_READ_WRITE_N_BYTES(
N                              Uint16 *buffer,
N                              Uint16 count,
N                              Uint16 *cmdBuf,
N                              Uint16 cmdCount,
N                              Uint16 readOrWrite);
N
N/**
N *                   SPI_WRITE_BYTES
N *  \brief  write 1 byte
N *
N *  \param  data
N *
N *  \return status of write
N */
NPSP_Result SPI_WRITE_BYTES(Uint16 data);
N
N/**
N *  \brief  spi driver configurations
N *
N *  \param spiCtrlMode  [IN]
N *  \param spiMaster    [IN]
N *  \param opMode   [IN]
N *  \param instance [IN]
N *  \param syncMode [IN]
N *
N *  \return spi driver configurations result
N */
NPSP_Result DDA_spiConfiguration(
N                           DDA_SPI_CTRLMODE         spiCtrlMode,
N                           DDA_SPI_MASTER_SLAVE     spiMaster,
N                           DDA_SPI_OPMODE           opMode,
N                           DDA_SPI_DRV_INST         instance,
N                           DDA_SPI_SYNC_ASYNC       syncMode);
N
N/**
N *  \brief SPI slave configurations
N *
N *  \param slaveNo  [IN]
N *  \param slvClkPhase  [IN]
N *  \param slvClkPolr   [IN]
N *  \param slvDataDly   [IN]
N *  \param csPolr   [IN]
N *
N *  \return spi slave configurations result
N */
NPSP_Result DDA_spiSlaveSelect(
N                        SPI_SlaveSel            slaveNo,
N                        SPI_SlaveClkPh          slvClkPhase,
N                        SPI_SlaveClkPolarity    slvClkPolr,
N                        SPI_SlaveDataDly        slvDataDly,
N                        SPI_SlaveSelPol         csPolr);
N/**
N *  \brief SPI slave deselect
N *
N *  \return spi slave deselect status
N */
NPSP_Result SPI_DeInit(void);
N
N/**
N *  \brief Write enable for EEPROM
N *
N *  \return void
N */
Nvoid DDA_SPI_DeviceConfigToWrite(void);
N
N#endif /* _DDA_SPI_H_ */
N/*EOF*/
L 21 "../inc/ECGDemoNonBios.h" 2
N#include "llc_spi.h"
N//#include "cpu_power.h"
N#include "csl_ioport.h"
L 1 "../common_inc/csl_ioport.h" 1
N/*****************************************************************************
N * File Name : csl_ioport.h 
N *
N * Brief	 : CSL file for CPU registers
N *
N * Copyright (C) 2009 -2010 Texas Instruments Incorporated - http://www.ti.com/ 
N *
N *
N *  Redistribution and use in source and binary forms, with or without 
N *  modification, are permitted provided that the following conditions 
N *  are met:
N *
N *    Redistributions of source code must retain the above copyright 
N *    notice, this list of conditions and the following disclaimer.
N *
N *    Redistributions in binary form must reproduce the above copyright
N *    notice, this list of conditions and the following disclaimer in the 
N *    documentation and/or other materials provided with the   
N *    distribution.
N *
N *    Neither the name of Texas Instruments Incorporated nor the names of
N *    its contributors may be used to endorse or promote products derived
N *    from this software without specific prior written permission.
N *
N *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
N *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
N *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
N *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
N *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
N *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
N *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
N *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
N *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
N *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
N *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
N *
N******************************************************************************/
N
N#ifndef _CSLR_IOPORT_H_
N#define _CSLR_IOPORT_H_
N
N/* Macro definitions for register addresses */
N
N#define PERIPHSEL0_ADDR     ((ioport volatile unsigned*)0x1C00)
N/**< Extenral bus selection register address                                */
N#define IDLE_PCGCRL_ADDR    ((ioport volatile unsigned*)0x1C02)             
N/**< IDLE PCGCR LSW address                                                 */
N#define IDLE_PCGCRM_ADDR    ((ioport volatile unsigned*)0x1C03)             
N/**< IDLE PCGCR MSW address                                                 */
N#define PER_RSTCOUNT_ADDR   ((ioport volatile unsigned*)0x1C04)             
N/**< Peripheral reset counter register address                              */
N#define PER_RESET_ADDR      ((ioport volatile unsigned*)0x1C05)             
N/**< Peripheral reset control register address                              */
N#define GPIO_DIR0_ADDR      ((ioport volatile unsigned*)0x1C06)             
N/**< GPIO Direction Register0 address                                       */
N#define GPIO_DIR1_ADDR      ((ioport volatile unsigned*)0x1C07)             
N/**< GPIO Direction Register1 address                                       */
N#define GPIO_DIN0_ADDR     ((ioport volatile unsigned*)0x1C08)             
N/**< GPIO Data In Register0 address                                        */
N#define GPIO_DIN1_ADDR     ((ioport volatile unsigned*)0x1C09)             
N/**< GPIO Data In Register1 address                                        */
N
N#define GPIO_DOUT0_ADDR     ((ioport volatile unsigned*)0x1C0A)             
N/**< GPIO Data Out Register0 address                                        */
N#define GPIO_DOUT1_ADDR     ((ioport volatile unsigned*)0x1C0B)             
N/**< GPIO Data Out Register1 address                                        */
N
N#define GPIO_IER0_ADDR     ((ioport volatile unsigned*)0x1C0E)             
N/**< GPIO IER Register0 address                                        */
N#define GPIO_IER1_ADDR     ((ioport volatile unsigned*)0x1C0F)             
N/**< GPIO IER Register1 address                                        */
N#define GPIO_IFR0_ADDR		((ioport volatile unsigned*)0x1C10)
N/*	GPIO IFR Register0 address																			*/
N#define GPIO_IFR1_ADDR		((ioport volatile unsigned*)0x1C11)
N/*	GPIO IFR Register1 address																			*/
N
N#define CPU_IER0_ADDR       ((volatile unsigned*)0x0000)
N/**< CPU interrupt enable register0 address                                 */
N#define CPU_IER1_ADDR       ((volatile unsigned*)0x0045)                        
N/**< CPU interrupt enable register1 address                                 */
N#define CPU_IFR0_ADDR       ((volatile unsigned*)0x0001)                        
N/**< CPU interrupt flag register0 address                                   */
N#define CPU_IFR1_ADDR       ((volatile unsigned*)0x0046)                        
N/**< CPU interrupt flag register1 address                                   */
N
N
N/* Extenral bus selection register bit fields */
N
N#define CSL_PERIPHSEL0_SEL_PARALLELPORT_MASK (0x7000u)
N#define CSL_PERIPHSEL0_SEL_PARALLELPORT_SHIFT (0x000Cu)
N#define CSL_PERIPHSEL0_SEL_PARALLELPORT_RESETVAL (0x0000u)
N
N#define CSL_PERIPHSEL0_SEL_SERIALPORT1_MASK (0x0C00u)
N#define CSL_PERIPHSEL0_SEL_SERIALPORT1_SHIFT (0x000Au)
N#define CSL_PERIPHSEL0_SEL_SERIALPORT1_RESETVAL (0x0000u)
N
N#define CSL_PERIPHSEL0_SEL_SERIALPORT0_MASK (0x0300u)
N#define CSL_PERIPHSEL0_SEL_SERIALPORT0_SHIFT (0x0008u)
N#define CSL_PERIPHSEL0_SEL_SERIALPORT0_RESETVAL (0x0000u)
N
N#define CSL_PERIPHSEL0_SEL_A20GPIO26_MASK (0x0020u)
N#define CSL_PERIPHSEL0_SEL_A20GPIO26_SHIFT (0x0005u)
N#define CSL_PERIPHSEL0_SEL_A20GPIO26_RESETVAL (0x0000u)
N
N#define CSL_PERIPHSEL0_SEL_A19GPIO25_MASK (0x0010u)
N#define CSL_PERIPHSEL0_SEL_A19GPIO25_SHIFT (0x0004u)
N#define CSL_PERIPHSEL0_SEL_A19GPIO25_RESETVAL (0x0000u)
N
N#define CSL_PERIPHSEL0_SEL_A18GPIO24_MASK (0x0008u)
N#define CSL_PERIPHSEL0_SEL_A18GPIO24_SHIFT (0x0003u)
N#define CSL_PERIPHSEL0_SEL_A18GPIO24_RESETVAL (0x0000u)
N
N#define CSL_PERIPHSEL0_SEL_A17GPIO23_MASK (0x0004u)
N#define CSL_PERIPHSEL0_SEL_A17GPIO23_SHIFT (0x0002u)
N#define CSL_PERIPHSEL0_SEL_A17GPIO23_RESETVAL (0x0000u)
N
N#define CSL_PERIPHSEL0_SEL_A16GPIO22_MASK (0x0002u)
N#define CSL_PERIPHSEL0_SEL_A16GPIO22_SHIFT (0x0001u)
N#define CSL_PERIPHSEL0_SEL_A16GPIO22_RESETVAL (0x0000u)
N
N#define CSL_PERIPHSEL0_SEL_A15GPIO21_MASK (0x0001u)
N#define CSL_PERIPHSEL0_SEL_A15GPIO21_SHIFT (0x0000u)
N#define CSL_PERIPHSEL0_SEL_A15GPIO21_RESETVAL (0x0000u)
N
N#define CSL_PERIPHSEL0_RESETVAL (0x0000u)
N
N
N/* IDLE PCGCR LSW bit fields */
N
N#define CSL_IDLE_PCGCRL_MASTER_CLK_DIS_MASK (0x8000u)
N#define CSL_IDLE_PCGCRL_MASTER_CLK_DIS_SHIFT (0x000Fu)
N#define CSL_IDLE_PCGCRL_MASTER_CLK_DIS_RESETVAL (0x0000u)
N
N#define CSL_IDLE_PCGCRL_IIS2_IDLE_MASK (0x4000u)
N#define CSL_IDLE_PCGCRL_IIS2_IDLE_SHIFT (0x000Eu)
N#define CSL_IDLE_PCGCRL_IIS2_IDLE_RESETVAL (0x0000u)
N
N#define CSL_IDLE_PCGCRL_TIMER2_IDLE_MASK (0x2000u)
N#define CSL_IDLE_PCGCRL_TIMER2_IDLE_SHIFT (0x000Du)
N#define CSL_IDLE_PCGCRL_TIMER2_IDLE_RESETVAL (0x0000u)
N
N#define CSL_IDLE_PCGCRL_TIMER1_IDLE_MASK (0x1000u)
N#define CSL_IDLE_PCGCRL_TIMER1_IDLE_SHIFT (0x000Cu)
N#define CSL_IDLE_PCGCRL_TIMER1_IDLE_RESETVAL (0x0000u)
N
N#define CSL_IDLE_PCGCRL_EMIF_IDLE_MASK (0x0800u)
N#define CSL_IDLE_PCGCRL_EMIF_IDLE_SHIFT (0x000Bu)
N#define CSL_IDLE_PCGCRL_EMIF_IDLE_RESETVAL (0x0000u)
N
N#define CSL_IDLE_PCGCRL_TIMER0_IDLE_MASK (0x0400u)
N#define CSL_IDLE_PCGCRL_TIMER0_IDLE_SHIFT (0x000Au)
N#define CSL_IDLE_PCGCRL_TIMER0_IDLE_RESETVAL (0x0000u)
N
N#define CSL_IDLE_PCGCRL_IIS1_IDLE_MASK (0x0200u)
N#define CSL_IDLE_PCGCRL_IIS1_IDLE_SHIFT (0x0009u)
N#define CSL_IDLE_PCGCRL_IIS1_IDLE_RESETVAL (0x0000u)
N
N#define CSL_IDLE_PCGCRL_IIS0_IDLE_MASK (0x0100u)
N#define CSL_IDLE_PCGCRL_IIS0_IDLE_SHIFT (0x0008u)
N#define CSL_IDLE_PCGCRL_IIS0_IDLE_RESETVAL (0x0000u)
N
N#define CSL_IDLE_PCGCRL_MMCSD1_IDLE_MASK (0x0080u)
N#define CSL_IDLE_PCGCRL_MMCSD1_IDLE_SHIFT (0x0007u)
N#define CSL_IDLE_PCGCRL_MMCSD1_IDLE_RESETVAL (0x0000u)
N
N#define CSL_IDLE_PCGCRL_I2C_IDLE_MASK (0x0040u)
N#define CSL_IDLE_PCGCRL_I2C_IDLE_SHIFT (0x0006u)
N#define CSL_IDLE_PCGCRL_I2C_IDLE_RESETVAL (0x0000u)
N
N#define CSL_IDLE_PCGCRL_COPROC_IDLE_MASK (0x0020u)
N#define CSL_IDLE_PCGCRL_COPROC_IDLE_SHIFT (0x0005u)
N#define CSL_IDLE_PCGCRL_COPROC_IDLE_RESETVAL (0x0000u)
N
N#define CSL_IDLE_PCGCRL_MMCSD0_IDLE_MASK (0x0010u)
N#define CSL_IDLE_PCGCRL_MMCSD0_IDLE_SHIFT (0x0004u)
N#define CSL_IDLE_PCGCRL_MMCSD0_IDLE_RESETVAL (0x0000u)
N
N#define CSL_IDLE_PCGCRL_DMA0_IDLE_MASK (0x0008u)
N#define CSL_IDLE_PCGCRL_DMA0_IDLE_SHIFT (0x0003u)
N#define CSL_IDLE_PCGCRL_DMA0_IDLE_RESETVAL (0x0000u)
N
N#define CSL_IDLE_PCGCRL_UART_IDLE_MASK (0x0004u)
N#define CSL_IDLE_PCGCRL_UART_IDLE_SHIFT (0x0002u)
N#define CSL_IDLE_PCGCRL_UART_IDLE_RESETVAL (0x0000u)
N
N#define CSL_IDLE_PCGCRL_SPI_IDLE_MASK (0x0002u)
N#define CSL_IDLE_PCGCRL_SPI_IDLE_SHIFT (0x0001u)
N#define CSL_IDLE_PCGCRL_SPI_IDLE_RESETVAL (0x0000u)
N
N#define CSL_IDLE_PCGCRL_IIS3_IDLE_MASK (0x0001u)
N#define CSL_IDLE_PCGCRL_IIS3_IDLE_SHIFT (0x0000u)
N#define CSL_IDLE_PCGCRL_IIS3_IDLE_RESETVAL (0x0000u)
N
N#define CSL_IDLE_PCGCRL_RESETVAL (0x0000u)
N
N
N/* IDLE PCGCR MSW bit fields */
N
N#define CSL_IDLE_PCGCRM_ANAREG_CG_MASK (0x0040u)
N#define CSL_IDLE_PCGCRM_ANAREG_CG_SHIFT (0x0006u)
N#define CSL_IDLE_PCGCRM_ANAREG_CG_RESETVAL (0x0000u)
N
N#define CSL_IDLE_PCGCRM_DMA3_CG_MASK (0x0020u)
N#define CSL_IDLE_PCGCRM_DMA3_CG_SHIFT (0x0005u)
N#define CSL_IDLE_PCGCRM_DMA3_CG_RESETVAL (0x0000u)
N
N#define CSL_IDLE_PCGCRM_DMA2_CG_MASK (0x0010u)
N#define CSL_IDLE_PCGCRM_DMA2_CG_SHIFT (0x0004u)
N#define CSL_IDLE_PCGCRM_DMA2_CG_RESETVAL (0x0000u)
N
N#define CSL_IDLE_PCGCRM_DMA1_CG_MASK (0x0008u)
N#define CSL_IDLE_PCGCRM_DMA1_CG_SHIFT (0x0003u)
N#define CSL_IDLE_PCGCRM_DMA1_CG_RESETVAL (0x0000u)
N
N#define CSL_IDLE_PCGCRM_USB_CG_MASK (0x0004u)
N#define CSL_IDLE_PCGCRM_USB_CG_SHIFT (0x0002u)
N#define CSL_IDLE_PCGCRM_USB_CG_RESETVAL (0x0000u)
N
N#define CSL_IDLE_PCGCRM_SAR_CG_MASK (0x0002u)
N#define CSL_IDLE_PCGCRM_SAR_CG_SHIFT (0x0001u)
N#define CSL_IDLE_PCGCRM_SAR_CG_RESETVAL (0x0000u)
N
N#define CSL_IDLE_PCGCRM_LCD_CG_MASK (0x0001u)
N#define CSL_IDLE_PCGCRM_LCD_CG_SHIFT (0x0000u)
N#define CSL_IDLE_PCGCRM_LCD_CG_RESETVAL (0x0000u)
N
N#define CSL_IDLE_PCGCRM_RESETVAL (0x0000u)
N
N
N/* Peripheral reset counter register */
N
N#define CSL_PER_RSTCOUNT_MASK (0xFFFFu)
N#define CSL_PER_RSTCOUNT_SHIFT (0x0000u)
N#define CSL_PER_RSTCOUNT_RESETVAL (0x0000u)
N
N
N/* Peripheral Reset Control Register bit fields */
N
N#define CSL_PER_RESET_HOLDINRESET_LCD_IIS2_IIS3_UART_SPI_MASK (0x8000u)
N#define CSL_PER_RESET_HOLDINRESET_LCD_IIS2_IIS3_UART_SPI_SHIFT (0x000Fu)
N#define CSL_PER_RESET_HOLDINRESET_LCD_IIS2_IIS3_UART_SPI_RESETVAL (0x0000u)
N
N#define CSL_PER_RESET_HOLDINRESET_FFTCOP_MASK (0x4000u)
N#define CSL_PER_RESET_HOLDINRESET_FFTCOP_SHIFT (0x000Eu)
N#define CSL_PER_RESET_HOLDINRESET_FFTCOP_RESETVAL (0x0000u)
N
N#define CSL_PER_RESET_HOLDINRESET_MMCSD0_MMCSD1_IIS0_IIS1_MASK (0x2000u)
N#define CSL_PER_RESET_HOLDINRESET_MMCSD0_MMCSD1_IIS0_IIS1_SHIFT (0x000Du)
N#define CSL_PER_RESET_HOLDINRESET_MMCSD0_MMCSD1_IIS0_IIS1_RESETVAL (0x0000u)
N
N#define CSL_PER_RESET_HOLDINRESET_DMA_MASK (0x1000u)
N#define CSL_PER_RESET_HOLDINRESET_DMA_SHIFT (0x000Cu)
N#define CSL_PER_RESET_HOLDINRESET_DMA_RESETVAL (0x0000u)
N
N#define CSL_PER_RESET_HOLDINRESET_USB_MASK (0x0800u)
N#define CSL_PER_RESET_HOLDINRESET_USB_SHIFT (0x000Bu)
N#define CSL_PER_RESET_HOLDINRESET_USB_RESETVAL (0x0000u)
N
N#define CSL_PER_RESET_HOLDINRESET_ANALOG_SAR_MASK (0x0400u)
N#define CSL_PER_RESET_HOLDINRESET_ANALOG_SAR_SHIFT (0x000Au)
N#define CSL_PER_RESET_HOLDINRESET_ANALOG_SAR_RESETVAL (0x0000u)
N
N#define CSL_PER_RESET_HOLDINRESET_EMIF_TIMERS_RTC_MASK (0x0200u)
N#define CSL_PER_RESET_HOLDINRESET_EMIF_TIMERS_RTC_SHIFT (0x0009u)
N#define CSL_PER_RESET_HOLDINRESET_EMIF_TIMERS_RTC_RESETVAL (0x0000u)
N
N#define CSL_PER_RESET_HOLDINRESET_I2C_MASK (0x0100u)
N#define CSL_PER_RESET_HOLDINRESET_I2C_SHIFT (0x0008u)
N#define CSL_PER_RESET_HOLDINRESET_I2C_RESETVAL (0x0000u)
N
N#define CSL_PER_RESET_RESETEN_LCD_IIS2_IIS3_UART_SPI_MASK (0x0080u)
N#define CSL_PER_RESET_RESETEN_LCD_IIS2_IIS3_UART_SPI_SHIFT (0x0007u)
N#define CSL_PER_RESET_RESETEN_LCD_IIS2_IIS3_UART_SPI_RESETVAL (0x0000u)
N
N#define CSL_PER_RESET_RESETEN_FFTCOP_MASK (0x0040u)
N#define CSL_PER_RESET_RESETEN_FFTCOP_SHIFT (0x0006u)
N#define CSL_PER_RESET_RESETEN_FFTCOP_RESETVAL (0x0000u)
N
N#define CSL_PER_RESET_RESETEN_MMCSD0_MMCSD1_IIS0_IIS1_MASK (0x0020u)
N#define CSL_PER_RESET_RESETEN_MMCSD0_MMCSD1_IIS0_IIS1_SHIFT (0x0005u)
N#define CSL_PER_RESET_RESETEN_MMCSD0_MMCSD1_IIS0_IIS1_RESETVAL (0x0000u)
N
N#define CSL_PER_RESET_RESETEN_DMA_MASK (0x0010u)
N#define CSL_PER_RESET_RESETEN_DMA_SHIFT (0x0004u)
N#define CSL_PER_RESET_RESETEN_DMA_RESETVAL (0x0000u)
N
N#define CSL_PER_RESET_RESETEN_USB_MASK (0x0008u)
N#define CSL_PER_RESET_RESETEN_USB_SHIFT (0x0003u)
N#define CSL_PER_RESET_RESETEN_USB_RESETVAL (0x0000u)
N
N#define CSL_PER_RESET_RESETEN_ANALOG_SAR_MASK (0x0004u)
N#define CSL_PER_RESET_RESETEN_ANALOG_SAR_SHIFT (0x0002u)
N#define CSL_PER_RESET_RESETEN_ANALOG_SAR_RESETVAL (0x0000u)
N
N#define CSL_PER_RESET_RESETEN_EMIF_TIMERS_RTC_MASK (0x0002u)
N#define CSL_PER_RESET_RESETEN_EMIF_TIMERS_RTC_SHIFT (0x0001u)
N#define CSL_PER_RESET_RESETEN_EMIF_TIMERS_RTC_RESETVAL (0x0000u)
N
N#define CSL_PER_RESET_RESETEN_I2C_MASK (0x0001u)
N#define CSL_PER_RESET_RESETEN_I2C_SHIFT (0x0000u)
N#define CSL_PER_RESET_RESETEN_I2C_RESETVAL (0x0000u)
N
N#define CSL_PER_RESET_RESETVAL (0x0000u)
N
N
N/* GPIO Direction Register0 */
N
N#define CSL_GPIO_DIR0_MASK (0xFFFFu)
N#define CSL_GPIO_DIR0_SHIFT (0x0000u)
N#define CSL_GPIO_DIR0_RESETVAL (0x0000u)
N
N
N/* GPIO Direction Register1 */
N
N#define CSL_GPIO_DIR1_MASK (0xFFFFu)
N#define CSL_GPIO_DIR1_SHIFT (0x0000u)
N#define CSL_GPIO_DIR1_RESETVAL (0x0000u)
N
N
N/* GPIO Data Out Register0 */
N
N#define CSL_GPIO_DOUT0_MASK (0xFFFFu)
N#define CSL_GPIO_DOUT0_SHIFT (0x0000u)
N#define CSL_GPIO_DOUT0_RESETVAL (0x0000u)
N
N
N/* GPIO Data Out Register1 */
N
N#define CSL_GPIO_DOUT1_MASK (0xFFFFu)
N#define CSL_GPIO_DOUT1_SHIFT (0x0000u)
N#define CSL_GPIO_DOUT1_RESETVAL (0x0000u)
N
N
N/* CPU interrupt enable register0 bit fields */
N
N#define CSL_CPUIER0_RCV2INT_ENABLE_MASK (0x8000u)
N#define CSL_CPUIER0_RCV2INT_ENABLE_SHIFT (0x000Fu)
N#define CSL_CPUIER0_RCV2INT_ENABLE_RESETVAL (0x0000u)
N
N#define CSL_CPUIER0_XMT2INT_ENABLE_MASK (0x4000u)
N#define CSL_CPUIER0_XMT2INT_ENABLE_SHIFT (0x000Eu)
N#define CSL_CPUIER0_XMT2INT_ENABLE_RESETVAL (0x0000u)
N
N#define CSL_CPUIER0_SARINT_ENABLE_MASK (0x2000u)
N#define CSL_CPUIER0_SARINT_ENABLE_SHIFT (0x000Du)
N#define CSL_CPUIER0_SARINT_ENABLE_RESETVAL (0x0000u)
N
N#define CSL_CPUIER0_LCDINT_ENABLE_MASK (0x1000u)
N#define CSL_CPUIER0_LCDINT_ENABLE_SHIFT (0x000Cu)
N#define CSL_CPUIER0_LCDINT_ENABLE_RESETVAL (0x0000u)
N
N#define CSL_CPUIER0_RVC1INT_ENABLE_MASK (0x0800u)
N#define CSL_CPUIER0_RVC1INT_ENABLE_SHIFT (0x000Bu)
N#define CSL_CPUIER0_RVC1INT_ENABLE_RESETVAL (0x0000u)
N
N#define CSL_CPUIER0_COPROCINT_ENABLE_MASK (0x0400u)
N#define CSL_CPUIER0_COPROCINT_ENABLE_SHIFT (0x000Au)
N#define CSL_CPUIER0_COPROCINT_ENABLE_RESETVAL (0x0000u)
N
N#define CSL_CPUIER0_XMT1INT_ENABLE_MASK (0x0200u)
N#define CSL_CPUIER0_XMT1INT_ENABLE_SHIFT (0x0009u)
N#define CSL_CPUIER0_XMT1INT_ENABLE_RESETVAL (0x0000u)
N
N#define CSL_CPUIER0_DMAINT_ENABLE_MASK (0x0100u)
N#define CSL_CPUIER0_DMAINT_ENABLE_SHIFT (0x0008u)
N#define CSL_CPUIER0_DMAINT_ENABLE_RESETVAL (0x0000u)
N
N#define CSL_CPUIER0_RCV0INT_ENABLE_MASK (0x0080u)
N#define CSL_CPUIER0_RCV0INT_ENABLE_SHIFT (0x0007u)
N#define CSL_CPUIER0_RCV0INT_ENABLE_RESETVAL (0x0000u)
N
N#define CSL_CPUIER0_UARTINT_ENABLE_MASK (0x0040u)
N#define CSL_CPUIER0_UARTINT_ENABLE_SHIFT (0x0006u)
N#define CSL_CPUIER0_UARTINT_ENABLE_RESETVAL (0x0000u)
N
N#define CSL_CPUIER0_XMT0INT_ENABLE_MASK (0x0020u)
N#define CSL_CPUIER0_XMT0INT_ENABLE_SHIFT (0x0005u)
N#define CSL_CPUIER0_XMT0INT_ENABLE_RESETVAL (0x0000u)
N
N#define CSL_CPUIER0_TIMERINT_ENABLE_MASK (0x0010u)
N#define CSL_CPUIER0_TIMERINT_ENABLE_SHIFT (0x0004u)
N#define CSL_CPUIER0_TIMERINT_ENABLE_RESETVAL (0x0000u)
N
N#define CSL_CPUIER0_INT1_ENABLE_MASK (0x0008u)
N#define CSL_CPUIER0_INT1_ENABLE_SHIFT (0x0003u)
N#define CSL_CPUIER0_INT1_ENABLE_RESETVAL (0x0000u)
N
N#define CSL_CPUIER0_INT0_ENABLE_MASK (0x0004u)
N#define CSL_CPUIER0_INT0_ENABLE_SHIFT (0x0002u)
N#define CSL_CPUIER0_INT0_ENABLE_RESETVAL (0x0000u)
N
N#define CSL_CPUIER0_RESETVAL (0x0000u)
N
N
N/* CPU interrupt enable register1 bit fields */
N
N#define CSL_CPUIER1_RTOSINT_ENABLE_MASK (0x0400u)
N#define CSL_CPUIER1_RTOSINT_ENABLE_SHIFT (0x000Au)
N#define CSL_CPUIER1_RTOSINT_ENABLE_RESETVAL (0x0000u)
N
N#define CSL_CPUIER1_DLOGINT_ENABLE_MASK (0x0200u)
N#define CSL_CPUIER1_DLOGINT_ENABLE_SHIFT (0x0009u)
N#define CSL_CPUIER1_DLOGINT_ENABLE_RESETVAL (0x0000u)
N
N#define CSL_CPUIER1_BERRINT_ENABLE_MASK (0x0100u)
N#define CSL_CPUIER1_BERRINT_ENABLE_SHIFT (0x0008u)
N#define CSL_CPUIER1_BERRINT_ENABLE_RESETVAL (0x0000u)
N
N#define CSL_CPUIER1_I2CINT_ENABLE_MASK (0x0080u)
N#define CSL_CPUIER1_I2CINT_ENABLE_SHIFT (0x0007u)
N#define CSL_CPUIER1_I2CINT_ENABLE_RESETVAL (0x0000u)
N
N#define CSL_CPUIER1_GPIOINT_ENABLE_MASK (0x0020u)
N#define CSL_CPUIER1_GPIOINT_ENABLE_SHIFT (0x0005u)
N#define CSL_CPUIER1_GPIOINT_ENABLE_RESETVAL (0x0000u)
N
N#define CSL_CPUIER1_USBINT_ENABLE_MASK (0x0010u)
N#define CSL_CPUIER1_USBINT_ENABLE_SHIFT (0x0004u)
N#define CSL_CPUIER1_USBINT_ENABLE_RESETVAL (0x0000u)
N
N#define CSL_CPUIER1_SPIINT_ENABLE_MASK (0x0008u)
N#define CSL_CPUIER1_SPIINT_ENABLE_SHIFT (0x0003u)
N#define CSL_CPUIER1_SPIINT_ENABLE_RESETVAL (0x0000u)
N
N#define CSL_CPUIER1_RTCINT_ENABLE_MASK (0x0004u)
N#define CSL_CPUIER1_RTCINT_ENABLE_SHIFT (0x0002u)
N#define CSL_CPUIER1_RTCINT_ENABLE_RESETVAL (0x0000u)
N
N#define CSL_CPUIER1_RCV3INT_ENABLE_MASK (0x0002u)
N#define CSL_CPUIER1_RCV3INT_ENABLE_SHIFT (0x0001u)
N#define CSL_CPUIER1_RCV3INT_ENABLE_RESETVAL (0x0000u)
N
N#define CSL_CPUIER1_XMT3INT_ENABLE_MASK (0x0001u)
N#define CSL_CPUIER1_XMT3INT_ENABLE_SHIFT (0x0000u)
N#define CSL_CPUIER1_XMT3INT_ENABLE_RESETVAL (0x0000u)
N
N#define CSL_CPUIER1_RESETVAL (0x0000u)
N
N
N/* CPU interrupt flag register0 */
N
N#define CSL_CPUIFR0_RESET_MASK (0xFFFFu)
N#define CSL_CPUIFR0_RESET_SHIFT (0x0000u)
N#define CSL_CPUIFR0_RESETVAL (0xFFFFu)
N
N
N/* CPU interrupt flag register1 */
N
N#define CSL_CPUIFR1_RESET_MASK (0xFFFFu)
N#define CSL_CPUIFR1_RESET_SHIFT (0x0000u)
N#define CSL_CPUIFR1_RESETVAL (0xFFFFu)
N
N
N#endif
L 24 "../inc/ECGDemoNonBios.h" 2
N#include "corazon.h"
L 1 "../common_inc/corazon.h" 1
N/*****************************************************************************
N * File Name : Corazon.h 
N *
N * Brief	 : This file contains the Chip Description for CORAZON
N *
N * Copyright (C) 2009 -2010 Texas Instruments Incorporated - http://www.ti.com/ 
N * 
N * 
N *  Redistribution and use in source and binary forms, with or without 
N *  modification, are permitted provided that the following conditions 
N *  are met:
N *
N *    Redistributions of source code must retain the above copyright 
N *    notice, this list of conditions and the following disclaimer.
N *
N *    Redistributions in binary form must reproduce the above copyright
N *    notice, this list of conditions and the following disclaimer in the 
N *    documentation and/or other materials provided with the   
N *    distribution.
N *
N *    Neither the name of Texas Instruments Incorporated nor the names of
N *    its contributors may be used to endorse or promote products derived
N *    from this software without specific prior written permission.
N *
N *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
N *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
N *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
N *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
N *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
N *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
N *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
N *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
N *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
N *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
N *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
N *
N******************************************************************************/
N#ifndef _CORAZON_H
N#define _CORAZON_H
N
N/*****************************************************************************/
N /** \file Corazon.h
N * 
N * \brief This file contains the Chip Description for CORAZON
N * 
N *****************************************************************************/
N
N#include <cslr.h>
N#include <tistdtypes.h>
N#include <csl_ioport.h>
N
N/*****************************************************************************\
N* Include files for all the modules in the device
X
N\*****************************************************************************/
N
N#include "cslr_i2c_001.h"
L 1 "..\common_inc\cslr_i2c_001.h" 1
N/*****************************************************************************
N * File Name : cslr_i2c_001.h 
N *
N * Brief	 : Define I2C register structure
N *
N * Copyright (C) 2009 -2010 Texas Instruments Incorporated - http://www.ti.com/ 
N * 
N * 
N *  Redistribution and use in source and binary forms, with or without 
N *  modification, are permitted provided that the following conditions 
N *  are met:
N *
N *    Redistributions of source code must retain the above copyright 
N *    notice, this list of conditions and the following disclaimer.
N *
N *    Redistributions in binary form must reproduce the above copyright
N *    notice, this list of conditions and the following disclaimer in the 
N *    documentation and/or other materials provided with the   
N *    distribution.
N *
N *    Neither the name of Texas Instruments Incorporated nor the names of
N *    its contributors may be used to endorse or promote products derived
N *    from this software without specific prior written permission.
N *
N *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
N *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
N *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
N *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
N *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
N *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
N *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
N *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
N *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
N *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
N *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
N *
N******************************************************************************/
N#ifndef _CSLR__I2C_1_H_
N#define _CSLR__I2C_1_H_
N
N#include "cslr.h"
N
N#include "stdtypes.h"
L 1 "..\common_inc\stdtypes.h" 1
N/*****************************************************************************
N * File Name : stdtypes.h 
N *
N * Brief	 : re-direct to the standard tistdtypes.h
N *
N * Copyright (C) 2009 -2010 Texas Instruments Incorporated - http://www.ti.com/ 
N * 
N * 
N *  Redistribution and use in source and binary forms, with or without 
N *  modification, are permitted provided that the following conditions 
N *  are met:
N *
N *    Redistributions of source code must retain the above copyright 
N *    notice, this list of conditions and the following disclaimer.
N *
N *    Redistributions in binary form must reproduce the above copyright
N *    notice, this list of conditions and the following disclaimer in the 
N *    documentation and/or other materials provided with the   
N *    distribution.
N *
N *    Neither the name of Texas Instruments Incorporated nor the names of
N *    its contributors may be used to endorse or promote products derived
N *    from this software without specific prior written permission.
N *
N *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
N *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
N *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
N *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
N *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
N *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
N *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
N *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
N *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
N *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
N *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
N *
N******************************************************************************/
N#ifndef _STD_TYPES
N#define _STD_TYPES
N
N/*
N * Just incase, if this file is included,
N * re-direct to the standard tistdtypes.h...
N */
N#include "tistdtypes.h"
N
N#endif /* _STDTYPES_H_ */
N
L 44 "..\common_inc\cslr_i2c_001.h" 2
N/* Minimum unit = 2 bytes */
N
N/**************************************************************************\
N* Register Overlay Structure
X
N\**************************************************************************/
Ntypedef struct  {
N    volatile Uint16 ICOAR;
N    volatile Uint16 RSVD0[3];
N    volatile Uint16 ICIMR;
N    volatile Uint16 RSVD1[3];
N    volatile Uint16 ICSTR;
N    volatile Uint16 RSVD2[3];
N    volatile Uint16 ICCLKL;
N    volatile Uint16 RSVD3[3];
N    volatile Uint16 ICCLKH;
N    volatile Uint16 RSVD4[3];
N    volatile Uint16 ICCNT;
N    volatile Uint16 RSVD5[3];
N    volatile Uint16 ICDRR;
N    volatile Uint16 RSVD6[3];
N    volatile Uint16 ICSAR;
N    volatile Uint16 RSVD7[3];
N    volatile Uint16 ICDXR;
N    volatile Uint16 RSVD8[3];
N    volatile Uint16 ICMDR;
N    volatile Uint16 RSVD9[3];
N    volatile Uint16 ICIVR;
N    volatile Uint16 RSVD10[3];
N    volatile Uint16 ICEMDR;
N    volatile Uint16 RSVD11[3];
N    volatile Uint16 ICPSC;
N    volatile Uint16 RSVD12[3];
N    volatile Uint16 ICPID1;
N    volatile Uint16 RSVD13[3];
N    volatile Uint16 ICPID2;
N    volatile Uint16 RSVD14[3];
N    volatile Uint16 ICDMAC;
N    volatile Uint16 RSVD15[11];
N    volatile Uint16 ICPFUNC;
N    volatile Uint16 RSVD16[3];
N    volatile Uint16 ICPDIR;
N    volatile Uint16 RSVD17[3];
N    volatile Uint16 ICPDIN;
N    volatile Uint16 RSVD18[3];
N    volatile Uint16 ICPDOUT;
N    volatile Uint16 RSVD19[3];
N    volatile Uint16 ICPDSET;
N    volatile Uint16 RSVD20[3];
N    volatile Uint16 ICPDCLR;
N    volatile Uint16 RSVD21[3];
N    volatile Uint16 ICPDRV;
N    volatile Uint16 RSVD22[3];
N    volatile Uint16 ICPPDIS;
N    volatile Uint16 RSVD23[3];
N    volatile Uint16 ICPPSEL;
N    volatile Uint16 RSVD24[3];
N    volatile Uint16 ICPSRS;
N} CSL_I2cRegs;
N
N/**************************************************************************\
N* Field Definition Macros
X
N\**************************************************************************/
N
N/* ICOAR */
N
N#define CSL_I2C_ICOAR_OADDR_MASK (0x03FFu)
N#define CSL_I2C_ICOAR_OADDR_SHIFT (0x0000u)
N#define CSL_I2C_ICOAR_OADDR_RESETVAL (0x0000u)
N
N#define CSL_I2C_ICOAR_RESETVAL (0x0000u)
N
N/* ICIMR */
N
N#define CSL_I2C_ICIMR_AAS_MASK (0x0040u)
N#define CSL_I2C_ICIMR_AAS_SHIFT (0x0006u)
N#define CSL_I2C_ICIMR_AAS_RESETVAL (0x0000u)
N/*----AAS Tokens----*/
N#define CSL_I2C_ICIMR_AAS_DISABLE (0x0000u)
N#define CSL_I2C_ICIMR_AAS_ENABLE (0x0001u)
N
N#define CSL_I2C_ICIMR_SCD_MASK (0x0020u)
N#define CSL_I2C_ICIMR_SCD_SHIFT (0x0005u)
N#define CSL_I2C_ICIMR_SCD_RESETVAL (0x0000u)
N/*----SCD Tokens----*/
N#define CSL_I2C_ICIMR_SCD_DISABLE (0x0000u)
N#define CSL_I2C_ICIMR_SCD_ENABLE (0x0001u)
N
N#define CSL_I2C_ICIMR_ICXRDY_MASK (0x0010u)
N#define CSL_I2C_ICIMR_ICXRDY_SHIFT (0x0004u)
N#define CSL_I2C_ICIMR_ICXRDY_RESETVAL (0x0000u)
N/*----ICXRDY Tokens----*/
N#define CSL_I2C_ICIMR_ICXRDY_DISABLE (0x0000u)
N#define CSL_I2C_ICIMR_ICXRDY_ENABLE (0x0001u)
N
N#define CSL_I2C_ICIMR_ICRRDY_MASK (0x0008u)
N#define CSL_I2C_ICIMR_ICRRDY_SHIFT (0x0003u)
N#define CSL_I2C_ICIMR_ICRRDY_RESETVAL (0x0000u)
N/*----ICRRDY Tokens----*/
N#define CSL_I2C_ICIMR_ICRRDY_DISABLE (0x0000u)
N#define CSL_I2C_ICIMR_ICRRDY_ENABLE (0x0001u)
N
N#define CSL_I2C_ICIMR_ARDY_MASK (0x0004u)
N#define CSL_I2C_ICIMR_ARDY_SHIFT (0x0002u)
N#define CSL_I2C_ICIMR_ARDY_RESETVAL (0x0000u)
N/*----ARDY Tokens----*/
N#define CSL_I2C_ICIMR_ARDY_DISABLE (0x0000u)
N#define CSL_I2C_ICIMR_ARDY_ENABLE (0x0001u)
N
N#define CSL_I2C_ICIMR_NACK_MASK (0x0002u)
N#define CSL_I2C_ICIMR_NACK_SHIFT (0x0001u)
N#define CSL_I2C_ICIMR_NACK_RESETVAL (0x0000u)
N/*----NACK Tokens----*/
N#define CSL_I2C_ICIMR_NACK_DISABLE (0x0000u)
N#define CSL_I2C_ICIMR_NACK_ENABLE (0x0001u)
N
N#define CSL_I2C_ICIMR_AL_MASK (0x0001u)
N#define CSL_I2C_ICIMR_AL_SHIFT (0x0000u)
N#define CSL_I2C_ICIMR_AL_RESETVAL (0x0000u)
N/*----AL Tokens----*/
N#define CSL_I2C_ICIMR_AL_DISABLE (0x0000u)
N#define CSL_I2C_ICIMR_AL_ENABLE (0x0001u)
N
N#define CSL_I2C_ICIMR_RESETVAL (0x0000u)
N
N/* ICSTR */
N
N#define CSL_I2C_ICSTR_SDIR_MASK (0x4000u)
N#define CSL_I2C_ICSTR_SDIR_SHIFT (0x000Eu)
N#define CSL_I2C_ICSTR_SDIR_RESETVAL (0x0000u)
N
N#define CSL_I2C_ICSTR_NACKSNT_MASK (0x2000u)
N#define CSL_I2C_ICSTR_NACKSNT_SHIFT (0x000Du)
N#define CSL_I2C_ICSTR_NACKSNT_RESETVAL (0x0000u)
N
N#define CSL_I2C_ICSTR_BB_MASK (0x1000u)
N#define CSL_I2C_ICSTR_BB_SHIFT (0x000Cu)
N#define CSL_I2C_ICSTR_BB_RESETVAL (0x0000u)
N
N#define CSL_I2C_ICSTR_RSFULL_MASK (0x0800u)
N#define CSL_I2C_ICSTR_RSFULL_SHIFT (0x000Bu)
N#define CSL_I2C_ICSTR_RSFULL_RESETVAL (0x0000u)
N
N#define CSL_I2C_ICSTR_XSMT_MASK (0x0400u)
N#define CSL_I2C_ICSTR_XSMT_SHIFT (0x000Au)
N#define CSL_I2C_ICSTR_XSMT_RESETVAL (0x0001u)
N
N#define CSL_I2C_ICSTR_AAS_MASK (0x0200u)
N#define CSL_I2C_ICSTR_AAS_SHIFT (0x0009u)
N#define CSL_I2C_ICSTR_AAS_RESETVAL (0x0000u)
N
N#define CSL_I2C_ICSTR_AD0_MASK (0x0100u)
N#define CSL_I2C_ICSTR_AD0_SHIFT (0x0008u)
N#define CSL_I2C_ICSTR_AD0_RESETVAL (0x0000u)
N
N
N#define CSL_I2C_ICSTR_SCD_MASK (0x0020u)
N#define CSL_I2C_ICSTR_SCD_SHIFT (0x0005u)
N#define CSL_I2C_ICSTR_SCD_RESETVAL (0x0000u)
N
N#define CSL_I2C_ICSTR_ICXRDY_MASK (0x0010u)
N#define CSL_I2C_ICSTR_ICXRDY_SHIFT (0x0004u)
N#define CSL_I2C_ICSTR_ICXRDY_RESETVAL (0x0001u)
N
N#define CSL_I2C_ICSTR_ICRRDY_MASK (0x0008u)
N#define CSL_I2C_ICSTR_ICRRDY_SHIFT (0x0003u)
N#define CSL_I2C_ICSTR_ICRRDY_RESETVAL (0x0000u)
N
N#define CSL_I2C_ICSTR_ARDY_MASK (0x0004u)
N#define CSL_I2C_ICSTR_ARDY_SHIFT (0x0002u)
N#define CSL_I2C_ICSTR_ARDY_RESETVAL (0x0000u)
N
N#define CSL_I2C_ICSTR_NACK_MASK (0x0002u)
N#define CSL_I2C_ICSTR_NACK_SHIFT (0x0001u)
N#define CSL_I2C_ICSTR_NACK_RESETVAL (0x0000u)
N
N#define CSL_I2C_ICSTR_AL_MASK (0x0001u)
N#define CSL_I2C_ICSTR_AL_SHIFT (0x0000u)
N#define CSL_I2C_ICSTR_AL_RESETVAL (0x0000u)
N
N#define CSL_I2C_ICSTR_RESETVAL (0x0410u)
N
N/* ICCLKL */
N
N#define CSL_I2C_ICCLKL_ICCL_MASK (0xFFFFu)
N#define CSL_I2C_ICCLKL_ICCL_SHIFT (0x0000u)
N#define CSL_I2C_ICCLKL_ICCL_RESETVAL (0x0000u)
N
N#define CSL_I2C_ICCLKL_RESETVAL (0x0000u)
N
N/* ICCLKH */
N
N#define CSL_I2C_ICCLKH_ICCH_MASK (0xFFFFu)
N#define CSL_I2C_ICCLKH_ICCH_SHIFT (0x0000u)
N#define CSL_I2C_ICCLKH_ICCH_RESETVAL (0x0000u)
N
N#define CSL_I2C_ICCLKH_RESETVAL (0x0000u)
N
N/* ICCNT */
N
N#define CSL_I2C_ICCNT_ICDC_MASK (0xFFFFu)
N#define CSL_I2C_ICCNT_ICDC_SHIFT (0x0000u)
N#define CSL_I2C_ICCNT_ICDC_RESETVAL (0x0000u)
N
N#define CSL_I2C_ICCNT_RESETVAL (0x0000u)
N
N/* ICDRR */
N
N
N#define CSL_I2C_ICDRR_D_MASK (0x00FFu)
N#define CSL_I2C_ICDRR_D_SHIFT (0x0000u)
N#define CSL_I2C_ICDRR_D_RESETVAL (0x0000u)
N
N#define CSL_I2C_ICDRR_RESETVAL (0x0000u)
N
N/* ICSAR */
N
N
N#define CSL_I2C_ICSAR_SADDR_MASK (0x03FFu)
N#define CSL_I2C_ICSAR_SADDR_SHIFT (0x0000u)
N#define CSL_I2C_ICSAR_SADDR_RESETVAL (0x03FFu)
N
N#define CSL_I2C_ICSAR_RESETVAL (0x03FFu)
N
N/* ICDXR */
N
N
N#define CSL_I2C_ICDXR_D_MASK (0x00FFu)
N#define CSL_I2C_ICDXR_D_SHIFT (0x0000u)
N#define CSL_I2C_ICDXR_D_RESETVAL (0x0000u)
N
N#define CSL_I2C_ICDXR_RESETVAL (0x0000u)
N
N/* ICMDR */
N
N#define CSL_I2C_ICMDR_NACKMOD_MASK (0x8000u)
N#define CSL_I2C_ICMDR_NACKMOD_SHIFT (0x000Fu)
N#define CSL_I2C_ICMDR_NACKMOD_RESETVAL (0x0000u)
N
N#define CSL_I2C_ICMDR_FREE_MASK (0x4000u)
N#define CSL_I2C_ICMDR_FREE_SHIFT (0x000Eu)
N#define CSL_I2C_ICMDR_FREE_RESETVAL (0x0000u)
N
N#define CSL_I2C_ICMDR_STT_MASK (0x2000u)
N#define CSL_I2C_ICMDR_STT_SHIFT (0x000Du)
N#define CSL_I2C_ICMDR_STT_RESETVAL (0x0000u)
N
N
N#define CSL_I2C_ICMDR_STP_MASK (0x0800u)
N#define CSL_I2C_ICMDR_STP_SHIFT (0x000Bu)
N#define CSL_I2C_ICMDR_STP_RESETVAL (0x0000u)
N
N#define CSL_I2C_ICMDR_MST_MASK (0x0400u)
N#define CSL_I2C_ICMDR_MST_SHIFT (0x000Au)
N#define CSL_I2C_ICMDR_MST_RESETVAL (0x0000u)
N
N#define CSL_I2C_ICMDR_TRX_MASK (0x0200u)
N#define CSL_I2C_ICMDR_TRX_SHIFT (0x0009u)
N#define CSL_I2C_ICMDR_TRX_RESETVAL (0x0000u)
N
N#define CSL_I2C_ICMDR_XA_MASK (0x0100u)
N#define CSL_I2C_ICMDR_XA_SHIFT (0x0008u)
N#define CSL_I2C_ICMDR_XA_RESETVAL (0x0000u)
N
N#define CSL_I2C_ICMDR_RM_MASK (0x0080u)
N#define CSL_I2C_ICMDR_RM_SHIFT (0x0007u)
N#define CSL_I2C_ICMDR_RM_RESETVAL (0x0000u)
N
N#define CSL_I2C_ICMDR_DLB_MASK (0x0040u)
N#define CSL_I2C_ICMDR_DLB_SHIFT (0x0006u)
N#define CSL_I2C_ICMDR_DLB_RESETVAL (0x0000u)
N
N#define CSL_I2C_ICMDR_IRS_MASK (0x0020u)
N#define CSL_I2C_ICMDR_IRS_SHIFT (0x0005u)
N#define CSL_I2C_ICMDR_IRS_RESETVAL (0x0000u)
N
N#define CSL_I2C_ICMDR_STB_MASK (0x0010u)
N#define CSL_I2C_ICMDR_STB_SHIFT (0x0004u)
N#define CSL_I2C_ICMDR_STB_RESETVAL (0x0000u)
N
N#define CSL_I2C_ICMDR_FDF_MASK (0x0008u)
N#define CSL_I2C_ICMDR_FDF_SHIFT (0x0003u)
N#define CSL_I2C_ICMDR_FDF_RESETVAL (0x0000u)
N
N#define CSL_I2C_ICMDR_BC_MASK (0x0007u)
N#define CSL_I2C_ICMDR_BC_SHIFT (0x0000u)
N#define CSL_I2C_ICMDR_BC_RESETVAL (0x0000u)
N
N#define CSL_I2C_ICMDR_RESETVAL (0x0000u)
N
N/* ICIVR */
N
N
N#define CSL_I2C_ICIVR_TESTMD_MASK (0x0F00u)
N#define CSL_I2C_ICIVR_TESTMD_SHIFT (0x0008u)
N#define CSL_I2C_ICIVR_TESTMD_RESETVAL (0x0000u)
N
N
N#define CSL_I2C_ICIVR_INTCODE_MASK (0x0007u)
N#define CSL_I2C_ICIVR_INTCODE_SHIFT (0x0000u)
N#define CSL_I2C_ICIVR_INTCODE_RESETVAL (0x0000u)
N/*----INTCODE Tokens----*/
N#define CSL_I2C_ICIVR_INTCODE_NONE (0x0000u)
N#define CSL_I2C_ICIVR_INTCODE_AL (0x0001u)
N#define CSL_I2C_ICIVR_INTCODE_NACK (0x0002u)
N#define CSL_I2C_ICIVR_INTCODE_RAR (0x0003u)
N#define CSL_I2C_ICIVR_INTCODE_RDR (0x0004u)
N#define CSL_I2C_ICIVR_INTCODE_TDR (0x0005u)
N#define CSL_I2C_ICIVR_INTCODE_SCD (0x0006u)
N#define CSL_I2C_ICIVR_INTCODE_AAS (0x0007u)
N
N#define CSL_I2C_ICIVR_RESETVAL (0x0000u)
N
N/* ICEMDR */
N
N
N#define CSL_I2C_ICEMDR_IGNACK_MASK (0x0002u)
N#define CSL_I2C_ICEMDR_IGNACK_SHIFT (0x0001u)
N#define CSL_I2C_ICEMDR_IGNACK_RESETVAL (0x0000u)
N
N#define CSL_I2C_ICEMDR_BCM_MASK (0x0001u)
N#define CSL_I2C_ICEMDR_BCM_SHIFT (0x0000u)
N#define CSL_I2C_ICEMDR_BCM_RESETVAL (0x0001u)
N
N#define CSL_I2C_ICEMDR_RESETVAL (0x0001u)
N
N/* ICPSC */
N
N
N#define CSL_I2C_ICPSC_IPSC_MASK (0x00FFu)
N#define CSL_I2C_ICPSC_IPSC_SHIFT (0x0000u)
N#define CSL_I2C_ICPSC_IPSC_RESETVAL (0x0000u)
N
N#define CSL_I2C_ICPSC_RESETVAL (0x0000u)
N
N/* ICPID1 */
N
N#define CSL_I2C_ICPID1_CLASS_MASK (0xFF00u)
N#define CSL_I2C_ICPID1_CLASS_SHIFT (0x0008u)
N#define CSL_I2C_ICPID1_CLASS_RESETVAL (0x0001u)
N
N#define CSL_I2C_ICPID1_REVISION_MASK (0x00FFu)
N#define CSL_I2C_ICPID1_REVISION_SHIFT (0x0000u)
N#define CSL_I2C_ICPID1_REVISION_RESETVAL (0x0025u)
N
N#define CSL_I2C_ICPID1_RESETVAL (0x0125u)
N
N/* ICPID2 */
N
N
N#define CSL_I2C_ICPID2_TYPE_MASK (0x00FFu)
N#define CSL_I2C_ICPID2_TYPE_SHIFT (0x0000u)
N#define CSL_I2C_ICPID2_TYPE_RESETVAL (0x0005u)
N
N#define CSL_I2C_ICPID2_RESETVAL (0x0005u)
N
N/* ICDMAC */
N
N
N#define CSL_I2C_ICDMAC_TXDMAEN_MASK (0x0002u)
N#define CSL_I2C_ICDMAC_TXDMAEN_SHIFT (0x0001u)
N#define CSL_I2C_ICDMAC_TXDMAEN_RESETVAL (0x0001u)
N
N#define CSL_I2C_ICDMAC_RXDMAEN_MASK (0x0001u)
N#define CSL_I2C_ICDMAC_RXDMAEN_SHIFT (0x0000u)
N#define CSL_I2C_ICDMAC_RXDMAEN_RESETVAL (0x0001u)
N
N#define CSL_I2C_ICDMAC_RESETVAL (0x0003u)
N
N/* ICPFUNC */
N
N
N#define CSL_I2C_ICPFUNC_PFUNC_MASK (0x0001u)
N#define CSL_I2C_ICPFUNC_PFUNC_SHIFT (0x0000u)
N#define CSL_I2C_ICPFUNC_PFUNC_RESETVAL (0x0000u)
N
N#define CSL_I2C_ICPFUNC_RESETVAL (0x0000u)
N
N/* ICPDIR */
N
N
N#define CSL_I2C_ICPDIR_PDIR1_MASK (0x0002u)
N#define CSL_I2C_ICPDIR_PDIR1_SHIFT (0x0001u)
N#define CSL_I2C_ICPDIR_PDIR1_RESETVAL (0x0000u)
N/*----PDIR1 Tokens----*/
N#define CSL_I2C_ICPDIR_PDIR1_IN (0x0000u)
N#define CSL_I2C_ICPDIR_PDIR1_OUT (0x0001u)
N
N#define CSL_I2C_ICPDIR_PDIR0_MASK (0x0001u)
N#define CSL_I2C_ICPDIR_PDIR0_SHIFT (0x0000u)
N#define CSL_I2C_ICPDIR_PDIR0_RESETVAL (0x0000u)
N/*----PDIR0 Tokens----*/
N#define CSL_I2C_ICPDIR_PDIR0_IN (0x0000u)
N#define CSL_I2C_ICPDIR_PDIR0_OUT (0x0001u)
N
N#define CSL_I2C_ICPDIR_RESETVAL (0x0000u)
N
N/* ICPDIN */
N
N
N#define CSL_I2C_ICPDIN_PDIN1_MASK (0x0002u)
N#define CSL_I2C_ICPDIN_PDIN1_SHIFT (0x0001u)
N#define CSL_I2C_ICPDIN_PDIN1_RESETVAL (0x0000u)
N/*----PDIN1 Tokens----*/
N#define CSL_I2C_ICPDIN_PDIN1_LOW (0x0000u)
N#define CSL_I2C_ICPDIN_PDIN1_HIGH (0x0001u)
N
N#define CSL_I2C_ICPDIN_PDIN0_MASK (0x0001u)
N#define CSL_I2C_ICPDIN_PDIN0_SHIFT (0x0000u)
N#define CSL_I2C_ICPDIN_PDIN0_RESETVAL (0x0000u)
N/*----PDIN0 Tokens----*/
N#define CSL_I2C_ICPDIN_PDIN0_LOW (0x0000u)
N#define CSL_I2C_ICPDIN_PDIN0_HIGH (0x0001u)
N
N#define CSL_I2C_ICPDIN_RESETVAL (0x0000u)
N
N/* ICPDOUT */
N
N
N#define CSL_I2C_ICPDOUT_PDOUT1_MASK (0x0002u)
N#define CSL_I2C_ICPDOUT_PDOUT1_SHIFT (0x0001u)
N#define CSL_I2C_ICPDOUT_PDOUT1_RESETVAL (0x0000u)
N/*----PDOUT1 Tokens----*/
N#define CSL_I2C_ICPDOUT_PDOUT1_LOW (0x0000u)
N#define CSL_I2C_ICPDOUT_PDOUT1_HIGH (0x0001u)
N
N#define CSL_I2C_ICPDOUT_PDOUT0_MASK (0x0001u)
N#define CSL_I2C_ICPDOUT_PDOUT0_SHIFT (0x0000u)
N#define CSL_I2C_ICPDOUT_PDOUT0_RESETVAL (0x0000u)
N/*----PDOUT0 Tokens----*/
N#define CSL_I2C_ICPDOUT_PDOUT0_LOW (0x0000u)
N#define CSL_I2C_ICPDOUT_PDOUT0_HIGH (0x0001u)
N
N#define CSL_I2C_ICPDOUT_RESETVAL (0x0000u)
N
N/* ICPDSET */
N
N
N#define CSL_I2C_ICPDSET_PDSET1_MASK (0x0002u)
N#define CSL_I2C_ICPDSET_PDSET1_SHIFT (0x0001u)
N#define CSL_I2C_ICPDSET_PDSET1_RESETVAL (0x0000u)
N/*----PDSET1 Tokens----*/
N#define CSL_I2C_ICPDSET_PDSET1_NONE (0x0000u)
N#define CSL_I2C_ICPDSET_PDSET1_SET (0x0001u)
N
N#define CSL_I2C_ICPDSET_PDSET0_MASK (0x0001u)
N#define CSL_I2C_ICPDSET_PDSET0_SHIFT (0x0000u)
N#define CSL_I2C_ICPDSET_PDSET0_RESETVAL (0x0000u)
N/*----PDSET0 Tokens----*/
N#define CSL_I2C_ICPDSET_PDSET0_NONE (0x0000u)
N#define CSL_I2C_ICPDSET_PDSET0_SET (0x0001u)
N
N#define CSL_I2C_ICPDSET_RESETVAL (0x0000u)
N
N/* ICPDCLR */
N
N
N#define CSL_I2C_ICPDCLR_PDCLR1_MASK (0x0002u)
N#define CSL_I2C_ICPDCLR_PDCLR1_SHIFT (0x0001u)
N#define CSL_I2C_ICPDCLR_PDCLR1_RESETVAL (0x0000u)
N/*----PDCLR1 Tokens----*/
N#define CSL_I2C_ICPDCLR_PDCLR1_NONE (0x0000u)
N#define CSL_I2C_ICPDCLR_PDCLR1_RESET (0x0001u)
N
N#define CSL_I2C_ICPDCLR_PDCLR0_MASK (0x0001u)
N#define CSL_I2C_ICPDCLR_PDCLR0_SHIFT (0x0000u)
N#define CSL_I2C_ICPDCLR_PDCLR0_RESETVAL (0x0000u)
N/*----PDCLR0 Tokens----*/
N#define CSL_I2C_ICPDCLR_PDCLR0_NONE (0x0000u)
N#define CSL_I2C_ICPDCLR_PDCLR0_RESET (0x0001u)
N
N#define CSL_I2C_ICPDCLR_RESETVAL (0x0000u)
N
N/* ICPDRV */
N
N
N#define CSL_I2C_ICPDRV_PDRV1_MASK (0x0002u)
N#define CSL_I2C_ICPDRV_PDRV1_SHIFT (0x0001u)
N#define CSL_I2C_ICPDRV_PDRV1_RESETVAL (0x0001u)
N
N#define CSL_I2C_ICPDRV_PDRV0_MASK (0x0001u)
N#define CSL_I2C_ICPDRV_PDRV0_SHIFT (0x0000u)
N#define CSL_I2C_ICPDRV_PDRV0_RESETVAL (0x0001u)
N
N#define CSL_I2C_ICPDRV_RESETVAL (0x0003u)
N
N/* ICPPDIS */
N
N
N#define CSL_I2C_ICPPDIS_PPDIS1_MASK (0x0002u)
N#define CSL_I2C_ICPPDIS_PPDIS1_SHIFT (0x0001u)
N#define CSL_I2C_ICPPDIS_PPDIS1_RESETVAL (0x0001u)
N
N#define CSL_I2C_ICPPDIS_PPDIS0_MASK (0x0001u)
N#define CSL_I2C_ICPPDIS_PPDIS0_SHIFT (0x0000u)
N#define CSL_I2C_ICPPDIS_PPDIS0_RESETVAL (0x0001u)
N
N#define CSL_I2C_ICPPDIS_RESETVAL (0x0003u)
N
N/* ICPPSEL */
N
N
N#define CSL_I2C_ICPPSEL_PPSEL1_MASK (0x0002u)
N#define CSL_I2C_ICPPSEL_PPSEL1_SHIFT (0x0001u)
N#define CSL_I2C_ICPPSEL_PPSEL1_RESETVAL (0x0000u)
N
N#define CSL_I2C_ICPPSEL_PPSEL0_MASK (0x0001u)
N#define CSL_I2C_ICPPSEL_PPSEL0_SHIFT (0x0000u)
N#define CSL_I2C_ICPPSEL_PPSEL0_RESETVAL (0x0000u)
N
N#define CSL_I2C_ICPPSEL_RESETVAL (0x0000u)
N
N/* ICPSRS */
N
N
N#define CSL_I2C_ICPSRS_PSRS1_MASK (0x0002u)
N#define CSL_I2C_ICPSRS_PSRS1_SHIFT (0x0001u)
N#define CSL_I2C_ICPSRS_PSRS1_RESETVAL (0x0000u)
N
N#define CSL_I2C_ICPSRS_PSRS0_MASK (0x0001u)
N#define CSL_I2C_ICPSRS_PSRS0_SHIFT (0x0000u)
N#define CSL_I2C_ICPSRS_PSRS0_RESETVAL (0x0000u)
N
N#define CSL_I2C_ICPSRS_RESETVAL (0x0000u)
N
N#endif
N
L 57 "../common_inc/corazon.h" 2
N#include "cslr_i2s_001.h"
L 1 "..\common_inc\cslr_i2s_001.h" 1
N/*****************************************************************************
N * File Name : cslr_i2s_001.h 
N *
N * Brief	 : Define I2S register structure
N *
N * Copyright (C) 2009 -2010 Texas Instruments Incorporated - http://www.ti.com/ 
N * 
N * 
N *  Redistribution and use in source and binary forms, with or without 
N *  modification, are permitted provided that the following conditions 
N *  are met:
N *
N *    Redistributions of source code must retain the above copyright 
N *    notice, this list of conditions and the following disclaimer.
N *
N *    Redistributions in binary form must reproduce the above copyright
N *    notice, this list of conditions and the following disclaimer in the 
N *    documentation and/or other materials provided with the   
N *    distribution.
N *
N *    Neither the name of Texas Instruments Incorporated nor the names of
N *    its contributors may be used to endorse or promote products derived
N *    from this software without specific prior written permission.
N *
N *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
N *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
N *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
N *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
N *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
N *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
N *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
N *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
N *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
N *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
N *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
N *
N******************************************************************************/
N#ifndef _CSLR__I2S_1_H_
N#define _CSLR__I2S_1_H_
N
N#include <cslr.h>
N
N/* Minimum unit = 2 bytes */
N
N/**************************************************************************\
N* Register Overlay Structure
X
N\**************************************************************************/
Ntypedef struct  {
N    volatile Uint16 SCRL;
N    volatile Uint16 SCRM;
N    volatile Uint16 RSVD0[2];
N    volatile Uint16 SRGRL;
N    volatile Uint16 SRGRM;
N    volatile Uint16 RSVD1[2];
N    volatile Uint16 TRW0L;
N    volatile Uint16 TRW0M;
N    volatile Uint16 RSVD2[2];
N    volatile Uint16 TRW1L;
N    volatile Uint16 TRW1M;
N    volatile Uint16 RSVD3[2];
N    volatile Uint16 IRL;
N    volatile Uint16 IRM;
N    volatile Uint16 RSVD4[2];
N    volatile Uint16 ICMRL;
N    volatile Uint16 ICMRM;
N    volatile Uint16 RSVD5[18];
N    volatile Uint16 RRW0L;
N    volatile Uint16 RRW0M;
N    volatile Uint16 RSVD6[2];
N    volatile Uint16 RRW1L;
N    volatile Uint16 RRW1M;
N} CSL_I2sRegs;
N
N/**************************************************************************\
N* Field Definition Macros
X
N\**************************************************************************/
N
N/* SCRL */
N
N#define CSL_I2S_SCRL_ENABLE_MASK (0x8000u)
N#define CSL_I2S_SCRL_ENABLE_SHIFT (0x000Fu)
N#define CSL_I2S_SCRL_ENABLE_RESETVAL (0x0000u)
N
N#define CSL_I2S_SCRL_MONO_MASK (0x1000u)
N#define CSL_I2S_SCRL_MONO_SHIFT (0x000Cu)
N#define CSL_I2S_SCRL_MONO_RESETVAL (0x0000u)
N
N#define CSL_I2S_SCRL_LOOPBACK_MASK (0x0800u)
N#define CSL_I2S_SCRL_LOOPBACK_SHIFT (0x000Bu)
N#define CSL_I2S_SCRL_LOOPBACK_RESETVAL (0x0000u)
N
N#define CSL_I2S_SCRL_FSPOL_MASK (0x0400u)
N#define CSL_I2S_SCRL_FSPOL_SHIFT (0x000Au)
N#define CSL_I2S_SCRL_FSPOL_RESETVAL (0x0000u)
N
N#define CSL_I2S_SCRL_CLOCKPOL_MASK (0x0200u)
N#define CSL_I2S_SCRL_CLOCKPOL_SHIFT (0x0009u)
N#define CSL_I2S_SCRL_CLOCKPOL_RESETVAL (0x0000u)
N
N#define CSL_I2S_SCRL_DATADELAY_MASK (0x0100u)
N#define CSL_I2S_SCRL_DATADELAY_SHIFT (0x0008u)
N#define CSL_I2S_SCRL_DATADELAY_RESETVAL (0x0000u)
N
N#define CSL_I2S_SCRL_PACK_MASK (0x0080u)
N#define CSL_I2S_SCRL_PACK_SHIFT (0x0007u)
N#define CSL_I2S_SCRL_PACK_RESETVAL (0x0000u)
N
N#define CSL_I2S_SCRL_SIGNEXT_MASK (0x0040u)
N#define CSL_I2S_SCRL_SIGNEXT_SHIFT (0x0006u)
N#define CSL_I2S_SCRL_SIGNEXT_RESETVAL (0x0000u)
N
N#define CSL_I2S_SCRL_WORDLENGTH_MASK (0x003Cu)
N#define CSL_I2S_SCRL_WORDLENGTH_SHIFT (0x0002u)
N#define CSL_I2S_SCRL_WORDLENGTH_RESETVAL (0x0000u)
N
N#define CSL_I2S_SCRL_MODE_MASK (0x0002u)
N#define CSL_I2S_SCRL_MODE_SHIFT (0x0001u)
N#define CSL_I2S_SCRL_MODE_RESETVAL (0x0000u)
N
N#define CSL_I2S_SCRL_FORMAT_MASK (0x0001u)
N#define CSL_I2S_SCRL_FORMAT_SHIFT (0x0000u)
N#define CSL_I2S_SCRL_FORMAT_RESETVAL (0x0000u)
N
N#define CSL_I2S_SCRL_RESETVAL (0x0000u)
N
N/* SCRM */
N
N
N#define CSL_I2S_SCRM_RESETVAL (0x0000u)
N
N/* SRGRL */
N
N
N#define CSL_I2S_SRGRL_FSDIV_MASK (0x0038u)
N#define CSL_I2S_SRGRL_FSDIV_SHIFT (0x0003u)
N#define CSL_I2S_SRGRL_FSDIV_RESETVAL (0x0000u)
N
N#define CSL_I2S_SRGRL_CLOCKDIV_MASK (0x0007u)
N#define CSL_I2S_SRGRL_CLOCKDIV_SHIFT (0x0000u)
N#define CSL_I2S_SRGRL_CLOCKDIV_RESETVAL (0x0000u)
N
N#define CSL_I2S_SRGRL_RESETVAL (0x0000u)
N
N/* SRGRM */
N
N
N
N#define CSL_I2S_SRGRM_RESETVAL (0x0000u)
N
N/* TRW0L */
N
N#define CSL_I2S_TRW0L_DATA_MASK (0xFFFFu)
N#define CSL_I2S_TRW0L_DATA_SHIFT (0x0000u)
N#define CSL_I2S_TRW0L_DATA_RESETVAL (0x0000u)
N
N#define CSL_I2S_TRW0L_RESETVAL (0x0000u)
N
N/* TRW0M */
N
N#define CSL_I2S_TRW0M_DATA_MASK (0xFFFFu)
N#define CSL_I2S_TRW0M_DATA_SHIFT (0x0000u)
N#define CSL_I2S_TRW0M_DATA_RESETVAL (0x0000u)
N
N#define CSL_I2S_TRW0M_RESETVAL (0x0000u)
N
N/* TRW1L */
N
N#define CSL_I2S_TRW1L_DATA_MASK (0xFFFFu)
N#define CSL_I2S_TRW1L_DATA_SHIFT (0x0000u)
N#define CSL_I2S_TRW1L_DATA_RESETVAL (0x0000u)
N
N#define CSL_I2S_TRW1L_RESETVAL (0x0000u)
N
N/* TRW1M */
N
N#define CSL_I2S_TRW1M_DATA_MASK (0xFFFFu)
N#define CSL_I2S_TRW1M_DATA_SHIFT (0x0000u)
N#define CSL_I2S_TRW1M_DATA_RESETVAL (0x0000u)
N
N#define CSL_I2S_TRW1M_RESETVAL (0x0000u)
N
N/* IRL */
N
N
N#define CSL_I2S_IRL_XMIT1_MASK (0x0020u)
N#define CSL_I2S_IRL_XMIT1_SHIFT (0x0005u)
N#define CSL_I2S_IRL_XMIT1_RESETVAL (0x0000u)
N
N#define CSL_I2S_IRL_XMIT0_MASK (0x0010u)
N#define CSL_I2S_IRL_XMIT0_SHIFT (0x0004u)
N#define CSL_I2S_IRL_XMIT0_RESETVAL (0x0000u)
N
N#define CSL_I2S_IRL_RCV1_MASK (0x0008u)
N#define CSL_I2S_IRL_RCV1_SHIFT (0x0003u)
N#define CSL_I2S_IRL_RCV1_RESETVAL (0x0000u)
N
N#define CSL_I2S_IRL_RCV0_MASK (0x0004u)
N#define CSL_I2S_IRL_RCV0_SHIFT (0x0002u)
N#define CSL_I2S_IRL_RCV0_RESETVAL (0x0000u)
N
N#define CSL_I2S_IRL_FERR_MASK (0x0002u)
N#define CSL_I2S_IRL_FERR_SHIFT (0x0001u)
N#define CSL_I2S_IRL_FERR_RESETVAL (0x0000u)
N
N#define CSL_I2S_IRL_O_U_MASK (0x0001u)
N#define CSL_I2S_IRL_O_U_SHIFT (0x0000u)
N#define CSL_I2S_IRL_O_U_RESETVAL (0x0000u)
N
N#define CSL_I2S_IRL_RESETVAL (0x0000u)
N
N/* IRM */
N
N
N#define CSL_I2S_IRM_RESETVAL (0x0000u)
N
N/* ICMRL */
N
N
N#define CSL_I2S_ICMRL_XMIT1_MASK (0x0020u)
N#define CSL_I2S_ICMRL_XMIT1_SHIFT (0x0005u)
N#define CSL_I2S_ICMRL_XMIT1_RESETVAL (0x0000u)
N
N#define CSL_I2S_ICMRL_XMIT0_MASK (0x0010u)
N#define CSL_I2S_ICMRL_XMIT0_SHIFT (0x0004u)
N#define CSL_I2S_ICMRL_XMIT0_RESETVAL (0x0000u)
N
N#define CSL_I2S_ICMRL_RCV1_MASK (0x0008u)
N#define CSL_I2S_ICMRL_RCV1_SHIFT (0x0003u)
N#define CSL_I2S_ICMRL_RCV1_RESETVAL (0x0000u)
N
N#define CSL_I2S_ICMRL_RCV0_MASK (0x0004u)
N#define CSL_I2S_ICMRL_RCV0_SHIFT (0x0002u)
N#define CSL_I2S_ICMRL_RCV0_RESETVAL (0x0000u)
N
N#define CSL_I2S_ICMRL_FERR_MASK (0x0002u)
N#define CSL_I2S_ICMRL_FERR_SHIFT (0x0001u)
N#define CSL_I2S_ICMRL_FERR_RESETVAL (0x0000u)
N
N#define CSL_I2S_ICMRL_O_U_MASK (0x0001u)
N#define CSL_I2S_ICMRL_O_U_SHIFT (0x0000u)
N#define CSL_I2S_ICMRL_O_U_RESETVAL (0x0000u)
N
N#define CSL_I2S_ICMRL_RESETVAL (0x0000u)
N
N/* ICMRM */
N
N
N#define CSL_I2S_ICMRM_RESETVAL (0x0000u)
N
N/* RRW0L */
N
N#define CSL_I2S_RRW0L_DATA_MASK (0xFFFFu)
N#define CSL_I2S_RRW0L_DATA_SHIFT (0x0000u)
N#define CSL_I2S_RRW0L_DATA_RESETVAL (0x0000u)
N
N#define CSL_I2S_RRW0L_RESETVAL (0x0000u)
N
N/* RRW0M */
N
N#define CSL_I2S_RRW0M_DATA_MASK (0xFFFFu)
N#define CSL_I2S_RRW0M_DATA_SHIFT (0x0000u)
N#define CSL_I2S_RRW0M_DATA_RESETVAL (0x0000u)
N
N#define CSL_I2S_RRW0M_RESETVAL (0x0000u)
N
N/* RRW1L */
N
N#define CSL_I2S_RRW1L_DATA_MASK (0xFFFFu)
N#define CSL_I2S_RRW1L_DATA_SHIFT (0x0000u)
N#define CSL_I2S_RRW1L_DATA_RESETVAL (0x0000u)
N
N#define CSL_I2S_RRW1L_RESETVAL (0x0000u)
N
N/* RRW1M */
N
N#define CSL_I2S_RRW1M_DATA_MASK (0xFFFFu)
N#define CSL_I2S_RRW1M_DATA_SHIFT (0x0000u)
N#define CSL_I2S_RRW1M_DATA_RESETVAL (0x0000u)
N
N#define CSL_I2S_RRW1M_RESETVAL (0x0000u)
N
N#endif
L 58 "../common_inc/corazon.h" 2
N#include "cslr_emif_001.h"
L 1 "..\common_inc\cslr_emif_001.h" 1
N/*****************************************************************************
N * File Name : cslr_emif_001.h 
N *
N * Brief	 : Define EMIF register structure
N *
N * Copyright (C) 2009 -2010 Texas Instruments Incorporated - http://www.ti.com/ 
N * 
N * 
N *  Redistribution and use in source and binary forms, with or without 
N *  modification, are permitted provided that the following conditions 
N *  are met:
N *
N *    Redistributions of source code must retain the above copyright 
N *    notice, this list of conditions and the following disclaimer.
N *
N *    Redistributions in binary form must reproduce the above copyright
N *    notice, this list of conditions and the following disclaimer in the 
N *    documentation and/or other materials provided with the   
N *    distribution.
N *
N *    Neither the name of Texas Instruments Incorporated nor the names of
N *    its contributors may be used to endorse or promote products derived
N *    from this software without specific prior written permission.
N *
N *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
N *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
N *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
N *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
N *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
N *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
N *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
N *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
N *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
N *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
N *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
N *
N******************************************************************************/
N#ifndef _CSLR__EMIF_1_H_
N#define _CSLR__EMIF_1_H_
N
N#include <cslr.h>
N
N/* Minimum unit = 2 bytes */
N
N/**************************************************************************\
N* Register Overlay Structure
X
N\**************************************************************************/
Ntypedef struct  {
N    volatile Uint16 REV;
N    volatile Uint16 STATUS;
N    volatile Uint16 RSVD0[2];
N    volatile Uint16 ASYNCCONFIG0;
N    volatile Uint16 ASYNCCONFIG1;
N    volatile Uint16 RSVD1[2];
N    volatile Uint16 SDRAMCONFIG0;
N    volatile Uint16 SDRAMCONFIG1;
N    volatile Uint16 RSVD2[2];
N    volatile Uint16 SDRCREFCTRL0;
N    volatile Uint16 SDRCREFCTRL1;
N    volatile Uint16 RSVD3[2];
N    volatile Uint16 ASYNC1CTRL0;
N    volatile Uint16 ASYNC1CTRL1;
N    volatile Uint16 RSVD4[2];
N    volatile Uint16 ASYNC2CTRL0;
N    volatile Uint16 ASYNC2CTRL1;
N    volatile Uint16 RSVD5[2];
N    volatile Uint16 ASYNC3CTRL0;
N    volatile Uint16 ASYNC3CTRL1;
N    volatile Uint16 RSVD6[2];
N    volatile Uint16 ASYNC4CTRL0;
N    volatile Uint16 ASYNC4CTRL1;
N    volatile Uint16 RSVD7[2];
N    volatile Uint16 SDRAMTIMING0;
N    volatile Uint16 SDRAMTIMING1;
N    volatile Uint16 RSVD8[2];
N    volatile Uint16 SDRAMSTAT;
N    volatile Uint16 RSVD9[11];
N    volatile Uint16 SDRACCESS0;
N    volatile Uint16 SDRACCESS1;
N    volatile Uint16 RSVD10[2];
N    volatile Uint16 SDRACTIVATE0;
N    volatile Uint16 SDRACTIVATE1;
N    volatile Uint16 RSVD11[6];
N    volatile Uint16 SDRCSRPD;
N    volatile Uint16 RSVD12[3];
N    volatile Uint16 INTRAW;
N    volatile Uint16 RSVD13[3];
N    volatile Uint16 INTMASK;
N    volatile Uint16 RSVD14[3];
N    volatile Uint16 INTMASKSET;
N    volatile Uint16 RSVD15[3];
N    volatile Uint16 INTMASKCLEAR;
N    volatile Uint16 RSVD16[3];
N    volatile Uint16 IOCTRL;
N    volatile Uint16 RSVD17[3];
N    volatile Uint16 IOSTATUS;
N    volatile Uint16 RSVD18[11];
N    volatile Uint16 NANDCTRL;
N    volatile Uint16 RSVD19[3];
N    volatile Uint16 NANDSTAT0;
N    volatile Uint16 NANDSTAT1;
N    volatile Uint16 RSVD20[2];
N    volatile Uint16 PAGEMODCTRL0;
N    volatile Uint16 PAGEMODCTRL1;
N    volatile Uint16 RSVD21[6];
N    volatile Uint16 CS2ECC0;
N    volatile Uint16 CS2ECC1;
N    volatile Uint16 RSVD22[2];
N    volatile Uint16 CS3ECC0;
N    volatile Uint16 CS3ECC1;
N    volatile Uint16 RSVD23[2];
N    volatile Uint16 CS4ECC0;
N    volatile Uint16 CS4ECC1;
N    volatile Uint16 RSVD24[2];
N    volatile Uint16 CS5ECC0;
N    volatile Uint16 CS5ECC1;
N    volatile Uint16 RSVD25[50];
N    volatile Uint16 VERSION;
N    volatile Uint16 RSVD26[11];
N    volatile Uint16 FOURBITECCLD;
N    volatile Uint16 RSVD27[3];
N    volatile Uint16 FOURBITECC10;
N    volatile Uint16 FOURBITECC11;
N    volatile Uint16 RSVD28[2];
N    volatile Uint16 FOURBITECC20;
N    volatile Uint16 FOURBITECC21;
N    volatile Uint16 RSVD29[2];
N    volatile Uint16 FOURBITECC30;
N    volatile Uint16 FOURBITECC31;
N    volatile Uint16 RSVD30[2];
N    volatile Uint16 FOURBITECC40;
N    volatile Uint16 FOURBITECC41;
N    volatile Uint16 RSVD31[2];
N    volatile Uint16 ERRADRR10;
N    volatile Uint16 ERRADRR11;
N    volatile Uint16 RSVD32[2];
N    volatile Uint16 ERRADRR20;
N    volatile Uint16 ERRADRR21;
N    volatile Uint16 RSVD33[2];
N    volatile Uint16 ERRVAL10;
N    volatile Uint16 ERRVAL11;
N    volatile Uint16 RSVD34[2];
N    volatile Uint16 ERRVAL20;
N    volatile Uint16 ERRVAL21;
N} CSL_EmifRegs;
N
N/**************************************************************************\
N* Field Definition Macros
X
N\**************************************************************************/
N
N/* REV */
N
N#define CSL_EMIF_REV_REVISION_MASK (0xFFFFu)
N#define CSL_EMIF_REV_REVISION_SHIFT (0x0000u)
N#define CSL_EMIF_REV_REVISION_RESETVAL (0x0000u)
N
N#define CSL_EMIF_REV_RESETVAL (0x0000u)
N
N/* STATUS */
N
N#define CSL_EMIF_STATUS_BIGENDIAN_MASK (0x8000u)
N#define CSL_EMIF_STATUS_BIGENDIAN_SHIFT (0x000Fu)
N#define CSL_EMIF_STATUS_BIGENDIAN_RESETVAL (0x0000u)
N
N#define CSL_EMIF_STATUS_FULLRATE_MASK (0x4000u)
N#define CSL_EMIF_STATUS_FULLRATE_SHIFT (0x000Eu)
N#define CSL_EMIF_STATUS_FULLRATE_RESETVAL (0x000Fu)
N
N#define CSL_EMIF_STATUS_MODID_MASK (0x3FFFu)
N#define CSL_EMIF_STATUS_MODID_SHIFT (0x0000u)
N#define CSL_EMIF_STATUS_MODID_RESETVAL (0x0000u)
N
N#define CSL_EMIF_STATUS_RESETVAL (0xC003u)
N
N/* ASYNCCONFIG0 */
N
N
N#define CSL_EMIF_ASYNCCONFIG0_MAX_EXT_WAIT_MASK (0x00FFu)
N#define CSL_EMIF_ASYNCCONFIG0_MAX_EXT_WAIT_SHIFT (0x0000u)
N#define CSL_EMIF_ASYNCCONFIG0_MAX_EXT_WAIT_RESETVAL (0x0080u)
N
N#define CSL_EMIF_ASYNCCONFIG0_RESETVAL (0x0080u)
N
N/* ASYNCCONFIG1 */
N
N#define CSL_EMIF_ASYNCCONFIG1_WP3_MASK (0x8000u)
N#define CSL_EMIF_ASYNCCONFIG1_WP3_SHIFT (0x000Fu)
N#define CSL_EMIF_ASYNCCONFIG1_WP3_RESETVAL (0x0001u)
N
N#define CSL_EMIF_ASYNCCONFIG1_WP2_MASK (0x4000u)
N#define CSL_EMIF_ASYNCCONFIG1_WP2_SHIFT (0x000Eu)
N#define CSL_EMIF_ASYNCCONFIG1_WP2_RESETVAL (0x0001u)
N
N#define CSL_EMIF_ASYNCCONFIG1_WP1_MASK (0x2000u)
N#define CSL_EMIF_ASYNCCONFIG1_WP1_SHIFT (0x000Du)
N#define CSL_EMIF_ASYNCCONFIG1_WP1_RESETVAL (0x0001u)
N
N#define CSL_EMIF_ASYNCCONFIG1_WP0_MASK (0x1000u)
N#define CSL_EMIF_ASYNCCONFIG1_WP0_SHIFT (0x000Cu)
N#define CSL_EMIF_ASYNCCONFIG1_WP0_RESETVAL (0x0001u)
N
N
N#define CSL_EMIF_ASYNCCONFIG1_CS5_WAIT_MASK (0x00C0u)
N#define CSL_EMIF_ASYNCCONFIG1_CS5_WAIT_SHIFT (0x0006u)
N#define CSL_EMIF_ASYNCCONFIG1_CS5_WAIT_RESETVAL (0x0000u)
N
N#define CSL_EMIF_ASYNCCONFIG1_CS4_WAIT_MASK (0x0030u)
N#define CSL_EMIF_ASYNCCONFIG1_CS4_WAIT_SHIFT (0x0004u)
N#define CSL_EMIF_ASYNCCONFIG1_CS4_WAIT_RESETVAL (0x0000u)
N
N#define CSL_EMIF_ASYNCCONFIG1_CS3_WAIT_MASK (0x000Cu)
N#define CSL_EMIF_ASYNCCONFIG1_CS3_WAIT_SHIFT (0x0002u)
N#define CSL_EMIF_ASYNCCONFIG1_CS3_WAIT_RESETVAL (0x0000u)
N
N#define CSL_EMIF_ASYNCCONFIG1_CS2_WAIT_MASK (0x0003u)
N#define CSL_EMIF_ASYNCCONFIG1_CS2_WAIT_SHIFT (0x0000u)
N#define CSL_EMIF_ASYNCCONFIG1_CS2_WAIT_RESETVAL (0x0000u)
N
N#define CSL_EMIF_ASYNCCONFIG1_RESETVAL (0xF000u)
N
N/* SDRAMCONFIG0 */
N
N
N#define CSL_EMIF_SDRAMCONFIG0_NM_MASK (0x4000u)
N#define CSL_EMIF_SDRAMCONFIG0_NM_SHIFT (0x000Eu)
N#define CSL_EMIF_SDRAMCONFIG0_NM_RESETVAL (0x0000u)
N
N#define CSL_EMIF_SDRAMCONFIG0_DDRDLL_ENABLE_MASK (0x2000u)
N#define CSL_EMIF_SDRAMCONFIG0_DDRDLL_ENABLE_SHIFT (0x000Du)
N#define CSL_EMIF_SDRAMCONFIG0_DDRDLL_ENABLE_RESETVAL (0x0000u)
N
N#define CSL_EMIF_SDRAMCONFIG0_DDRPLL_LOCK_MASK (0x1000u)
N#define CSL_EMIF_SDRAMCONFIG0_DDRPLL_LOCK_SHIFT (0x000Cu)
N#define CSL_EMIF_SDRAMCONFIG0_DDRPLL_LOCK_RESETVAL (0x0000u)
N
N#define CSL_EMIF_SDRAMCONFIG0_CASLATENCY_MASK (0x0E00u)
N#define CSL_EMIF_SDRAMCONFIG0_CASLATENCY_SHIFT (0x0009u)
N#define CSL_EMIF_SDRAMCONFIG0_CASLATENCY_RESETVAL (0x0000u)
N
N#define CSL_EMIF_SDRAMCONFIG0_CASLOCK_MASK (0x0100u)
N#define CSL_EMIF_SDRAMCONFIG0_CASLOCK_SHIFT (0x0008u)
N#define CSL_EMIF_SDRAMCONFIG0_CASLOCK_RESETVAL (0x0000u)
N
N
N#define CSL_EMIF_SDRAMCONFIG0_IBANK_MASK (0x0070u)
N#define CSL_EMIF_SDRAMCONFIG0_IBANK_SHIFT (0x0004u)
N#define CSL_EMIF_SDRAMCONFIG0_IBANK_RESETVAL (0x0002u)
N
N#define CSL_EMIF_SDRAMCONFIG0_EBANK_MASK (0x0008u)
N#define CSL_EMIF_SDRAMCONFIG0_EBANK_SHIFT (0x0003u)
N#define CSL_EMIF_SDRAMCONFIG0_EBANK_RESETVAL (0x0000u)
N
N#define CSL_EMIF_SDRAMCONFIG0_PAGESIZE_MASK (0x0007u)
N#define CSL_EMIF_SDRAMCONFIG0_PAGESIZE_SHIFT (0x0000u)
N#define CSL_EMIF_SDRAMCONFIG0_PAGESIZE_RESETVAL (0x0000u)
N
N#define CSL_EMIF_SDRAMCONFIG0_RESETVAL (0x0020u)
N
N/* SDRAMCONFIG1 */
N
N#define CSL_EMIF_SDRAMCONFIG1_SR_MASK (0x8000u)
N#define CSL_EMIF_SDRAMCONFIG1_SR_SHIFT (0x000Fu)
N#define CSL_EMIF_SDRAMCONFIG1_SR_RESETVAL (0x0000u)
N
N#define CSL_EMIF_SDRAMCONFIG1_PD_MASK (0x4000u)
N#define CSL_EMIF_SDRAMCONFIG1_PD_SHIFT (0x000Eu)
N#define CSL_EMIF_SDRAMCONFIG1_PD_RESETVAL (0x0000u)
N
N#define CSL_EMIF_SDRAMCONFIG1_PDWR_MASK (0x2000u)
N#define CSL_EMIF_SDRAMCONFIG1_PDWR_SHIFT (0x000Du)
N#define CSL_EMIF_SDRAMCONFIG1_PDWR_RESETVAL (0x0000u)
N
N
N#define CSL_EMIF_SDRAMCONFIG1_PASR_MASK (0x0380u)
N#define CSL_EMIF_SDRAMCONFIG1_PASR_SHIFT (0x0007u)
N#define CSL_EMIF_SDRAMCONFIG1_PASR_RESETVAL (0x0000u)
N
N#define CSL_EMIF_SDRAMCONFIG1_ROWSIZE_MASK (0x0070u)
N#define CSL_EMIF_SDRAMCONFIG1_ROWSIZE_SHIFT (0x0004u)
N#define CSL_EMIF_SDRAMCONFIG1_ROWSIZE_RESETVAL (0x0000u)
N
N#define CSL_EMIF_SDRAMCONFIG1_IBANK_POS_MASK (0x0008u)
N#define CSL_EMIF_SDRAMCONFIG1_IBANK_POS_SHIFT (0x0003u)
N#define CSL_EMIF_SDRAMCONFIG1_IBANK_POS_RESETVAL (0x0000u)
N
N#define CSL_EMIF_SDRAMCONFIG1_SDRAM_DRIVE_MASK (0x0006u)
N#define CSL_EMIF_SDRAMCONFIG1_SDRAM_DRIVE_SHIFT (0x0001u)
N#define CSL_EMIF_SDRAMCONFIG1_SDRAM_DRIVE_RESETVAL (0x0000u)
N
N#define CSL_EMIF_SDRAMCONFIG1_LOCK_17_25_MASK (0x0001u)
N#define CSL_EMIF_SDRAMCONFIG1_LOCK_17_25_SHIFT (0x0000u)
N#define CSL_EMIF_SDRAMCONFIG1_LOCK_17_25_RESETVAL (0x0000u)
N
N#define CSL_EMIF_SDRAMCONFIG1_RESETVAL (0x0000u)
N
N/* SDRCREFCTRL0 */
N
N
N#define CSL_EMIF_SDRCREFCTRL0_REFRATE_MASK (0x1FFFu)
N#define CSL_EMIF_SDRCREFCTRL0_REFRATE_SHIFT (0x0000u)
N#define CSL_EMIF_SDRCREFCTRL0_REFRATE_RESETVAL (0x0000u)
N
N#define CSL_EMIF_SDRCREFCTRL0_RESETVAL (0x0000u)
N
N/* SDRCREFCTRL1 */
N
N
N#define CSL_EMIF_SDRCREFCTRL1_DDR_REF_THR_MASK (0x0007u)
N#define CSL_EMIF_SDRCREFCTRL1_DDR_REF_THR_SHIFT (0x0000u)
N#define CSL_EMIF_SDRCREFCTRL1_DDR_REF_THR_RESETVAL (0x0000u)
N
N#define CSL_EMIF_SDRCREFCTRL1_RESETVAL (0x0000u)
N
N/* ASYNCCS0CTRL0 */
N
N#define CSL_EMIF_ASYNCCS0CTRL0_RSETUP_MASK (0xE000u)
N#define CSL_EMIF_ASYNCCS0CTRL0_RSETUP_SHIFT (0x000Du)
N#define CSL_EMIF_ASYNCCS0CTRL0_RSETUP_RESETVAL (0x0007u)
N
N#define CSL_EMIF_ASYNCCS0CTRL0_RSTROBE_MASK (0x1F80u)
N#define CSL_EMIF_ASYNCCS0CTRL0_RSTROBE_SHIFT (0x0007u)
N#define CSL_EMIF_ASYNCCS0CTRL0_RSTROBE_RESETVAL (0x003Fu)
N
N#define CSL_EMIF_ASYNCCS0CTRL0_RHOLD_MASK (0x0070u)
N#define CSL_EMIF_ASYNCCS0CTRL0_RHOLD_SHIFT (0x0004u)
N#define CSL_EMIF_ASYNCCS0CTRL0_RHOLD_RESETVAL (0x0007u)
N
N#define CSL_EMIF_ASYNCCS0CTRL0_TA_MASK (0x000Cu)
N#define CSL_EMIF_ASYNCCS0CTRL0_TA_SHIFT (0x0002u)
N#define CSL_EMIF_ASYNCCS0CTRL0_TA_RESETVAL (0x0003u)
N
N#define CSL_EMIF_ASYNCCS0CTRL0_BUSWIDTH_MASK (0x0003u)
N#define CSL_EMIF_ASYNCCS0CTRL0_BUSWIDTH_SHIFT (0x0000u)
N#define CSL_EMIF_ASYNCCS0CTRL0_BUSWIDTH_RESETVAL (0x0000u)
N
N#define CSL_EMIF_ASYNCCS0CTRL0_RESETVAL (0xFFFCu)
N
N/* ASYNCCS0CTRL1 */
N
N#define CSL_EMIF_ASYNCCS0CTRL1_SELSTROBE_MASK (0x8000u)
N#define CSL_EMIF_ASYNCCS0CTRL1_SELSTROBE_SHIFT (0x000Fu)
N#define CSL_EMIF_ASYNCCS0CTRL1_SELSTROBE_RESETVAL (0x0000u)
N
N#define CSL_EMIF_ASYNCCS0CTRL1_EXTWAIT_MASK (0x4000u)
N#define CSL_EMIF_ASYNCCS0CTRL1_EXTWAIT_SHIFT (0x000Eu)
N#define CSL_EMIF_ASYNCCS0CTRL1_EXTWAIT_RESETVAL (0x0000u)
N
N#define CSL_EMIF_ASYNCCS0CTRL1_WSETUP_MASK (0x3C00u)
N#define CSL_EMIF_ASYNCCS0CTRL1_WSETUP_SHIFT (0x000Au)
N#define CSL_EMIF_ASYNCCS0CTRL1_WSETUP_RESETVAL (0x000Fu)
N
N#define CSL_EMIF_ASYNCCS0CTRL1_WSTROBE_MASK (0x03F0u)
N#define CSL_EMIF_ASYNCCS0CTRL1_WSTROBE_SHIFT (0x0004u)
N#define CSL_EMIF_ASYNCCS0CTRL1_WSTROBE_RESETVAL (0x003Fu)
N
N#define CSL_EMIF_ASYNCCS0CTRL1_WHOLD_MASK (0x000Eu)
N#define CSL_EMIF_ASYNCCS0CTRL1_WHOLD_SHIFT (0x0001u)
N#define CSL_EMIF_ASYNCCS0CTRL1_WHOLD_RESETVAL (0x0007u)
N
N#define CSL_EMIF_ASYNCCS0CTRL1_RSETUP_MASK (0x0001u)
N#define CSL_EMIF_ASYNCCS0CTRL1_RSETUP_SHIFT (0x0000u)
N#define CSL_EMIF_ASYNCCS0CTRL1_RSETUP_RESETVAL (0x0001u)
N
N#define CSL_EMIF_ASYNCCS0CTRL1_RESETVAL (0x3FFFu)
N
N/* ASYNCCS1CTRL0 */
N
N#define CSL_EMIF_ASYNCCS1CTRL0_RSETUP_MASK (0xE000u)
N#define CSL_EMIF_ASYNCCS1CTRL0_RSETUP_SHIFT (0x000Du)
N#define CSL_EMIF_ASYNCCS1CTRL0_RSETUP_RESETVAL (0x0007u)
N
N#define CSL_EMIF_ASYNCCS1CTRL0_RSTROBE_MASK (0x1F80u)
N#define CSL_EMIF_ASYNCCS1CTRL0_RSTROBE_SHIFT (0x0007u)
N#define CSL_EMIF_ASYNCCS1CTRL0_RSTROBE_RESETVAL (0x003Fu)
N
N#define CSL_EMIF_ASYNCCS1CTRL0_RHOLD_MASK (0x0070u)
N#define CSL_EMIF_ASYNCCS1CTRL0_RHOLD_SHIFT (0x0004u)
N#define CSL_EMIF_ASYNCCS1CTRL0_RHOLD_RESETVAL (0x0007u)
N
N#define CSL_EMIF_ASYNCCS1CTRL0_TA_MASK (0x000Cu)
N#define CSL_EMIF_ASYNCCS1CTRL0_TA_SHIFT (0x0002u)
N#define CSL_EMIF_ASYNCCS1CTRL0_TA_RESETVAL (0x0003u)
N
N#define CSL_EMIF_ASYNCCS1CTRL0_BUSWIDTH_MASK (0x0003u)
N#define CSL_EMIF_ASYNCCS1CTRL0_BUSWIDTH_SHIFT (0x0000u)
N#define CSL_EMIF_ASYNCCS1CTRL0_BUSWIDTH_RESETVAL (0x0000u)
N
N#define CSL_EMIF_ASYNCCS1CTRL0_RESETVAL (0xFFFCu)
N
N/* ASYNCCS1CTRL1 */
N
N#define CSL_EMIF_ASYNCCS1CTRL1_SELSTROBE_MASK (0x8000u)
N#define CSL_EMIF_ASYNCCS1CTRL1_SELSTROBE_SHIFT (0x000Fu)
N#define CSL_EMIF_ASYNCCS1CTRL1_SELSTROBE_RESETVAL (0x0000u)
N
N#define CSL_EMIF_ASYNCCS1CTRL1_EXTWAIT_MASK (0x4000u)
N#define CSL_EMIF_ASYNCCS1CTRL1_EXTWAIT_SHIFT (0x000Eu)
N#define CSL_EMIF_ASYNCCS1CTRL1_EXTWAIT_RESETVAL (0x0000u)
N
N#define CSL_EMIF_ASYNCCS1CTRL1_WSETUP_MASK (0x3C00u)
N#define CSL_EMIF_ASYNCCS1CTRL1_WSETUP_SHIFT (0x000Au)
N#define CSL_EMIF_ASYNCCS1CTRL1_WSETUP_RESETVAL (0x000Fu)
N
N#define CSL_EMIF_ASYNCCS1CTRL1_WSTROBE_MASK (0x03F0u)
N#define CSL_EMIF_ASYNCCS1CTRL1_WSTROBE_SHIFT (0x0004u)
N#define CSL_EMIF_ASYNCCS1CTRL1_WSTROBE_RESETVAL (0x003Fu)
N
N#define CSL_EMIF_ASYNCCS1CTRL1_WHOLD_MASK (0x000Eu)
N#define CSL_EMIF_ASYNCCS1CTRL1_WHOLD_SHIFT (0x0001u)
N#define CSL_EMIF_ASYNCCS1CTRL1_WHOLD_RESETVAL (0x0007u)
N
N#define CSL_EMIF_ASYNCCS1CTRL1_RSETUP_MASK (0x0001u)
N#define CSL_EMIF_ASYNCCS1CTRL1_RSETUP_SHIFT (0x0000u)
N#define CSL_EMIF_ASYNCCS1CTRL1_RSETUP_RESETVAL (0x0001u)
N
N#define CSL_EMIF_ASYNCCS1CTRL1_RESETVAL (0x3FFFu)
N
N/* ASYNCCS2CTRL0 */
N
N#define CSL_EMIF_ASYNCCS2CTRL0_RSETUP_MASK (0xE000u)
N#define CSL_EMIF_ASYNCCS2CTRL0_RSETUP_SHIFT (0x000Du)
N#define CSL_EMIF_ASYNCCS2CTRL0_RSETUP_RESETVAL (0x0007u)
N
N#define CSL_EMIF_ASYNCCS2CTRL0_RSTROBE_MASK (0x1F80u)
N#define CSL_EMIF_ASYNCCS2CTRL0_RSTROBE_SHIFT (0x0007u)
N#define CSL_EMIF_ASYNCCS2CTRL0_RSTROBE_RESETVAL (0x003Fu)
N
N#define CSL_EMIF_ASYNCCS2CTRL0_RHOLD_MASK (0x0070u)
N#define CSL_EMIF_ASYNCCS2CTRL0_RHOLD_SHIFT (0x0004u)
N#define CSL_EMIF_ASYNCCS2CTRL0_RHOLD_RESETVAL (0x0007u)
N
N#define CSL_EMIF_ASYNCCS2CTRL0_TA_MASK (0x000Cu)
N#define CSL_EMIF_ASYNCCS2CTRL0_TA_SHIFT (0x0002u)
N#define CSL_EMIF_ASYNCCS2CTRL0_TA_RESETVAL (0x0003u)
N
N#define CSL_EMIF_ASYNCCS2CTRL0_BUSWIDTH_MASK (0x0003u)
N#define CSL_EMIF_ASYNCCS2CTRL0_BUSWIDTH_SHIFT (0x0000u)
N#define CSL_EMIF_ASYNCCS2CTRL0_BUSWIDTH_RESETVAL (0x0000u)
N
N#define CSL_EMIF_ASYNCCS2CTRL0_RESETVAL (0xFFFCu)
N
N/* ASYNCCS2CTRL1 */
N
N#define CSL_EMIF_ASYNCCS2CTRL1_SELSTROBE_MASK (0x8000u)
N#define CSL_EMIF_ASYNCCS2CTRL1_SELSTROBE_SHIFT (0x000Fu)
N#define CSL_EMIF_ASYNCCS2CTRL1_SELSTROBE_RESETVAL (0x0000u)
N
N#define CSL_EMIF_ASYNCCS2CTRL1_EXTWAIT_MASK (0x4000u)
N#define CSL_EMIF_ASYNCCS2CTRL1_EXTWAIT_SHIFT (0x000Eu)
N#define CSL_EMIF_ASYNCCS2CTRL1_EXTWAIT_RESETVAL (0x0000u)
N
N#define CSL_EMIF_ASYNCCS2CTRL1_WSETUP_MASK (0x3C00u)
N#define CSL_EMIF_ASYNCCS2CTRL1_WSETUP_SHIFT (0x000Au)
N#define CSL_EMIF_ASYNCCS2CTRL1_WSETUP_RESETVAL (0x000Fu)
N
N#define CSL_EMIF_ASYNCCS2CTRL1_WSTROBE_MASK (0x03F0u)
N#define CSL_EMIF_ASYNCCS2CTRL1_WSTROBE_SHIFT (0x0004u)
N#define CSL_EMIF_ASYNCCS2CTRL1_WSTROBE_RESETVAL (0x003Fu)
N
N#define CSL_EMIF_ASYNCCS2CTRL1_WHOLD_MASK (0x000Eu)
N#define CSL_EMIF_ASYNCCS2CTRL1_WHOLD_SHIFT (0x0001u)
N#define CSL_EMIF_ASYNCCS2CTRL1_WHOLD_RESETVAL (0x0007u)
N
N#define CSL_EMIF_ASYNCCS2CTRL1_RSETUP_MASK (0x0001u)
N#define CSL_EMIF_ASYNCCS2CTRL1_RSETUP_SHIFT (0x0000u)
N#define CSL_EMIF_ASYNCCS2CTRL1_RSETUP_RESETVAL (0x0001u)
N
N#define CSL_EMIF_ASYNCCS2CTRL1_RESETVAL (0x3FFFu)
N
N/* ASYNCCS3CTRL0 */
N
N#define CSL_EMIF_ASYNCCS3CTRL0_RSETUP_MASK (0xE000u)
N#define CSL_EMIF_ASYNCCS3CTRL0_RSETUP_SHIFT (0x000Du)
N#define CSL_EMIF_ASYNCCS3CTRL0_RSETUP_RESETVAL (0x0007u)
N
N#define CSL_EMIF_ASYNCCS3CTRL0_RSTROBE_MASK (0x1F80u)
N#define CSL_EMIF_ASYNCCS3CTRL0_RSTROBE_SHIFT (0x0007u)
N#define CSL_EMIF_ASYNCCS3CTRL0_RSTROBE_RESETVAL (0x003Fu)
N
N#define CSL_EMIF_ASYNCCS3CTRL0_RHOLD_MASK (0x0070u)
N#define CSL_EMIF_ASYNCCS3CTRL0_RHOLD_SHIFT (0x0004u)
N#define CSL_EMIF_ASYNCCS3CTRL0_RHOLD_RESETVAL (0x0007u)
N
N#define CSL_EMIF_ASYNCCS3CTRL0_TA_MASK (0x000Cu)
N#define CSL_EMIF_ASYNCCS3CTRL0_TA_SHIFT (0x0002u)
N#define CSL_EMIF_ASYNCCS3CTRL0_TA_RESETVAL (0x0003u)
N
N#define CSL_EMIF_ASYNCCS3CTRL0_BUSWIDTH_MASK (0x0003u)
N#define CSL_EMIF_ASYNCCS3CTRL0_BUSWIDTH_SHIFT (0x0000u)
N#define CSL_EMIF_ASYNCCS3CTRL0_BUSWIDTH_RESETVAL (0x0000u)
N
N#define CSL_EMIF_ASYNCCS3CTRL0_RESETVAL (0xFFFCu)
N
N/* ASYNCCS3CTRL1 */
N
N#define CSL_EMIF_ASYNCCS3CTRL1_SELSTROBE_MASK (0x8000u)
N#define CSL_EMIF_ASYNCCS3CTRL1_SELSTROBE_SHIFT (0x000Fu)
N#define CSL_EMIF_ASYNCCS3CTRL1_SELSTROBE_RESETVAL (0x0000u)
N
N#define CSL_EMIF_ASYNCCS3CTRL1_EXTWAIT_MASK (0x4000u)
N#define CSL_EMIF_ASYNCCS3CTRL1_EXTWAIT_SHIFT (0x000Eu)
N#define CSL_EMIF_ASYNCCS3CTRL1_EXTWAIT_RESETVAL (0x0000u)
N
N#define CSL_EMIF_ASYNCCS3CTRL1_WSETUP_MASK (0x3C00u)
N#define CSL_EMIF_ASYNCCS3CTRL1_WSETUP_SHIFT (0x000Au)
N#define CSL_EMIF_ASYNCCS3CTRL1_WSETUP_RESETVAL (0x000Fu)
N
N#define CSL_EMIF_ASYNCCS3CTRL1_WSTROBE_MASK (0x03F0u)
N#define CSL_EMIF_ASYNCCS3CTRL1_WSTROBE_SHIFT (0x0004u)
N#define CSL_EMIF_ASYNCCS3CTRL1_WSTROBE_RESETVAL (0x003Fu)
N
N#define CSL_EMIF_ASYNCCS3CTRL1_WHOLD_MASK (0x000Eu)
N#define CSL_EMIF_ASYNCCS3CTRL1_WHOLD_SHIFT (0x0001u)
N#define CSL_EMIF_ASYNCCS3CTRL1_WHOLD_RESETVAL (0x0007u)
N
N#define CSL_EMIF_ASYNCCS3CTRL1_RSETUP_MASK (0x0001u)
N#define CSL_EMIF_ASYNCCS3CTRL1_RSETUP_SHIFT (0x0000u)
N#define CSL_EMIF_ASYNCCS3CTRL1_RSETUP_RESETVAL (0x0001u)
N
N#define CSL_EMIF_ASYNCCS3CTRL1_RESETVAL (0x3FFFu)
N
N/* SDRAMTIMING0 */
N
N#define CSL_EMIF_SDRAMTIMING0_TRAS_MASK (0xF000u)
N#define CSL_EMIF_SDRAMTIMING0_TRAS_SHIFT (0x000Cu)
N#define CSL_EMIF_SDRAMTIMING0_TRAS_RESETVAL (0x0000u)
N
N#define CSL_EMIF_SDRAMTIMING0_TRC_MASK (0x0F00u)
N#define CSL_EMIF_SDRAMTIMING0_TRC_SHIFT (0x0008u)
N#define CSL_EMIF_SDRAMTIMING0_TRC_RESETVAL (0x0000u)
N
N
N#define CSL_EMIF_SDRAMTIMING0_TRRD_MASK (0x0070u)
N#define CSL_EMIF_SDRAMTIMING0_TRRD_SHIFT (0x0004u)
N#define CSL_EMIF_SDRAMTIMING0_TRRD_RESETVAL (0x0000u)
N
N
N#define CSL_EMIF_SDRAMTIMING0_RESETVAL (0x0000u)
N
N/* SDRAMTIMING1 */
N
N#define CSL_EMIF_SDRAMTIMING1_TRFC_MASK (0xF800u)
N#define CSL_EMIF_SDRAMTIMING1_TRFC_SHIFT (0x000Bu)
N#define CSL_EMIF_SDRAMTIMING1_TRFC_RESETVAL (0x0000u)
N
N#define CSL_EMIF_SDRAMTIMING1_TRP_MASK (0x0700u)
N#define CSL_EMIF_SDRAMTIMING1_TRP_SHIFT (0x0008u)
N#define CSL_EMIF_SDRAMTIMING1_TRP_RESETVAL (0x0000u)
N
N
N#define CSL_EMIF_SDRAMTIMING1_TRCD_MASK (0x0070u)
N#define CSL_EMIF_SDRAMTIMING1_TRCD_SHIFT (0x0004u)
N#define CSL_EMIF_SDRAMTIMING1_TRCD_RESETVAL (0x0000u)
N
N
N#define CSL_EMIF_SDRAMTIMING1_TWR_MASK (0x0007u)
N#define CSL_EMIF_SDRAMTIMING1_TWR_SHIFT (0x0000u)
N#define CSL_EMIF_SDRAMTIMING1_TWR_RESETVAL (0x0000u)
N
N#define CSL_EMIF_SDRAMTIMING1_RESETVAL (0x0000u)
N
N/* SDRAMSTAT */
N
N
N#define CSL_EMIF_SDRAMSTAT_PHYRDY_MASK (0x0008u)
N#define CSL_EMIF_SDRAMSTAT_PHYRDY_SHIFT (0x0003u)
N#define CSL_EMIF_SDRAMSTAT_PHYRDY_RESETVAL (0x0000u)
N
N
N#define CSL_EMIF_SDRAMSTAT_MSDRMODE_MASK (0x0002u)
N#define CSL_EMIF_SDRAMSTAT_MSDRMODE_SHIFT (0x0001u)
N#define CSL_EMIF_SDRAMSTAT_MSDRMODE_RESETVAL (0x0000u)
N
N#define CSL_EMIF_SDRAMSTAT_DOUBLERATE_MASK (0x0001u)
N#define CSL_EMIF_SDRAMSTAT_DOUBLERATE_SHIFT (0x0000u)
N#define CSL_EMIF_SDRAMSTAT_DOUBLERATE_RESETVAL (0x0000u)
N
N#define CSL_EMIF_SDRAMSTAT_RESETVAL (0x0000u)
N
N/* SDRACCESS0 */
N
N#define CSL_EMIF_SDRACCESS0_TOTACCESS_0_15_MASK (0xFFFFu)
N#define CSL_EMIF_SDRACCESS0_TOTACCESS_0_15_SHIFT (0x0000u)
N#define CSL_EMIF_SDRACCESS0_TOTACCESS_0_15_RESETVAL (0x0000u)
N
N#define CSL_EMIF_SDRACCESS0_RESETVAL (0x0000u)
N
N/* SDRACCESS1 */
N
N#define CSL_EMIF_SDRACCESS1_TOTACCESS_16_31_MASK (0xFFFFu)
N#define CSL_EMIF_SDRACCESS1_TOTACCESS_16_31_SHIFT (0x0000u)
N#define CSL_EMIF_SDRACCESS1_TOTACCESS_16_31_RESETVAL (0x0000u)
N
N#define CSL_EMIF_SDRACCESS1_RESETVAL (0x0000u)
N
N/* SDRACTIVATE0 */
N
N#define CSL_EMIF_SDRACTIVATE0_TOTACCESS_0_15_MASK (0xFFFFu)
N#define CSL_EMIF_SDRACTIVATE0_TOTACCESS_0_15_SHIFT (0x0000u)
N#define CSL_EMIF_SDRACTIVATE0_TOTACCESS_0_15_RESETVAL (0x0000u)
N
N#define CSL_EMIF_SDRACTIVATE0_RESETVAL (0x0000u)
N
N/* SDRACTIVATE1 */
N
N#define CSL_EMIF_SDRACTIVATE1_TOTACCESS_16_31_MASK (0xFFFFu)
N#define CSL_EMIF_SDRACTIVATE1_TOTACCESS_16_31_SHIFT (0x0000u)
N#define CSL_EMIF_SDRACTIVATE1_TOTACCESS_16_31_RESETVAL (0x0000u)
N
N#define CSL_EMIF_SDRACTIVATE1_RESETVAL (0x0000u)
N
N/* SDRCSRPD */
N
N
N
N#define CSL_EMIF_SDRCSRPD_TXS_MASK (0x001Fu)
N#define CSL_EMIF_SDRCSRPD_TXS_SHIFT (0x0000u)
N#define CSL_EMIF_SDRCSRPD_TXS_RESETVAL (0x0000u)
N
N#define CSL_EMIF_SDRCSRPD_RESETVAL (0x0000u)
N
N/* INTRAW */
N
N
N#define CSL_EMIF_INTRAW_WAITRISE_MASK (0x007Cu)
N#define CSL_EMIF_INTRAW_WAITRISE_SHIFT (0x0002u)
N#define CSL_EMIF_INTRAW_WAITRISE_RESETVAL (0x0000u)
N
N#define CSL_EMIF_INTRAW_LINETRAP_MASK (0x0002u)
N#define CSL_EMIF_INTRAW_LINETRAP_SHIFT (0x0001u)
N#define CSL_EMIF_INTRAW_LINETRAP_RESETVAL (0x0000u)
N
N#define CSL_EMIF_INTRAW_ASYNCTOUT_MASK (0x0001u)
N#define CSL_EMIF_INTRAW_ASYNCTOUT_SHIFT (0x0000u)
N#define CSL_EMIF_INTRAW_ASYNCTOUT_RESETVAL (0x0000u)
N
N#define CSL_EMIF_INTRAW_RESETVAL (0x0000u)
N
N/* INTMASK */
N
N
N#define CSL_EMIF_INTMASK_WRMASK_MASK (0x003Cu)
N#define CSL_EMIF_INTMASK_WRMASK_SHIFT (0x0002u)
N#define CSL_EMIF_INTMASK_WRMASK_RESETVAL (0x0000u)
N
N#define CSL_EMIF_INTMASK_LTMASK_MASK (0x0002u)
N#define CSL_EMIF_INTMASK_LTMASK_SHIFT (0x0001u)
N#define CSL_EMIF_INTMASK_LTMASK_RESETVAL (0x0000u)
N
N#define CSL_EMIF_INTMASK_ATMASK_MASK (0x0001u)
N#define CSL_EMIF_INTMASK_ATMASK_SHIFT (0x0000u)
N#define CSL_EMIF_INTMASK_ATMASK_RESETVAL (0x0000u)
N
N#define CSL_EMIF_INTMASK_RESETVAL (0x0000u)
N
N/* INTMASKSET */
N
N
N#define CSL_EMIF_INTMASKSET_WRMASKSET_MASK (0x003Cu)
N#define CSL_EMIF_INTMASKSET_WRMASKSET_SHIFT (0x0002u)
N#define CSL_EMIF_INTMASKSET_WRMASKSET_RESETVAL (0x0000u)
N
N#define CSL_EMIF_INTMASKSET_LTMASKSET_MASK (0x0002u)
N#define CSL_EMIF_INTMASKSET_LTMASKSET_SHIFT (0x0001u)
N#define CSL_EMIF_INTMASKSET_LTMASKSET_RESETVAL (0x0000u)
N
N#define CSL_EMIF_INTMASKSET_ATMASKSET_MASK (0x0001u)
N#define CSL_EMIF_INTMASKSET_ATMASKSET_SHIFT (0x0000u)
N#define CSL_EMIF_INTMASKSET_ATMASKSET_RESETVAL (0x0000u)
N
N#define CSL_EMIF_INTMASKSET_RESETVAL (0x0000u)
N
N/* INTMASKCLEAR */
N
N
N#define CSL_EMIF_INTMASKCLEAR_WRMASKSET_MASK (0x003Cu)
N#define CSL_EMIF_INTMASKCLEAR_WRMASKSET_SHIFT (0x0002u)
N#define CSL_EMIF_INTMASKCLEAR_WRMASKSET_RESETVAL (0x0000u)
N
N#define CSL_EMIF_INTMASKCLEAR_LTMASKSET_MASK (0x0002u)
N#define CSL_EMIF_INTMASKCLEAR_LTMASKSET_SHIFT (0x0001u)
N#define CSL_EMIF_INTMASKCLEAR_LTMASKSET_RESETVAL (0x0000u)
N
N#define CSL_EMIF_INTMASKCLEAR_ATMASKSET_MASK (0x0001u)
N#define CSL_EMIF_INTMASKCLEAR_ATMASKSET_SHIFT (0x0000u)
N#define CSL_EMIF_INTMASKCLEAR_ATMASKSET_RESETVAL (0x0000u)
N
N#define CSL_EMIF_INTMASKCLEAR_RESETVAL (0x0000u)
N
N/* IOCTRL */
N
N
N#define CSL_EMIF_IOCTRL_IOCTRL_MASK (0xFFFFu)
N#define CSL_EMIF_IOCTRL_IOCTRL_SHIFT (0x0000u)
N#define CSL_EMIF_IOCTRL_IOCTRL_RESETVAL (0x0000u)
N
N#define CSL_EMIF_IOCTRL_RESETVAL (0x0000u)
N
N/* IOSTATUS */
N
N
N#define CSL_EMIF_IOSTATUS_IOSTAT_MASK (0xFFFFu)
N#define CSL_EMIF_IOSTATUS_IOSTAT_SHIFT (0x0000u)
N#define CSL_EMIF_IOSTATUS_IOSTAT_RESETVAL (0x0000u)
N
N#define CSL_EMIF_IOSTATUS_RESETVAL (0x0000u)
N
N/* NANDCTRL */
N
N
N#define CSL_EMIF_NANDCTRL_ADDCAL_MASK (0x2000u)
N#define CSL_EMIF_NANDCTRL_ADDCAL_SHIFT (0x000Du)
N#define CSL_EMIF_NANDCTRL_ADDCAL_RESETVAL (0x0000u)
N
N#define CSL_EMIF_NANDCTRL_4BITECCSTART_MASK (0x1000u)
N#define CSL_EMIF_NANDCTRL_4BITECCSTART_SHIFT (0x000Cu)
N#define CSL_EMIF_NANDCTRL_4BITECCSTART_RESETVAL (0x0000u)
N
N#define CSL_EMIF_NANDCTRL_CS5ECCSTART_MASK (0x0800u)
N#define CSL_EMIF_NANDCTRL_CS5ECCSTART_SHIFT (0x000Bu)
N#define CSL_EMIF_NANDCTRL_CS5ECCSTART_RESETVAL (0x0000u)
N
N#define CSL_EMIF_NANDCTRL_CS4ECCSTART_MASK (0x0400u)
N#define CSL_EMIF_NANDCTRL_CS4ECCSTART_SHIFT (0x000Au)
N#define CSL_EMIF_NANDCTRL_CS4ECCSTART_RESETVAL (0x0000u)
N
N#define CSL_EMIF_NANDCTRL_CS3ECCSTART_MASK (0x0200u)
N#define CSL_EMIF_NANDCTRL_CS3ECCSTART_SHIFT (0x0009u)
N#define CSL_EMIF_NANDCTRL_CS3ECCSTART_RESETVAL (0x0000u)
N
N#define CSL_EMIF_NANDCTRL_CS2ECCSTART_MASK (0x0100u)
N#define CSL_EMIF_NANDCTRL_CS2ECCSTART_SHIFT (0x0008u)
N#define CSL_EMIF_NANDCTRL_CS2ECCSTART_RESETVAL (0x0000u)
N
N
N#define CSL_EMIF_NANDCTRL_ECCSEL_MASK (0x0030u)
N#define CSL_EMIF_NANDCTRL_ECCSEL_SHIFT (0x0004u)
N#define CSL_EMIF_NANDCTRL_ECCSEL_RESETVAL (0x0000u)
N
N#define CSL_EMIF_NANDCTRL_CS5SEL_MASK (0x0008u)
N#define CSL_EMIF_NANDCTRL_CS5SEL_SHIFT (0x0003u)
N#define CSL_EMIF_NANDCTRL_CS5SEL_RESETVAL (0x0000u)
N
N#define CSL_EMIF_NANDCTRL_CS4SEL_MASK (0x0004u)
N#define CSL_EMIF_NANDCTRL_CS4SEL_SHIFT (0x0002u)
N#define CSL_EMIF_NANDCTRL_CS4SEL_RESETVAL (0x0000u)
N
N#define CSL_EMIF_NANDCTRL_CS3SEL_MASK (0x0002u)
N#define CSL_EMIF_NANDCTRL_CS3SEL_SHIFT (0x0001u)
N#define CSL_EMIF_NANDCTRL_CS3SEL_RESETVAL (0x0000u)
N
N#define CSL_EMIF_NANDCTRL_CS2SEL_MASK (0x0001u)
N#define CSL_EMIF_NANDCTRL_CS2SEL_SHIFT (0x0000u)
N#define CSL_EMIF_NANDCTRL_CS2SEL_RESETVAL (0x0000u)
N
N#define CSL_EMIF_NANDCTRL_RESETVAL (0x0000u)
N
N/* NANDSTAT0 */
N
N
N#define CSL_EMIF_NANDSTAT0_CORRSTATE_MASK (0x0F00u)
N#define CSL_EMIF_NANDSTAT0_CORRSTATE_SHIFT (0x0008u)
N#define CSL_EMIF_NANDSTAT0_CORRSTATE_RESETVAL (0x0000u)
N
N
N#define CSL_EMIF_NANDSTAT0_WAITSTAT_MASK (0x000Fu)
N#define CSL_EMIF_NANDSTAT0_WAITSTAT_SHIFT (0x0000u)
N#define CSL_EMIF_NANDSTAT0_WAITSTAT_RESETVAL (0x0000u)
N
N#define CSL_EMIF_NANDSTAT0_RESETVAL (0x0000u)
N
N/* NANDSTAT1 */
N
N
N#define CSL_EMIF_NANDSTAT1_ERRNUM_MASK (0x0003u)
N#define CSL_EMIF_NANDSTAT1_ERRNUM_SHIFT (0x0000u)
N#define CSL_EMIF_NANDSTAT1_ERRNUM_RESETVAL (0x0000u)
N
N#define CSL_EMIF_NANDSTAT1_RESETVAL (0x0000u)
N
N/* PAGEMODCTRL0 */
N
N#define CSL_EMIF_PAGEMODCTRL0_CS3_PAGE_DELAY_MASK (0xFC00u)
N#define CSL_EMIF_PAGEMODCTRL0_CS3_PAGE_DELAY_SHIFT (0x000Au)
N#define CSL_EMIF_PAGEMODCTRL0_CS3_PAGE_DELAY_RESETVAL (0x003Fu)
N
N#define CSL_EMIF_PAGEMODCTRL0_CS3_PAGE_SIZE_MASK (0x0200u)
N#define CSL_EMIF_PAGEMODCTRL0_CS3_PAGE_SIZE_SHIFT (0x0009u)
N#define CSL_EMIF_PAGEMODCTRL0_CS3_PAGE_SIZE_RESETVAL (0x0000u)
N
N#define CSL_EMIF_PAGEMODCTRL0_CS3_PAGEMOD_EN_MASK (0x0100u)
N#define CSL_EMIF_PAGEMODCTRL0_CS3_PAGEMOD_EN_SHIFT (0x0008u)
N#define CSL_EMIF_PAGEMODCTRL0_CS3_PAGEMOD_EN_RESETVAL (0x0000u)
N
N#define CSL_EMIF_PAGEMODCTRL0_CS2_PAGE_DELAY_MASK (0x00FCu)
N#define CSL_EMIF_PAGEMODCTRL0_CS2_PAGE_DELAY_SHIFT (0x0002u)
N#define CSL_EMIF_PAGEMODCTRL0_CS2_PAGE_DELAY_RESETVAL (0x003Fu)
N
N#define CSL_EMIF_PAGEMODCTRL0_CS2_PAGE_SIZE_MASK (0x0002u)
N#define CSL_EMIF_PAGEMODCTRL0_CS2_PAGE_SIZE_SHIFT (0x0001u)
N#define CSL_EMIF_PAGEMODCTRL0_CS2_PAGE_SIZE_RESETVAL (0x0000u)
N
N#define CSL_EMIF_PAGEMODCTRL0_CS2_PAGEMOD_EN_MASK (0x0001u)
N#define CSL_EMIF_PAGEMODCTRL0_CS2_PAGEMOD_EN_SHIFT (0x0000u)
N#define CSL_EMIF_PAGEMODCTRL0_CS2_PAGEMOD_EN_RESETVAL (0x0000u)
N
N#define CSL_EMIF_PAGEMODCTRL0_RESETVAL (0xFCFCu)
N
N/* PAGEMODCTRL1 */
N
N#define CSL_EMIF_PAGEMODCTRL1_CS5_PAGE_DELAY_MASK (0xFC00u)
N#define CSL_EMIF_PAGEMODCTRL1_CS5_PAGE_DELAY_SHIFT (0x000Au)
N#define CSL_EMIF_PAGEMODCTRL1_CS5_PAGE_DELAY_RESETVAL (0x003Fu)
N
N#define CSL_EMIF_PAGEMODCTRL1_CS5_PAGE_SIZE_MASK (0x0200u)
N#define CSL_EMIF_PAGEMODCTRL1_CS5_PAGE_SIZE_SHIFT (0x0009u)
N#define CSL_EMIF_PAGEMODCTRL1_CS5_PAGE_SIZE_RESETVAL (0x0000u)
N
N#define CSL_EMIF_PAGEMODCTRL1_CS5_PAGEMOD_EN_MASK (0x0100u)
N#define CSL_EMIF_PAGEMODCTRL1_CS5_PAGEMOD_EN_SHIFT (0x0008u)
N#define CSL_EMIF_PAGEMODCTRL1_CS5_PAGEMOD_EN_RESETVAL (0x0000u)
N
N#define CSL_EMIF_PAGEMODCTRL1_CS4_PAGE_DELAY_MASK (0x00FCu)
N#define CSL_EMIF_PAGEMODCTRL1_CS4_PAGE_DELAY_SHIFT (0x0002u)
N#define CSL_EMIF_PAGEMODCTRL1_CS4_PAGE_DELAY_RESETVAL (0x003Fu)
N
N#define CSL_EMIF_PAGEMODCTRL1_CS4_PAGE_SIZE_MASK (0x0002u)
N#define CSL_EMIF_PAGEMODCTRL1_CS4_PAGE_SIZE_SHIFT (0x0001u)
N#define CSL_EMIF_PAGEMODCTRL1_CS4_PAGE_SIZE_RESETVAL (0x0000u)
N
N#define CSL_EMIF_PAGEMODCTRL1_CS4_PAGEMOD_EN_MASK (0x0001u)
N#define CSL_EMIF_PAGEMODCTRL1_CS4_PAGEMOD_EN_SHIFT (0x0000u)
N#define CSL_EMIF_PAGEMODCTRL1_CS4_PAGEMOD_EN_RESETVAL (0x0000u)
N
N#define CSL_EMIF_PAGEMODCTRL1_RESETVAL (0xFCFCu)
N
N/* CS2ECC0 */
N
N
N
N#define CSL_EMIF_CS2ECC0_P2048E_MASK (0x0800u)
N#define CSL_EMIF_CS2ECC0_P2048E_SHIFT (0x000Bu)
N#define CSL_EMIF_CS2ECC0_P2048E_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS2ECC0_P1024E_MASK (0x0400u)
N#define CSL_EMIF_CS2ECC0_P1024E_SHIFT (0x000Au)
N#define CSL_EMIF_CS2ECC0_P1024E_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS2ECC0_P512E_MASK (0x0200u)
N#define CSL_EMIF_CS2ECC0_P512E_SHIFT (0x0009u)
N#define CSL_EMIF_CS2ECC0_P512E_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS2ECC0_P256E_MASK (0x0100u)
N#define CSL_EMIF_CS2ECC0_P256E_SHIFT (0x0008u)
N#define CSL_EMIF_CS2ECC0_P256E_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS2ECC0_P128E_MASK (0x0080u)
N#define CSL_EMIF_CS2ECC0_P128E_SHIFT (0x0007u)
N#define CSL_EMIF_CS2ECC0_P128E_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS2ECC0_P64E_MASK (0x0040u)
N#define CSL_EMIF_CS2ECC0_P64E_SHIFT (0x0006u)
N#define CSL_EMIF_CS2ECC0_P64E_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS2ECC0_P32E_MASK (0x0020u)
N#define CSL_EMIF_CS2ECC0_P32E_SHIFT (0x0005u)
N#define CSL_EMIF_CS2ECC0_P32E_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS2ECC0_P16E_MASK (0x0010u)
N#define CSL_EMIF_CS2ECC0_P16E_SHIFT (0x0004u)
N#define CSL_EMIF_CS2ECC0_P16E_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS2ECC0_P8E_MASK (0x0008u)
N#define CSL_EMIF_CS2ECC0_P8E_SHIFT (0x0003u)
N#define CSL_EMIF_CS2ECC0_P8E_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS2ECC0_P4E_MASK (0x0004u)
N#define CSL_EMIF_CS2ECC0_P4E_SHIFT (0x0002u)
N#define CSL_EMIF_CS2ECC0_P4E_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS2ECC0_P2E_MASK (0x0002u)
N#define CSL_EMIF_CS2ECC0_P2E_SHIFT (0x0001u)
N#define CSL_EMIF_CS2ECC0_P2E_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS2ECC0_P1E_MASK (0x0001u)
N#define CSL_EMIF_CS2ECC0_P1E_SHIFT (0x0000u)
N#define CSL_EMIF_CS2ECC0_P1E_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS2ECC0_RESETVAL (0x0000u)
N
N/* CS2ECC1 */
N
N
N#define CSL_EMIF_CS2ECC1_P2048O_MASK (0x0800u)
N#define CSL_EMIF_CS2ECC1_P2048O_SHIFT (0x000Bu)
N#define CSL_EMIF_CS2ECC1_P2048O_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS2ECC1_P1024O_MASK (0x0400u)
N#define CSL_EMIF_CS2ECC1_P1024O_SHIFT (0x000Au)
N#define CSL_EMIF_CS2ECC1_P1024O_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS2ECC1_P512O_MASK (0x0200u)
N#define CSL_EMIF_CS2ECC1_P512O_SHIFT (0x0009u)
N#define CSL_EMIF_CS2ECC1_P512O_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS2ECC1_P256O_MASK (0x0100u)
N#define CSL_EMIF_CS2ECC1_P256O_SHIFT (0x0008u)
N#define CSL_EMIF_CS2ECC1_P256O_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS2ECC1_P128O_MASK (0x0080u)
N#define CSL_EMIF_CS2ECC1_P128O_SHIFT (0x0007u)
N#define CSL_EMIF_CS2ECC1_P128O_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS2ECC1_P64O_MASK (0x0040u)
N#define CSL_EMIF_CS2ECC1_P64O_SHIFT (0x0006u)
N#define CSL_EMIF_CS2ECC1_P64O_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS2ECC1_P32O_MASK (0x0020u)
N#define CSL_EMIF_CS2ECC1_P32O_SHIFT (0x0005u)
N#define CSL_EMIF_CS2ECC1_P32O_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS2ECC1_P16O_MASK (0x0010u)
N#define CSL_EMIF_CS2ECC1_P16O_SHIFT (0x0004u)
N#define CSL_EMIF_CS2ECC1_P16O_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS2ECC1_P8O_MASK (0x0008u)
N#define CSL_EMIF_CS2ECC1_P8O_SHIFT (0x0003u)
N#define CSL_EMIF_CS2ECC1_P8O_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS2ECC1_P4O_MASK (0x0004u)
N#define CSL_EMIF_CS2ECC1_P4O_SHIFT (0x0002u)
N#define CSL_EMIF_CS2ECC1_P4O_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS2ECC1_P2O_MASK (0x0002u)
N#define CSL_EMIF_CS2ECC1_P2O_SHIFT (0x0001u)
N#define CSL_EMIF_CS2ECC1_P2O_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS2ECC1_P1O_MASK (0x0001u)
N#define CSL_EMIF_CS2ECC1_P1O_SHIFT (0x0000u)
N#define CSL_EMIF_CS2ECC1_P1O_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS2ECC1_RESETVAL (0x0000u)
N
N/* CS3ECC0 */
N
N
N
N#define CSL_EMIF_CS3ECC0_P2048E_MASK (0x0800u)
N#define CSL_EMIF_CS3ECC0_P2048E_SHIFT (0x000Bu)
N#define CSL_EMIF_CS3ECC0_P2048E_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS3ECC0_P1024E_MASK (0x0400u)
N#define CSL_EMIF_CS3ECC0_P1024E_SHIFT (0x000Au)
N#define CSL_EMIF_CS3ECC0_P1024E_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS3ECC0_P512E_MASK (0x0200u)
N#define CSL_EMIF_CS3ECC0_P512E_SHIFT (0x0009u)
N#define CSL_EMIF_CS3ECC0_P512E_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS3ECC0_P256E_MASK (0x0100u)
N#define CSL_EMIF_CS3ECC0_P256E_SHIFT (0x0008u)
N#define CSL_EMIF_CS3ECC0_P256E_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS3ECC0_P128E_MASK (0x0080u)
N#define CSL_EMIF_CS3ECC0_P128E_SHIFT (0x0007u)
N#define CSL_EMIF_CS3ECC0_P128E_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS3ECC0_P64E_MASK (0x0040u)
N#define CSL_EMIF_CS3ECC0_P64E_SHIFT (0x0006u)
N#define CSL_EMIF_CS3ECC0_P64E_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS3ECC0_P32E_MASK (0x0020u)
N#define CSL_EMIF_CS3ECC0_P32E_SHIFT (0x0005u)
N#define CSL_EMIF_CS3ECC0_P32E_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS3ECC0_P16E_MASK (0x0010u)
N#define CSL_EMIF_CS3ECC0_P16E_SHIFT (0x0004u)
N#define CSL_EMIF_CS3ECC0_P16E_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS3ECC0_P8E_MASK (0x0008u)
N#define CSL_EMIF_CS3ECC0_P8E_SHIFT (0x0003u)
N#define CSL_EMIF_CS3ECC0_P8E_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS3ECC0_P4E_MASK (0x0004u)
N#define CSL_EMIF_CS3ECC0_P4E_SHIFT (0x0002u)
N#define CSL_EMIF_CS3ECC0_P4E_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS3ECC0_P2E_MASK (0x0002u)
N#define CSL_EMIF_CS3ECC0_P2E_SHIFT (0x0001u)
N#define CSL_EMIF_CS3ECC0_P2E_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS3ECC0_P1E_MASK (0x0001u)
N#define CSL_EMIF_CS3ECC0_P1E_SHIFT (0x0000u)
N#define CSL_EMIF_CS3ECC0_P1E_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS3ECC0_RESETVAL (0x0000u)
N
N/* CS3ECC1 */
N
N
N#define CSL_EMIF_CS3ECC1_P2048O_MASK (0x0800u)
N#define CSL_EMIF_CS3ECC1_P2048O_SHIFT (0x000Bu)
N#define CSL_EMIF_CS3ECC1_P2048O_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS3ECC1_P1024O_MASK (0x0400u)
N#define CSL_EMIF_CS3ECC1_P1024O_SHIFT (0x000Au)
N#define CSL_EMIF_CS3ECC1_P1024O_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS3ECC1_P512O_MASK (0x0200u)
N#define CSL_EMIF_CS3ECC1_P512O_SHIFT (0x0009u)
N#define CSL_EMIF_CS3ECC1_P512O_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS3ECC1_P256O_MASK (0x0100u)
N#define CSL_EMIF_CS3ECC1_P256O_SHIFT (0x0008u)
N#define CSL_EMIF_CS3ECC1_P256O_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS3ECC1_P128O_MASK (0x0080u)
N#define CSL_EMIF_CS3ECC1_P128O_SHIFT (0x0007u)
N#define CSL_EMIF_CS3ECC1_P128O_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS3ECC1_P64O_MASK (0x0040u)
N#define CSL_EMIF_CS3ECC1_P64O_SHIFT (0x0006u)
N#define CSL_EMIF_CS3ECC1_P64O_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS3ECC1_P32O_MASK (0x0020u)
N#define CSL_EMIF_CS3ECC1_P32O_SHIFT (0x0005u)
N#define CSL_EMIF_CS3ECC1_P32O_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS3ECC1_P16O_MASK (0x0010u)
N#define CSL_EMIF_CS3ECC1_P16O_SHIFT (0x0004u)
N#define CSL_EMIF_CS3ECC1_P16O_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS3ECC1_P8O_MASK (0x0008u)
N#define CSL_EMIF_CS3ECC1_P8O_SHIFT (0x0003u)
N#define CSL_EMIF_CS3ECC1_P8O_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS3ECC1_P4O_MASK (0x0004u)
N#define CSL_EMIF_CS3ECC1_P4O_SHIFT (0x0002u)
N#define CSL_EMIF_CS3ECC1_P4O_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS3ECC1_P2O_MASK (0x0002u)
N#define CSL_EMIF_CS3ECC1_P2O_SHIFT (0x0001u)
N#define CSL_EMIF_CS3ECC1_P2O_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS3ECC1_P1O_MASK (0x0001u)
N#define CSL_EMIF_CS3ECC1_P1O_SHIFT (0x0000u)
N#define CSL_EMIF_CS3ECC1_P1O_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS3ECC1_RESETVAL (0x0000u)
N
N/* CS4ECC0 */
N
N
N
N#define CSL_EMIF_CS4ECC0_P2048E_MASK (0x0800u)
N#define CSL_EMIF_CS4ECC0_P2048E_SHIFT (0x000Bu)
N#define CSL_EMIF_CS4ECC0_P2048E_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS4ECC0_P1024E_MASK (0x0400u)
N#define CSL_EMIF_CS4ECC0_P1024E_SHIFT (0x000Au)
N#define CSL_EMIF_CS4ECC0_P1024E_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS4ECC0_P512E_MASK (0x0200u)
N#define CSL_EMIF_CS4ECC0_P512E_SHIFT (0x0009u)
N#define CSL_EMIF_CS4ECC0_P512E_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS4ECC0_P256E_MASK (0x0100u)
N#define CSL_EMIF_CS4ECC0_P256E_SHIFT (0x0008u)
N#define CSL_EMIF_CS4ECC0_P256E_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS4ECC0_P128E_MASK (0x0080u)
N#define CSL_EMIF_CS4ECC0_P128E_SHIFT (0x0007u)
N#define CSL_EMIF_CS4ECC0_P128E_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS4ECC0_P64E_MASK (0x0040u)
N#define CSL_EMIF_CS4ECC0_P64E_SHIFT (0x0006u)
N#define CSL_EMIF_CS4ECC0_P64E_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS4ECC0_P32E_MASK (0x0020u)
N#define CSL_EMIF_CS4ECC0_P32E_SHIFT (0x0005u)
N#define CSL_EMIF_CS4ECC0_P32E_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS4ECC0_P16E_MASK (0x0010u)
N#define CSL_EMIF_CS4ECC0_P16E_SHIFT (0x0004u)
N#define CSL_EMIF_CS4ECC0_P16E_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS4ECC0_P8E_MASK (0x0008u)
N#define CSL_EMIF_CS4ECC0_P8E_SHIFT (0x0003u)
N#define CSL_EMIF_CS4ECC0_P8E_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS4ECC0_P4E_MASK (0x0004u)
N#define CSL_EMIF_CS4ECC0_P4E_SHIFT (0x0002u)
N#define CSL_EMIF_CS4ECC0_P4E_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS4ECC0_P2E_MASK (0x0002u)
N#define CSL_EMIF_CS4ECC0_P2E_SHIFT (0x0001u)
N#define CSL_EMIF_CS4ECC0_P2E_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS4ECC0_P1E_MASK (0x0001u)
N#define CSL_EMIF_CS4ECC0_P1E_SHIFT (0x0000u)
N#define CSL_EMIF_CS4ECC0_P1E_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS4ECC0_RESETVAL (0x0000u)
N
N/* CS4ECC1 */
N
N
N#define CSL_EMIF_CS4ECC1_P2048O_MASK (0x0800u)
N#define CSL_EMIF_CS4ECC1_P2048O_SHIFT (0x000Bu)
N#define CSL_EMIF_CS4ECC1_P2048O_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS4ECC1_P1024O_MASK (0x0400u)
N#define CSL_EMIF_CS4ECC1_P1024O_SHIFT (0x000Au)
N#define CSL_EMIF_CS4ECC1_P1024O_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS4ECC1_P512O_MASK (0x0200u)
N#define CSL_EMIF_CS4ECC1_P512O_SHIFT (0x0009u)
N#define CSL_EMIF_CS4ECC1_P512O_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS4ECC1_P256O_MASK (0x0100u)
N#define CSL_EMIF_CS4ECC1_P256O_SHIFT (0x0008u)
N#define CSL_EMIF_CS4ECC1_P256O_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS4ECC1_P128O_MASK (0x0080u)
N#define CSL_EMIF_CS4ECC1_P128O_SHIFT (0x0007u)
N#define CSL_EMIF_CS4ECC1_P128O_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS4ECC1_P64O_MASK (0x0040u)
N#define CSL_EMIF_CS4ECC1_P64O_SHIFT (0x0006u)
N#define CSL_EMIF_CS4ECC1_P64O_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS4ECC1_P32O_MASK (0x0020u)
N#define CSL_EMIF_CS4ECC1_P32O_SHIFT (0x0005u)
N#define CSL_EMIF_CS4ECC1_P32O_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS4ECC1_P16O_MASK (0x0010u)
N#define CSL_EMIF_CS4ECC1_P16O_SHIFT (0x0004u)
N#define CSL_EMIF_CS4ECC1_P16O_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS4ECC1_P8O_MASK (0x0008u)
N#define CSL_EMIF_CS4ECC1_P8O_SHIFT (0x0003u)
N#define CSL_EMIF_CS4ECC1_P8O_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS4ECC1_P4O_MASK (0x0004u)
N#define CSL_EMIF_CS4ECC1_P4O_SHIFT (0x0002u)
N#define CSL_EMIF_CS4ECC1_P4O_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS4ECC1_P2O_MASK (0x0002u)
N#define CSL_EMIF_CS4ECC1_P2O_SHIFT (0x0001u)
N#define CSL_EMIF_CS4ECC1_P2O_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS4ECC1_P1O_MASK (0x0001u)
N#define CSL_EMIF_CS4ECC1_P1O_SHIFT (0x0000u)
N#define CSL_EMIF_CS4ECC1_P1O_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS4ECC1_RESETVAL (0x0000u)
N
N/* CS5ECC0 */
N
N
N
N#define CSL_EMIF_CS5ECC0_P2048E_MASK (0x0800u)
N#define CSL_EMIF_CS5ECC0_P2048E_SHIFT (0x000Bu)
N#define CSL_EMIF_CS5ECC0_P2048E_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS5ECC0_P1024E_MASK (0x0400u)
N#define CSL_EMIF_CS5ECC0_P1024E_SHIFT (0x000Au)
N#define CSL_EMIF_CS5ECC0_P1024E_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS5ECC0_P512E_MASK (0x0200u)
N#define CSL_EMIF_CS5ECC0_P512E_SHIFT (0x0009u)
N#define CSL_EMIF_CS5ECC0_P512E_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS5ECC0_P256E_MASK (0x0100u)
N#define CSL_EMIF_CS5ECC0_P256E_SHIFT (0x0008u)
N#define CSL_EMIF_CS5ECC0_P256E_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS5ECC0_P128E_MASK (0x0080u)
N#define CSL_EMIF_CS5ECC0_P128E_SHIFT (0x0007u)
N#define CSL_EMIF_CS5ECC0_P128E_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS5ECC0_P64E_MASK (0x0040u)
N#define CSL_EMIF_CS5ECC0_P64E_SHIFT (0x0006u)
N#define CSL_EMIF_CS5ECC0_P64E_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS5ECC0_P32E_MASK (0x0020u)
N#define CSL_EMIF_CS5ECC0_P32E_SHIFT (0x0005u)
N#define CSL_EMIF_CS5ECC0_P32E_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS5ECC0_P16E_MASK (0x0010u)
N#define CSL_EMIF_CS5ECC0_P16E_SHIFT (0x0004u)
N#define CSL_EMIF_CS5ECC0_P16E_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS5ECC0_P8E_MASK (0x0008u)
N#define CSL_EMIF_CS5ECC0_P8E_SHIFT (0x0003u)
N#define CSL_EMIF_CS5ECC0_P8E_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS5ECC0_P4E_MASK (0x0004u)
N#define CSL_EMIF_CS5ECC0_P4E_SHIFT (0x0002u)
N#define CSL_EMIF_CS5ECC0_P4E_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS5ECC0_P2E_MASK (0x0002u)
N#define CSL_EMIF_CS5ECC0_P2E_SHIFT (0x0001u)
N#define CSL_EMIF_CS5ECC0_P2E_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS5ECC0_P1E_MASK (0x0001u)
N#define CSL_EMIF_CS5ECC0_P1E_SHIFT (0x0000u)
N#define CSL_EMIF_CS5ECC0_P1E_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS5ECC0_RESETVAL (0x0000u)
N
N/* CS5ECC1 */
N
N
N#define CSL_EMIF_CS5ECC1_P2048O_MASK (0x0800u)
N#define CSL_EMIF_CS5ECC1_P2048O_SHIFT (0x000Bu)
N#define CSL_EMIF_CS5ECC1_P2048O_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS5ECC1_P1024O_MASK (0x0400u)
N#define CSL_EMIF_CS5ECC1_P1024O_SHIFT (0x000Au)
N#define CSL_EMIF_CS5ECC1_P1024O_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS5ECC1_P512O_MASK (0x0200u)
N#define CSL_EMIF_CS5ECC1_P512O_SHIFT (0x0009u)
N#define CSL_EMIF_CS5ECC1_P512O_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS5ECC1_P256O_MASK (0x0100u)
N#define CSL_EMIF_CS5ECC1_P256O_SHIFT (0x0008u)
N#define CSL_EMIF_CS5ECC1_P256O_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS5ECC1_P128O_MASK (0x0080u)
N#define CSL_EMIF_CS5ECC1_P128O_SHIFT (0x0007u)
N#define CSL_EMIF_CS5ECC1_P128O_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS5ECC1_P64O_MASK (0x0040u)
N#define CSL_EMIF_CS5ECC1_P64O_SHIFT (0x0006u)
N#define CSL_EMIF_CS5ECC1_P64O_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS5ECC1_P32O_MASK (0x0020u)
N#define CSL_EMIF_CS5ECC1_P32O_SHIFT (0x0005u)
N#define CSL_EMIF_CS5ECC1_P32O_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS5ECC1_P16O_MASK (0x0010u)
N#define CSL_EMIF_CS5ECC1_P16O_SHIFT (0x0004u)
N#define CSL_EMIF_CS5ECC1_P16O_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS5ECC1_P8O_MASK (0x0008u)
N#define CSL_EMIF_CS5ECC1_P8O_SHIFT (0x0003u)
N#define CSL_EMIF_CS5ECC1_P8O_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS5ECC1_P4O_MASK (0x0004u)
N#define CSL_EMIF_CS5ECC1_P4O_SHIFT (0x0002u)
N#define CSL_EMIF_CS5ECC1_P4O_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS5ECC1_P2O_MASK (0x0002u)
N#define CSL_EMIF_CS5ECC1_P2O_SHIFT (0x0001u)
N#define CSL_EMIF_CS5ECC1_P2O_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS5ECC1_P1O_MASK (0x0001u)
N#define CSL_EMIF_CS5ECC1_P1O_SHIFT (0x0000u)
N#define CSL_EMIF_CS5ECC1_P1O_RESETVAL (0x0000u)
N
N#define CSL_EMIF_CS5ECC1_RESETVAL (0x0000u)
N
N/* VERSION */
N
N
N#define CSL_EMIF_VERSION_RELNUM_MASK (0x00FFu)
N#define CSL_EMIF_VERSION_RELNUM_SHIFT (0x0000u)
N#define CSL_EMIF_VERSION_RELNUM_RESETVAL (0x0000u)
N
N#define CSL_EMIF_VERSION_RESETVAL (0x0000u)
N
N/* FOURBITECCLD */
N
N
N#define CSL_EMIF_FOURBITECCLD_4BITECC_LOAD_MASK (0x03FFu)
N#define CSL_EMIF_FOURBITECCLD_4BITECC_LOAD_SHIFT (0x0000u)
N#define CSL_EMIF_FOURBITECCLD_4BITECC_LOAD_RESETVAL (0x0000u)
N
N#define CSL_EMIF_FOURBITECCLD_RESETVAL (0x0000u)
N
N/* FOURBITECC10 */
N
N
N#define CSL_EMIF_FOURBITECC10_VAL1_MASK (0x03FFu)
N#define CSL_EMIF_FOURBITECC10_VAL1_SHIFT (0x0000u)
N#define CSL_EMIF_FOURBITECC10_VAL1_RESETVAL (0x0000u)
N
N#define CSL_EMIF_FOURBITECC10_RESETVAL (0x0000u)
N
N/* FOURBITECC11 */
N
N
N#define CSL_EMIF_FOURBITECC11_VAL2_MASK (0x03FFu)
N#define CSL_EMIF_FOURBITECC11_VAL2_SHIFT (0x0000u)
N#define CSL_EMIF_FOURBITECC11_VAL2_RESETVAL (0x0000u)
N
N#define CSL_EMIF_FOURBITECC11_RESETVAL (0x0000u)
N
N/* FOURBITECC20 */
N
N
N#define CSL_EMIF_FOURBITECC20_VAL1_MASK (0x03FFu)
N#define CSL_EMIF_FOURBITECC20_VAL1_SHIFT (0x0000u)
N#define CSL_EMIF_FOURBITECC20_VAL1_RESETVAL (0x0000u)
N
N#define CSL_EMIF_FOURBITECC20_RESETVAL (0x0000u)
N
N/* FOURBITECC21 */
N
N
N#define CSL_EMIF_FOURBITECC21_VAL2_MASK (0x03FFu)
N#define CSL_EMIF_FOURBITECC21_VAL2_SHIFT (0x0000u)
N#define CSL_EMIF_FOURBITECC21_VAL2_RESETVAL (0x0000u)
N
N#define CSL_EMIF_FOURBITECC21_RESETVAL (0x0000u)
N
N/* FOURBITECC30 */
N
N
N#define CSL_EMIF_FOURBITECC30_VAL1_MASK (0x03FFu)
N#define CSL_EMIF_FOURBITECC30_VAL1_SHIFT (0x0000u)
N#define CSL_EMIF_FOURBITECC30_VAL1_RESETVAL (0x0000u)
N
N#define CSL_EMIF_FOURBITECC30_RESETVAL (0x0000u)
N
N/* FOURBITECC31 */
N
N
N#define CSL_EMIF_FOURBITECC31_VAL2_MASK (0x03FFu)
N#define CSL_EMIF_FOURBITECC31_VAL2_SHIFT (0x0000u)
N#define CSL_EMIF_FOURBITECC31_VAL2_RESETVAL (0x0000u)
N
N#define CSL_EMIF_FOURBITECC31_RESETVAL (0x0000u)
N
N/* FOURBITECC40 */
N
N
N#define CSL_EMIF_FOURBITECC40_VAL1_MASK (0x03FFu)
N#define CSL_EMIF_FOURBITECC40_VAL1_SHIFT (0x0000u)
N#define CSL_EMIF_FOURBITECC40_VAL1_RESETVAL (0x0000u)
N
N#define CSL_EMIF_FOURBITECC40_RESETVAL (0x0000u)
N
N/* FOURBITECC41 */
N
N
N#define CSL_EMIF_FOURBITECC41_VAL2_MASK (0x03FFu)
N#define CSL_EMIF_FOURBITECC41_VAL2_SHIFT (0x0000u)
N#define CSL_EMIF_FOURBITECC41_VAL2_RESETVAL (0x0000u)
N
N#define CSL_EMIF_FOURBITECC41_RESETVAL (0x0000u)
N
N/* ERRADRR10 */
N
N
N#define CSL_EMIF_ERRADRR10_ERRADDR1_MASK (0x03FFu)
N#define CSL_EMIF_ERRADRR10_ERRADDR1_SHIFT (0x0000u)
N#define CSL_EMIF_ERRADRR10_ERRADDR1_RESETVAL (0x0000u)
N
N#define CSL_EMIF_ERRADRR10_RESETVAL (0x0000u)
N
N/* ERRADRR11 */
N
N
N#define CSL_EMIF_ERRADRR11_ERRADDR2_MASK (0x03FFu)
N#define CSL_EMIF_ERRADRR11_ERRADDR2_SHIFT (0x0000u)
N#define CSL_EMIF_ERRADRR11_ERRADDR2_RESETVAL (0x0000u)
N
N#define CSL_EMIF_ERRADRR11_RESETVAL (0x0000u)
N
N/* ERRADRR20 */
N
N
N#define CSL_EMIF_ERRADRR20_ERRADDR1_MASK (0x03FFu)
N#define CSL_EMIF_ERRADRR20_ERRADDR1_SHIFT (0x0000u)
N#define CSL_EMIF_ERRADRR20_ERRADDR1_RESETVAL (0x0000u)
N
N#define CSL_EMIF_ERRADRR20_RESETVAL (0x0000u)
N
N/* ERRADRR21 */
N
N
N#define CSL_EMIF_ERRADRR21_ERRADDR2_MASK (0x03FFu)
N#define CSL_EMIF_ERRADRR21_ERRADDR2_SHIFT (0x0000u)
N#define CSL_EMIF_ERRADRR21_ERRADDR2_RESETVAL (0x0000u)
N
N#define CSL_EMIF_ERRADRR21_RESETVAL (0x0000u)
N
N/* ERRVAL10 */
N
N
N#define CSL_EMIF_ERRVAL10_ERRVAL1_MASK (0x03FFu)
N#define CSL_EMIF_ERRVAL10_ERRVAL1_SHIFT (0x0000u)
N#define CSL_EMIF_ERRVAL10_ERRVAL1_RESETVAL (0x0000u)
N
N#define CSL_EMIF_ERRVAL10_RESETVAL (0x0000u)
N
N/* ERRVAL11 */
N
N
N#define CSL_EMIF_ERRVAL11_ERRVAL2_MASK (0x03FFu)
N#define CSL_EMIF_ERRVAL11_ERRVAL2_SHIFT (0x0000u)
N#define CSL_EMIF_ERRVAL11_ERRVAL2_RESETVAL (0x0000u)
N
N#define CSL_EMIF_ERRVAL11_RESETVAL (0x0000u)
N
N/* ERRVAL20 */
N
N
N#define CSL_EMIF_ERRVAL20_ERRVAL1_MASK (0x03FFu)
N#define CSL_EMIF_ERRVAL20_ERRVAL1_SHIFT (0x0000u)
N#define CSL_EMIF_ERRVAL20_ERRVAL1_RESETVAL (0x0000u)
N
N#define CSL_EMIF_ERRVAL20_RESETVAL (0x0000u)
N
N/* ERRVAL21 */
N
N
N#define CSL_EMIF_ERRVAL21_ERRVAL2_MASK (0x03FFu)
N#define CSL_EMIF_ERRVAL21_ERRVAL2_SHIFT (0x0000u)
N#define CSL_EMIF_ERRVAL21_ERRVAL2_RESETVAL (0x0000u)
N
N#define CSL_EMIF_ERRVAL21_RESETVAL (0x0000u)
N
N#endif
L 59 "../common_inc/corazon.h" 2
N#include "cslr_uart_001.h"
L 1 "..\common_inc\cslr_uart_001.h" 1
N/*****************************************************************************
N * File Name : cslr_uart_001.h 
N *
N * Brief	 : Define UART register structure
N *
N * Copyright (C) 2009 -2010 Texas Instruments Incorporated - http://www.ti.com/ 
N * 
N * 
N *  Redistribution and use in source and binary forms, with or without 
N *  modification, are permitted provided that the following conditions 
N *  are met:
N *
N *    Redistributions of source code must retain the above copyright 
N *    notice, this list of conditions and the following disclaimer.
N *
N *    Redistributions in binary form must reproduce the above copyright
N *    notice, this list of conditions and the following disclaimer in the 
N *    documentation and/or other materials provided with the   
N *    distribution.
N *
N *    Neither the name of Texas Instruments Incorporated nor the names of
N *    its contributors may be used to endorse or promote products derived
N *    from this software without specific prior written permission.
N *
N *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
N *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
N *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
N *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
N *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
N *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
N *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
N *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
N *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
N *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
N *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
N *
N******************************************************************************/
N
N#ifndef _CSLR__UART_1_H_
N#define _CSLR__UART_1_H_
N
N#include <cslr.h>
N
N/* Minimum unit = 2 bytes */
N
N/**************************************************************************\
N* Register Overlay Structure
X
N\**************************************************************************/
Ntypedef struct  {
N    volatile Uint16 RBR;
N    volatile Uint16 RSVD0;
N    volatile Uint16 IER;
N    volatile Uint16 RSVD1;
N    volatile Uint16 IIR;
N    volatile Uint16 RSVD2;
N    volatile Uint16 LCR;
N    volatile Uint16 RSVD3;
N    volatile Uint16 MCR;
N    volatile Uint16 RSVD4;
N    volatile Uint16 LSR;
N    volatile Uint16 RSVD5;
N    volatile Uint16 MSR;
N    volatile Uint16 RSVD6;
N    volatile Uint16 SCR;
N    volatile Uint16 RSVD7;
N    volatile Uint16 DLL;
N    volatile Uint16 RSVD8;
N    volatile Uint16 DLH;
N    volatile Uint16 RSVD9;
N    volatile Uint16 PID1;
N    volatile Uint16 PID2;
N    volatile Uint16 RSVD10[2];
N    volatile Uint16 PWREMU_MGMT;
N    volatile Uint16 RSVD11;
N    volatile Uint16 MDR;
N} CSL_UartRegs;
N
N/* Following 2 lines are added due to  tools limitations */
N#define THR	RBR   /* RBR & THR have same offset */
N#define FCR IIR	  /* IIR & FCR have same offset */
N
N
N/**************************************************************************\
N* Field Definition Macros
X
N\**************************************************************************/
N
N/* RBR */
N
N#define CSL_UART_RBR_DATA_MASK (0x00FFu)
N#define CSL_UART_RBR_DATA_SHIFT (0x0000u)
N#define CSL_UART_RBR_DATA_RESETVAL (0x0000u)
N
N#define CSL_UART_RBR_RESETVAL (0x0000u)
N
N/* THR */
N
N#define CSL_UART_THR_DATA_MASK (0x00FFu)
N#define CSL_UART_THR_DATA_SHIFT (0x0000u)
N#define CSL_UART_THR_DATA_RESETVAL (0x0000u)
N
N#define CSL_UART_THR_RESETVAL (0x0000u)
N
N/* IER */
N
N#define CSL_UART_IER_EDSSI_MASK (0x0008u)
N#define CSL_UART_IER_EDSSI_SHIFT (0x0003u)
N#define CSL_UART_IER_EDSSI_RESETVAL (0x0000u)
N/*----EDSSI Tokens----*/
N#define CSL_UART_IER_EDSSI_DISABLE (0x0000u)
N#define CSL_UART_IER_EDSSI_ENABLE (0x0001u)
N
N#define CSL_UART_IER_ELSI_MASK (0x0004u)
N#define CSL_UART_IER_ELSI_SHIFT (0x0002u)
N#define CSL_UART_IER_ELSI_RESETVAL (0x0000u)
N/*----ELSI Tokens----*/
N#define CSL_UART_IER_ELSI_DISABLE (0x0000u)
N#define CSL_UART_IER_ELSI_ENABLE (0x0001u)
N
N#define CSL_UART_IER_ETBEI_MASK (0x0002u)
N#define CSL_UART_IER_ETBEI_SHIFT (0x0001u)
N#define CSL_UART_IER_ETBEI_RESETVAL (0x0000u)
N/*----ETBEI Tokens----*/
N#define CSL_UART_IER_ETBEI_DISABLE (0x0000u)
N#define CSL_UART_IER_ETBEI_ENABLE (0x0001u)
N
N#define CSL_UART_IER_ERBI_MASK (0x0001u)
N#define CSL_UART_IER_ERBI_SHIFT (0x0000u)
N#define CSL_UART_IER_ERBI_RESETVAL (0x0000u)
N/*----ERBI Tokens----*/
N#define CSL_UART_IER_ERBI_DISABLE (0x0000u)
N#define CSL_UART_IER_ERBI_ENABLE (0x0001u)
N
N#define CSL_UART_IER_RESETVAL (0x0000u)
N
N/* IIR */
N
N#define CSL_UART_IIR_FIFOEN_MASK (0x00C0u)
N#define CSL_UART_IIR_FIFOEN_SHIFT (0x0006u)
N#define CSL_UART_IIR_FIFOEN_RESETVAL (0x0000u)
N
N
N#define CSL_UART_IIR_INTID_MASK (0x000Eu)
N#define CSL_UART_IIR_INTID_SHIFT (0x0001u)
N#define CSL_UART_IIR_INTID_RESETVAL (0x0000u)
N/*----INTID Tokens----*/
N#define CSL_UART_IIR_INTID_MODSTAT (0x0000u)
N#define CSL_UART_IIR_INTID_THRE (0x0001u)
N#define CSL_UART_IIR_INTID_RDA (0x0002u)
N#define CSL_UART_IIR_INTID_RLS (0x0003u)
N#define CSL_UART_IIR_INTID_CTI (0x0006u)
N
N#define CSL_UART_IIR_IPEND_MASK (0x0001u)
N#define CSL_UART_IIR_IPEND_SHIFT (0x0000u)
N#define CSL_UART_IIR_IPEND_RESETVAL (0x0001u)
N/*----IPEND Tokens----*/
N#define CSL_UART_IIR_IPEND_NONE (0x0001u)
N
N#define CSL_UART_IIR_RESETVAL (0x0001u)
N
N/* FCR */
N
N
N#define CSL_UART_FCR_RXFIFTL_MASK (0x00C0u)
N#define CSL_UART_FCR_RXFIFTL_SHIFT (0x0006u)
N#define CSL_UART_FCR_RXFIFTL_RESETVAL (0x0000u)
N/*----RXFIFTL Tokens----*/
N#define CSL_UART_FCR_RXFIFTL_CHAR1 (0x0000u)
N#define CSL_UART_FCR_RXFIFTL_CHAR4 (0x0001u)
N#define CSL_UART_FCR_RXFIFTL_CHAR8 (0x0002u)
N#define CSL_UART_FCR_RXFIFTL_CHAR14 (0x0003u)
N
N
N#define CSL_UART_FCR_DMAMODE1_MASK (0x0008u)
N#define CSL_UART_FCR_DMAMODE1_SHIFT (0x0003u)
N#define CSL_UART_FCR_DMAMODE1_RESETVAL (0x0000u)
N/*----DMAMODE1 Tokens----*/
N#define CSL_UART_FCR_DMAMODE1_DISABLE (0x0000u)
N#define CSL_UART_FCR_DMAMODE1_ENABLE (0x0001u)
N
N#define CSL_UART_FCR_TXCLR_MASK (0x0004u)
N#define CSL_UART_FCR_TXCLR_SHIFT (0x0002u)
N#define CSL_UART_FCR_TXCLR_RESETVAL (0x0000u)
N/*----TXCLR Tokens----*/
N#define CSL_UART_FCR_TXCLR_CLR (0x0001u)
N
N#define CSL_UART_FCR_RXCLR_MASK (0x0002u)
N#define CSL_UART_FCR_RXCLR_SHIFT (0x0001u)
N#define CSL_UART_FCR_RXCLR_RESETVAL (0x0000u)
N/*----RXCLR Tokens----*/
N#define CSL_UART_FCR_RXCLR_CLR (0x0001u)
N
N#define CSL_UART_FCR_FIFOEN_MASK (0x0001u)
N#define CSL_UART_FCR_FIFOEN_SHIFT (0x0000u)
N#define CSL_UART_FCR_FIFOEN_RESETVAL (0x0000u)
N/*----FIFOEN Tokens----*/
N#define CSL_UART_FCR_FIFOEN_DISABLE (0x0000u)
N#define CSL_UART_FCR_FIFOEN_ENABLE (0x0001u)
N
N#define CSL_UART_FCR_RESETVAL (0x0000u)
N
N/* LCR */
N
N
N#define CSL_UART_LCR_DLAB_MASK (0x0080u)
N#define CSL_UART_LCR_DLAB_SHIFT (0x0007u)
N#define CSL_UART_LCR_DLAB_RESETVAL (0x0000u)
N
N#define CSL_UART_LCR_BC_MASK (0x0040u)
N#define CSL_UART_LCR_BC_SHIFT (0x0006u)
N#define CSL_UART_LCR_BC_RESETVAL (0x0000u)
N/*----BC Tokens----*/
N#define CSL_UART_LCR_BC_DISABLE (0x0000u)
N#define CSL_UART_LCR_BC_ENABLE (0x0001u)
N
N#define CSL_UART_LCR_SP_MASK (0x0020u)
N#define CSL_UART_LCR_SP_SHIFT (0x0005u)
N#define CSL_UART_LCR_SP_RESETVAL (0x0000u)
N
N#define CSL_UART_LCR_EPS_MASK (0x0010u)
N#define CSL_UART_LCR_EPS_SHIFT (0x0004u)
N#define CSL_UART_LCR_EPS_RESETVAL (0x0000u)
N
N#define CSL_UART_LCR_PEN_MASK (0x0008u)
N#define CSL_UART_LCR_PEN_SHIFT (0x0003u)
N#define CSL_UART_LCR_PEN_RESETVAL (0x0000u)
N
N#define CSL_UART_LCR_STB_MASK (0x0004u)
N#define CSL_UART_LCR_STB_SHIFT (0x0002u)
N#define CSL_UART_LCR_STB_RESETVAL (0x0000u)
N
N#define CSL_UART_LCR_WLS_MASK (0x0003u)
N#define CSL_UART_LCR_WLS_SHIFT (0x0000u)
N#define CSL_UART_LCR_WLS_RESETVAL (0x0000u)
N/*----WLS Tokens----*/
N#define CSL_UART_LCR_WLS_BITS5 (0x0000u)
N#define CSL_UART_LCR_WLS_BITS6 (0x0001u)
N#define CSL_UART_LCR_WLS_BITS7 (0x0002u)
N#define CSL_UART_LCR_WLS_BITS8 (0x0003u)
N
N#define CSL_UART_LCR_RESETVAL (0x0000u)
N
N/* MCR */
N
N
N#define CSL_UART_MCR_AFE_MASK (0x0020u)
N#define CSL_UART_MCR_AFE_SHIFT (0x0005u)
N#define CSL_UART_MCR_AFE_RESETVAL (0x0000u)
N/*----AFE Tokens----*/
N#define CSL_UART_MCR_AFE_DISABLE (0x0000u)
N#define CSL_UART_MCR_AFE_ENABLE (0x0001u)
N
N#define CSL_UART_MCR_LOOP_MASK (0x0010u)
N#define CSL_UART_MCR_LOOP_SHIFT (0x0004u)
N#define CSL_UART_MCR_LOOP_RESETVAL (0x0000u)
N/*----LOOP Tokens----*/
N#define CSL_UART_MCR_LOOP_DISABLE (0x0000u)
N#define CSL_UART_MCR_LOOP_ENABLE (0x0001u)
N
N#define CSL_UART_MCR_OUT2_MASK (0x0008u)
N#define CSL_UART_MCR_OUT2_SHIFT (0x0003u)
N#define CSL_UART_MCR_OUT2_RESETVAL (0x0000u)
N/*----OUT2 Tokens----*/
N#define CSL_UART_MCR_OUT2_HIGH (0x0000u)
N#define CSL_UART_MCR_OUT2_LOW (0x0001u)
N
N#define CSL_UART_MCR_OUT1_MASK (0x0004u)
N#define CSL_UART_MCR_OUT1_SHIFT (0x0002u)
N#define CSL_UART_MCR_OUT1_RESETVAL (0x0000u)
N/*----OUT1 Tokens----*/
N#define CSL_UART_MCR_OUT1_HIGH (0x0000u)
N#define CSL_UART_MCR_OUT1_LOW (0x0001u)
N
N#define CSL_UART_MCR_RTS_MASK (0x0002u)
N#define CSL_UART_MCR_RTS_SHIFT (0x0001u)
N#define CSL_UART_MCR_RTS_RESETVAL (0x0000u)
N/*----RTS Tokens----*/
N#define CSL_UART_MCR_RTS_HIGH (0x0000u)
N#define CSL_UART_MCR_RTS_DISABLE (0x0000u)
N#define CSL_UART_MCR_RTS_ENABLE (0x0001u)
N#define CSL_UART_MCR_RTS_LOW (0x0001u)
N
N#define CSL_UART_MCR_DTR_MASK (0x0001u)
N#define CSL_UART_MCR_DTR_SHIFT (0x0000u)
N#define CSL_UART_MCR_DTR_RESETVAL (0x0000u)
N/*----DTR Tokens----*/
N#define CSL_UART_MCR_DTR_HIGH (0x0000u)
N#define CSL_UART_MCR_DTR_LOW (0x0001u)
N
N#define CSL_UART_MCR_RESETVAL (0x0000u)
N
N/* LSR */
N
N
N#define CSL_UART_LSR_RXFIFOE_MASK (0x0080u)
N#define CSL_UART_LSR_RXFIFOE_SHIFT (0x0007u)
N#define CSL_UART_LSR_RXFIFOE_RESETVAL (0x0000u)
N
N#define CSL_UART_LSR_TEMT_MASK (0x0040u)
N#define CSL_UART_LSR_TEMT_SHIFT (0x0006u)
N#define CSL_UART_LSR_TEMT_RESETVAL (0x0001u)
N
N#define CSL_UART_LSR_THRE_MASK (0x0020u)
N#define CSL_UART_LSR_THRE_SHIFT (0x0005u)
N#define CSL_UART_LSR_THRE_RESETVAL (0x0001u)
N
N#define CSL_UART_LSR_BI_MASK (0x0010u)
N#define CSL_UART_LSR_BI_SHIFT (0x0004u)
N#define CSL_UART_LSR_BI_RESETVAL (0x0000u)
N
N#define CSL_UART_LSR_FE_MASK (0x0008u)
N#define CSL_UART_LSR_FE_SHIFT (0x0003u)
N#define CSL_UART_LSR_FE_RESETVAL (0x0000u)
N
N#define CSL_UART_LSR_PE_MASK (0x0004u)
N#define CSL_UART_LSR_PE_SHIFT (0x0002u)
N#define CSL_UART_LSR_PE_RESETVAL (0x0000u)
N
N#define CSL_UART_LSR_OE_MASK (0x0002u)
N#define CSL_UART_LSR_OE_SHIFT (0x0001u)
N#define CSL_UART_LSR_OE_RESETVAL (0x0000u)
N
N#define CSL_UART_LSR_DR_MASK (0x0001u)
N#define CSL_UART_LSR_DR_SHIFT (0x0000u)
N#define CSL_UART_LSR_DR_RESETVAL (0x0000u)
N
N#define CSL_UART_LSR_RESETVAL (0x0060u)
N
N/* MSR */
N
N
N#define CSL_UART_MSR_CD_MASK (0x0080u)
N#define CSL_UART_MSR_CD_SHIFT (0x0007u)
N#define CSL_UART_MSR_CD_RESETVAL (0x0000u)
N
N#define CSL_UART_MSR_RI_MASK (0x0040u)
N#define CSL_UART_MSR_RI_SHIFT (0x0006u)
N#define CSL_UART_MSR_RI_RESETVAL (0x0000u)
N
N#define CSL_UART_MSR_DSR_MASK (0x0020u)
N#define CSL_UART_MSR_DSR_SHIFT (0x0005u)
N#define CSL_UART_MSR_DSR_RESETVAL (0x0000u)
N
N#define CSL_UART_MSR_CTS_MASK (0x0010u)
N#define CSL_UART_MSR_CTS_SHIFT (0x0004u)
N#define CSL_UART_MSR_CTS_RESETVAL (0x0000u)
N
N#define CSL_UART_MSR_DCD_MASK (0x0008u)
N#define CSL_UART_MSR_DCD_SHIFT (0x0003u)
N#define CSL_UART_MSR_DCD_RESETVAL (0x0000u)
N
N#define CSL_UART_MSR_TERI_MASK (0x0004u)
N#define CSL_UART_MSR_TERI_SHIFT (0x0002u)
N#define CSL_UART_MSR_TERI_RESETVAL (0x0000u)
N
N#define CSL_UART_MSR_DDSR_MASK (0x0002u)
N#define CSL_UART_MSR_DDSR_SHIFT (0x0001u)
N#define CSL_UART_MSR_DDSR_RESETVAL (0x0000u)
N
N#define CSL_UART_MSR_DCTS_MASK (0x0001u)
N#define CSL_UART_MSR_DCTS_SHIFT (0x0000u)
N#define CSL_UART_MSR_DCTS_RESETVAL (0x0000u)
N
N#define CSL_UART_MSR_RESETVAL (0x0000u)
N
N/* SCR */
N
N
N#define CSL_UART_SCR_DATA_MASK (0x00FFu)
N#define CSL_UART_SCR_DATA_SHIFT (0x0000u)
N#define CSL_UART_SCR_DATA_RESETVAL (0x0000u)
N
N#define CSL_UART_SCR_RESETVAL (0x0000u)
N
N/* DLL */
N
N
N#define CSL_UART_DLL_DLL_MASK (0x00FFu)
N#define CSL_UART_DLL_DLL_SHIFT (0x0000u)
N#define CSL_UART_DLL_DLL_RESETVAL (0x0000u)
N
N#define CSL_UART_DLL_RESETVAL (0x0000u)
N
N/* DLH */
N
N
N#define CSL_UART_DLH_DLH_MASK (0x00FFu)
N#define CSL_UART_DLH_DLH_SHIFT (0x0000u)
N#define CSL_UART_DLH_DLH_RESETVAL (0x0000u)
N
N#define CSL_UART_DLH_RESETVAL (0x0000u)
N
N/* PID1 */
N
N#define CSL_UART_PID1_CLS_MASK (0xFF00u)
N#define CSL_UART_PID1_CLS_SHIFT (0x0008u)
N#define CSL_UART_PID1_CLS_RESETVAL (0x0001u)
N
N#define CSL_UART_PID1_REV_MASK (0x00FFu)
N#define CSL_UART_PID1_REV_SHIFT (0x0000u)
N#define CSL_UART_PID1_REV_RESETVAL (0x0001u)
N
N#define CSL_UART_PID1_RESETVAL (0x0101u)
N
N/* PID2 */
N
N#define CSL_UART_PID2_FUNC_MASK (0xFF00u)
N#define CSL_UART_PID2_FUNC_SHIFT (0x0008u)
N#define CSL_UART_PID2_FUNC_RESETVAL (0x0000u)
N
N#define CSL_UART_PID2_TYP_MASK (0x00FFu)
N#define CSL_UART_PID2_TYP_SHIFT (0x0000u)
N#define CSL_UART_PID2_TYP_RESETVAL (0x0004u)
N
N#define CSL_UART_PID2_RESETVAL (0x0004u)
N
N/* PWREMU_MGMT */
N
N
N#define CSL_UART_PWREMU_MGMT_UTRST_MASK (0x4000u)
N#define CSL_UART_PWREMU_MGMT_UTRST_SHIFT (0x000Eu)
N#define CSL_UART_PWREMU_MGMT_UTRST_RESETVAL (0x0000u)
N/*----UTRST Tokens----*/
N#define CSL_UART_PWREMU_MGMT_UTRST_RESET (0x0000u)
N
N#define CSL_UART_PWREMU_MGMT_URRST_MASK (0x2000u)
N#define CSL_UART_PWREMU_MGMT_URRST_SHIFT (0x000Du)
N#define CSL_UART_PWREMU_MGMT_URRST_RESETVAL (0x0000u)
N/*----URRST Tokens----*/
N#define CSL_UART_PWREMU_MGMT_URRST_RESET (0x0000u)
N#define CSL_UART_PWREMU_MGMT_URRST_ENABLE (0x0001u)
N
N
N#define CSL_UART_PWREMU_MGMT_SOFT_MASK (0x0002u)
N#define CSL_UART_PWREMU_MGMT_SOFT_SHIFT (0x0001u)
N#define CSL_UART_PWREMU_MGMT_SOFT_RESETVAL (0x0001u)
N
N#define CSL_UART_PWREMU_MGMT_FREE_MASK (0x0001u)
N#define CSL_UART_PWREMU_MGMT_FREE_SHIFT (0x0000u)
N#define CSL_UART_PWREMU_MGMT_FREE_RESETVAL (0x0000u)
N/*----FREE Tokens----*/
N#define CSL_UART_PWREMU_MGMT_FREE_STOP (0x0000u)
N#define CSL_UART_PWREMU_MGMT_FREE_RUN (0x0001u)
N
N#define CSL_UART_PWREMU_MGMT_RESETVAL (0x0002u)
N
N/* MDR */
N
N
N#define CSL_UART_MDR_OSM_SEL_MASK (0x0001u)
N#define CSL_UART_MDR_OSM_SEL_SHIFT (0x0000u)
N#define CSL_UART_MDR_OSM_SEL_RESETVAL (0x0000u)
N/*----OSM_SEL Tokens----*/
N#define CSL_UART_MDR_OSM_SEL_16XOVERSAMPLINGUSED (0x0000u)
N#define CSL_UART_MDR_OSM_SEL_13XOVERSAMPLINGUSED (0x0001u)
N
N#define CSL_UART_MDR_RESETVAL (0x0000u)
N
N#endif
L 60 "../common_inc/corazon.h" 2
N#include "cslr_spi_001.h"
N#include "cslr_mmcsd_001.h"
L 1 "..\common_inc\cslr_mmcsd_001.h" 1
N/*****************************************************************************
N * File Name : cslr_mmcsd_001.h 
N *
N * Brief	 : Define MMC/SD register structure
N *
N * Copyright (C) 2009 -2010 Texas Instruments Incorporated - http://www.ti.com/ 
N * 
N * 
N *  Redistribution and use in source and binary forms, with or without 
N *  modification, are permitted provided that the following conditions 
N *  are met:
N *
N *    Redistributions of source code must retain the above copyright 
N *    notice, this list of conditions and the following disclaimer.
N *
N *    Redistributions in binary form must reproduce the above copyright
N *    notice, this list of conditions and the following disclaimer in the 
N *    documentation and/or other materials provided with the   
N *    distribution.
N *
N *    Neither the name of Texas Instruments Incorporated nor the names of
N *    its contributors may be used to endorse or promote products derived
N *    from this software without specific prior written permission.
N *
N *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
N *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
N *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
N *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
N *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
N *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
N *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
N *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
N *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
N *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
N *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
N *
N******************************************************************************/
N#ifndef _CSLR__MMCSD_1_H_
N#define _CSLR__MMCSD_1_H_
N
N#include <cslr.h>
N/* Minimum unit = 2 bytes */
N
N/**************************************************************************\
N* Register Overlay Structure
X
N\**************************************************************************/
Ntypedef struct  {
N    volatile Uint16 MMCCTL;
N    volatile Uint16 RSVD0[3];
N    volatile Uint16 MMCCLK;
N    volatile Uint16 RSVD1[3];
N    volatile Uint16 MMCST0;
N    volatile Uint16 RSVD2[3];
N    volatile Uint16 MMCST1;
N    volatile Uint16 RSVD3[3];
N    volatile Uint16 MMCIM;
N    volatile Uint16 RSVD4[3];
N    volatile Uint16 MMCTOR;
N    volatile Uint16 RSVD5[3];
N    volatile Uint16 MMCTODL;
N    volatile Uint16 MMCTODH;
N    volatile Uint16 RSVD6[2];
N    volatile Uint16 MMCBLEN;
N    volatile Uint16 RSVD7[3];
N    volatile Uint16 MMCNBLK;
N    volatile Uint16 RSVD8[3];
N    volatile Uint16 MMCNBLC;
N    volatile Uint16 RSVD9[3];
N    volatile Uint16 MMCDRRL;
N    volatile Uint16 MMCDRRH;
N    volatile Uint16 RSVD10[2];
N    volatile Uint16 MMCDXRL;
N    volatile Uint16 MMCDXRH;
N    volatile Uint16 RSVD11[2];
N    volatile Uint16 MMCCMDL;
N    volatile Uint16 MMCCMDH;
N    volatile Uint16 RSVD12[2];
N    volatile Uint16 MMCARGL;
N    volatile Uint16 MMCARGH;
N    volatile Uint16 RSVD13[2];
N    volatile Uint16 MMCRSP0;
N    volatile Uint16 MMCRSP1;
N    volatile Uint16 RSVD14[2];
N    volatile Uint16 MMCRSP2;
N    volatile Uint16 MMCRSP3;
N    volatile Uint16 RSVD15[2];
N    volatile Uint16 MMCRSP4;
N    volatile Uint16 MMCRSP5;
N    volatile Uint16 RSVD16[2];
N    volatile Uint16 MMCRSP6;
N    volatile Uint16 MMCRSP7;
N    volatile Uint16 RSVD17[2];
N    volatile Uint16 MMCDRSP;
N    volatile Uint16 RSVD18[3];
N    volatile Uint16 MMCETOK;
N    volatile Uint16 RSVD19[3];
N    volatile Uint16 MMCCIDX;
N    volatile Uint16 RSVD20[3];
N    volatile Uint16 MMCCKC;
N    volatile Uint16 RSVD21[3];
N    volatile Uint16 MMCTORC;
N    volatile Uint16 RSVD22[3];
N    volatile Uint16 MMCTODC;
N    volatile Uint16 RSVD23[3];
N    volatile Uint16 MMCBLNC;
N    volatile Uint16 RSVD24[3];
N    volatile Uint16 SDIOCTL;
N    volatile Uint16 RSVD25[3];
N    volatile Uint16 SDIOST0;
N    volatile Uint16 RSVD26[3];
N    volatile Uint16 SDIOIEN;
N    volatile Uint16 RSVD27[3];
N    volatile Uint16 SDIOIST ;
N    volatile Uint16 RSVD28[3];
N    volatile Uint16 MMCFIFOCTL;
N} CSL_MmcsdRegs;
N
N/**************************************************************************\
N* Field Definition Macros
X
N\**************************************************************************/
N
N/* MMCCTL */
N
N
N#define CSL_MMCSD_MMCCTL_PERMDX_MASK (0x0400u)
N#define CSL_MMCSD_MMCCTL_PERMDX_SHIFT (0x000Au)
N#define CSL_MMCSD_MMCCTL_PERMDX_RESETVAL (0x0000u)
N/*----PERMDX Tokens----*/
N#define CSL_MMCSD_MMCCTL_PERMDX_LEND (0x0000u)
N#define CSL_MMCSD_MMCCTL_PERMDX_BEND (0x0001u)
N
N#define CSL_MMCSD_MMCCTL_PERMDR_MASK (0x0200u)
N#define CSL_MMCSD_MMCCTL_PERMDR_SHIFT (0x0009u)
N#define CSL_MMCSD_MMCCTL_PERMDR_RESETVAL (0x0000u)
N/*----PERMDR Tokens----*/
N#define CSL_MMCSD_MMCCTL_PERMDR_LEND (0x0000u)
N#define CSL_MMCSD_MMCCTL_PERMDR_BEND (0x0001u)
N
N
N#define CSL_MMCSD_MMCCTL_DATEG_MASK (0x00C0u)
N#define CSL_MMCSD_MMCCTL_DATEG_SHIFT (0x0006u)
N#define CSL_MMCSD_MMCCTL_DATEG_RESETVAL (0x0000u)
N/*----DATEG Tokens----*/
N#define CSL_MMCSD_MMCCTL_DATEG_DISABLE (0x0000u)
N#define CSL_MMCSD_MMCCTL_DATEG_R_EDGE (0x0001u)
N#define CSL_MMCSD_MMCCTL_DATEG_F_EDGE (0x0002u)
N#define CSL_MMCSD_MMCCTL_DATEG_RF_EDGE (0x0003u)
N
N
N#define CSL_MMCSD_MMCCTL_WIDTH_MASK (0x0004u)
N#define CSL_MMCSD_MMCCTL_WIDTH_SHIFT (0x0002u)
N#define CSL_MMCSD_MMCCTL_WIDTH_RESETVAL (0x0000u)
N/*----WIDTH Tokens----*/
N#define CSL_MMCSD_MMCCTL_WIDTH_BIT1 (0x0000u)
N#define CSL_MMCSD_MMCCTL_WIDTH_BIT4 (0x0001u)
N
N#define CSL_MMCSD_MMCCTL_CMDRST_MASK (0x0002u)
N#define CSL_MMCSD_MMCCTL_CMDRST_SHIFT (0x0001u)
N#define CSL_MMCSD_MMCCTL_CMDRST_RESETVAL (0x0000u)
N/*----CMDRST Tokens----*/
N#define CSL_MMCSD_MMCCTL_CMDRST_ENABLE (0x0000u)
N#define CSL_MMCSD_MMCCTL_CMDRST_DISABLE (0x0001u)
N
N#define CSL_MMCSD_MMCCTL_DATRST_MASK (0x0001u)
N#define CSL_MMCSD_MMCCTL_DATRST_SHIFT (0x0000u)
N#define CSL_MMCSD_MMCCTL_DATRST_RESETVAL (0x0000u)
N/*----DATRST Tokens----*/
N#define CSL_MMCSD_MMCCTL_DATRST_ENABLE (0x0000u)
N#define CSL_MMCSD_MMCCTL_DATRST_DISABLE (0x0001u)
N
N#define CSL_MMCSD_MMCCTL_RESETVAL (0x0000u)
N
N/* MMCCLK */
N
N
N#define CSL_MMCSD_MMCCLK_CLKEN_MASK (0x0100u)
N#define CSL_MMCSD_MMCCLK_CLKEN_SHIFT (0x0008u)
N#define CSL_MMCSD_MMCCLK_CLKEN_RESETVAL (0x0000u)
N/*----CLKEN Tokens----*/
N#define CSL_MMCSD_MMCCLK_CLKEN_DISABLE (0x0000u)
N#define CSL_MMCSD_MMCCLK_CLKEN_ENABLE (0x0001u)
N
N#define CSL_MMCSD_MMCCLK_CLKRT_MASK (0x00FFu)
N#define CSL_MMCSD_MMCCLK_CLKRT_SHIFT (0x0000u)
N#define CSL_MMCSD_MMCCLK_CLKRT_RESETVAL (0x00FFu)
N
N#define CSL_MMCSD_MMCCLK_RESETVAL (0x00FFu)
N
N/* MMCST0 */
N
N
N#define CSL_MMCSD_MMCST0_TRNDNE_MASK (0x1000u)
N#define CSL_MMCSD_MMCST0_TRNDNE_SHIFT (0x000Cu)
N#define CSL_MMCSD_MMCST0_TRNDNE_RESETVAL (0x0000u)
N
N#define CSL_MMCSD_MMCST0_DATED_MASK (0x0800u)
N#define CSL_MMCSD_MMCST0_DATED_SHIFT (0x000Bu)
N#define CSL_MMCSD_MMCST0_DATED_RESETVAL (0x0000u)
N
N#define CSL_MMCSD_MMCST0_DRRDY_MASK (0x0400u)
N#define CSL_MMCSD_MMCST0_DRRDY_SHIFT (0x000Au)
N#define CSL_MMCSD_MMCST0_DRRDY_RESETVAL (0x0000u)
N
N#define CSL_MMCSD_MMCST0_DXRDY_MASK (0x0200u)
N#define CSL_MMCSD_MMCST0_DXRDY_SHIFT (0x0009u)
N#define CSL_MMCSD_MMCST0_DXRDY_RESETVAL (0x0001u)
N
N
N#define CSL_MMCSD_MMCST0_CRCRS_MASK (0x0080u)
N#define CSL_MMCSD_MMCST0_CRCRS_SHIFT (0x0007u)
N#define CSL_MMCSD_MMCST0_CRCRS_RESETVAL (0x0000u)
N
N#define CSL_MMCSD_MMCST0_CRCRD_MASK (0x0040u)
N#define CSL_MMCSD_MMCST0_CRCRD_SHIFT (0x0006u)
N#define CSL_MMCSD_MMCST0_CRCRD_RESETVAL (0x0000u)
N
N#define CSL_MMCSD_MMCST0_CRCWR_MASK (0x0020u)
N#define CSL_MMCSD_MMCST0_CRCWR_SHIFT (0x0005u)
N#define CSL_MMCSD_MMCST0_CRCWR_RESETVAL (0x0000u)
N
N#define CSL_MMCSD_MMCST0_TOUTRS_MASK (0x0010u)
N#define CSL_MMCSD_MMCST0_TOUTRS_SHIFT (0x0004u)
N#define CSL_MMCSD_MMCST0_TOUTRS_RESETVAL (0x0000u)
N
N#define CSL_MMCSD_MMCST0_TOUTRD_MASK (0x0008u)
N#define CSL_MMCSD_MMCST0_TOUTRD_SHIFT (0x0003u)
N#define CSL_MMCSD_MMCST0_TOUTRD_RESETVAL (0x0000u)
N
N#define CSL_MMCSD_MMCST0_RSPDNE_MASK (0x0004u)
N#define CSL_MMCSD_MMCST0_RSPDNE_SHIFT (0x0002u)
N#define CSL_MMCSD_MMCST0_RSPDNE_RESETVAL (0x0000u)
N
N#define CSL_MMCSD_MMCST0_BSYDNE_MASK (0x0002u)
N#define CSL_MMCSD_MMCST0_BSYDNE_SHIFT (0x0001u)
N#define CSL_MMCSD_MMCST0_BSYDNE_RESETVAL (0x0000u)
N
N#define CSL_MMCSD_MMCST0_DATDNE_MASK (0x0001u)
N#define CSL_MMCSD_MMCST0_DATDNE_SHIFT (0x0000u)
N#define CSL_MMCSD_MMCST0_DATDNE_RESETVAL (0x0000u)
N
N#define CSL_MMCSD_MMCST0_RESETVAL (0x0200u)
N
N/* MMCST1 */
N
N
N#define CSL_MMCSD_MMCST1_FIFOFUL_MASK (0x0040u)
N#define CSL_MMCSD_MMCST1_FIFOFUL_SHIFT (0x0006u)
N#define CSL_MMCSD_MMCST1_FIFOFUL_RESETVAL (0x0000u)
N
N#define CSL_MMCSD_MMCST1_FIFOEMP_MASK (0x0020u)
N#define CSL_MMCSD_MMCST1_FIFOEMP_SHIFT (0x0005u)
N#define CSL_MMCSD_MMCST1_FIFOEMP_RESETVAL (0x0000u)
N
N#define CSL_MMCSD_MMCST1_DAT3ST_MASK (0x0010u)
N#define CSL_MMCSD_MMCST1_DAT3ST_SHIFT (0x0004u)
N#define CSL_MMCSD_MMCST1_DAT3ST_RESETVAL (0x0000u)
N
N#define CSL_MMCSD_MMCST1_DRFUL_MASK (0x0008u)
N#define CSL_MMCSD_MMCST1_DRFUL_SHIFT (0x0003u)
N#define CSL_MMCSD_MMCST1_DRFUL_RESETVAL (0x0000u)
N
N#define CSL_MMCSD_MMCST1_DXEMP_MASK (0x0004u)
N#define CSL_MMCSD_MMCST1_DXEMP_SHIFT (0x0002u)
N#define CSL_MMCSD_MMCST1_DXEMP_RESETVAL (0x0000u)
N
N#define CSL_MMCSD_MMCST1_CLKSTP_MASK (0x0002u)
N#define CSL_MMCSD_MMCST1_CLKSTP_SHIFT (0x0001u)
N#define CSL_MMCSD_MMCST1_CLKSTP_RESETVAL (0x0001u)
N
N#define CSL_MMCSD_MMCST1_BUSY_MASK (0x0001u)
N#define CSL_MMCSD_MMCST1_BUSY_SHIFT (0x0000u)
N#define CSL_MMCSD_MMCST1_BUSY_RESETVAL (0x0000u)
N
N#define CSL_MMCSD_MMCST1_RESETVAL (0x0002u)
N
N/* MMCIM */
N
N
N#define CSL_MMCSD_MMCIM_ETRNDNE_MASK (0x1000u)
N#define CSL_MMCSD_MMCIM_ETRNDNE_SHIFT (0x000Cu)
N#define CSL_MMCSD_MMCIM_ETRNDNE_RESETVAL (0x0000u)
N/*----ETRNDNE Tokens----*/
N#define CSL_MMCSD_MMCIM_ETRNDNE_PROHIBIT (0x0000u)
N#define CSL_MMCSD_MMCIM_ETRNDNE_PERMIT (0x0001u)
N
N#define CSL_MMCSD_MMCIM_EDATED_MASK (0x0800u)
N#define CSL_MMCSD_MMCIM_EDATED_SHIFT (0x000Bu)
N#define CSL_MMCSD_MMCIM_EDATED_RESETVAL (0x0000u)
N/*----EDATED Tokens----*/
N#define CSL_MMCSD_MMCIM_EDATED_PROHIBIT (0x0000u)
N#define CSL_MMCSD_MMCIM_EDATED_PERMIT (0x0001u)
N
N#define CSL_MMCSD_MMCIM_EDRRDY_MASK (0x0400u)
N#define CSL_MMCSD_MMCIM_EDRRDY_SHIFT (0x000Au)
N#define CSL_MMCSD_MMCIM_EDRRDY_RESETVAL (0x0000u)
N/*----EDRRDY Tokens----*/
N#define CSL_MMCSD_MMCIM_EDRRDY_PROHIBIT (0x0000u)
N#define CSL_MMCSD_MMCIM_EDRRDY_PERMIT (0x0001u)
N
N#define CSL_MMCSD_MMCIM_EDXRDY_MASK (0x0200u)
N#define CSL_MMCSD_MMCIM_EDXRDY_SHIFT (0x0009u)
N#define CSL_MMCSD_MMCIM_EDXRDY_RESETVAL (0x0000u)
N/*----EDXRDY Tokens----*/
N#define CSL_MMCSD_MMCIM_EDXRDY_PROHIBIT (0x0000u)
N#define CSL_MMCSD_MMCIM_EDXRDY_PERMIT (0x0001u)
N
N#define CSL_MMCSD_MMCIM_ESPIERR_MASK (0x0100u)
N#define CSL_MMCSD_MMCIM_ESPIERR_SHIFT (0x0008u)
N#define CSL_MMCSD_MMCIM_ESPIERR_RESETVAL (0x0000u)
N/*----ESPIERR Tokens----*/
N#define CSL_MMCSD_MMCIM_ESPIERR_PROHIBIT (0x0000u)
N#define CSL_MMCSD_MMCIM_ESPIERR_PERMIT (0x0001u)
N
N#define CSL_MMCSD_MMCIM_ECRCRS_MASK (0x0080u)
N#define CSL_MMCSD_MMCIM_ECRCRS_SHIFT (0x0007u)
N#define CSL_MMCSD_MMCIM_ECRCRS_RESETVAL (0x0000u)
N/*----ECRCRS Tokens----*/
N#define CSL_MMCSD_MMCIM_ECRCRS_PROHIBIT (0x0000u)
N#define CSL_MMCSD_MMCIM_ECRCRS_PERMIT (0x0001u)
N
N#define CSL_MMCSD_MMCIM_ECRCRD_MASK (0x0040u)
N#define CSL_MMCSD_MMCIM_ECRCRD_SHIFT (0x0006u)
N#define CSL_MMCSD_MMCIM_ECRCRD_RESETVAL (0x0000u)
N/*----ECRCRD Tokens----*/
N#define CSL_MMCSD_MMCIM_ECRCRD_PROHIBIT (0x0000u)
N#define CSL_MMCSD_MMCIM_ECRCRD_PERMIT (0x0001u)
N
N#define CSL_MMCSD_MMCIM_ECRCWR_MASK (0x0020u)
N#define CSL_MMCSD_MMCIM_ECRCWR_SHIFT (0x0005u)
N#define CSL_MMCSD_MMCIM_ECRCWR_RESETVAL (0x0000u)
N/*----ECRCWR Tokens----*/
N#define CSL_MMCSD_MMCIM_ECRCWR_PROHIBIT (0x0000u)
N#define CSL_MMCSD_MMCIM_ECRCWR_PERMIT (0x0001u)
N
N#define CSL_MMCSD_MMCIM_ETOUTRS_MASK (0x0010u)
N#define CSL_MMCSD_MMCIM_ETOUTRS_SHIFT (0x0004u)
N#define CSL_MMCSD_MMCIM_ETOUTRS_RESETVAL (0x0000u)
N/*----ETOUTRS Tokens----*/
N#define CSL_MMCSD_MMCIM_ETOUTRS_PROHIBIT (0x0000u)
N#define CSL_MMCSD_MMCIM_ETOUTRS_PERMIT (0x0001u)
N
N#define CSL_MMCSD_MMCIM_ETOUTRD_MASK (0x0008u)
N#define CSL_MMCSD_MMCIM_ETOUTRD_SHIFT (0x0003u)
N#define CSL_MMCSD_MMCIM_ETOUTRD_RESETVAL (0x0000u)
N/*----ETOUTRD Tokens----*/
N#define CSL_MMCSD_MMCIM_ETOUTRD_PROHIBIT (0x0000u)
N#define CSL_MMCSD_MMCIM_ETOUTRD_PERMIT (0x0001u)
N
N#define CSL_MMCSD_MMCIM_ERSPDNE_MASK (0x0004u)
N#define CSL_MMCSD_MMCIM_ERSPDNE_SHIFT (0x0002u)
N#define CSL_MMCSD_MMCIM_ERSPDNE_RESETVAL (0x0000u)
N/*----ERSPDNE Tokens----*/
N#define CSL_MMCSD_MMCIM_ERSPDNE_PROHIBIT (0x0000u)
N#define CSL_MMCSD_MMCIM_ERSPDNE_PERMIT (0x0001u)
N
N#define CSL_MMCSD_MMCIM_EBSYDNE_MASK (0x0002u)
N#define CSL_MMCSD_MMCIM_EBSYDNE_SHIFT (0x0001u)
N#define CSL_MMCSD_MMCIM_EBSYDNE_RESETVAL (0x0000u)
N/*----EBSYDNE Tokens----*/
N#define CSL_MMCSD_MMCIM_EBSYDNE_PROHIBIT (0x0000u)
N#define CSL_MMCSD_MMCIM_EBSYDNE_PERMIT (0x0001u)
N
N#define CSL_MMCSD_MMCIM_EDATDNE_MASK (0x0001u)
N#define CSL_MMCSD_MMCIM_EDATDNE_SHIFT (0x0000u)
N#define CSL_MMCSD_MMCIM_EDATDNE_RESETVAL (0x0000u)
N/*----EDATDNE Tokens----*/
N#define CSL_MMCSD_MMCIM_EDATDNE_PROHIBIT (0x0000u)
N#define CSL_MMCSD_MMCIM_EDATDNE_PERMIT (0x0001u)
N
N#define CSL_MMCSD_MMCIM_RESETVAL (0x0000u)
N
N/* MMCTOR */
N
N
N#define CSL_MMCSD_MMCTOR_TOD_20_16_MASK (0x1F00u)
N#define CSL_MMCSD_MMCTOR_TOD_20_16_SHIFT (0x0008u)
N#define CSL_MMCSD_MMCTOR_TOD_20_16_RESETVAL (0x0000u)
N
N#define CSL_MMCSD_MMCTOR_TOR_MASK (0x00FFu)
N#define CSL_MMCSD_MMCTOR_TOR_SHIFT (0x0000u)
N#define CSL_MMCSD_MMCTOR_TOR_RESETVAL (0x0000u)
N
N#define CSL_MMCSD_MMCTOR_RESETVAL (0x0000u)
N
N/* MMCTODL */
N
N#define CSL_MMCSD_MMCTODL_TOD_15_0_MASK (0xFFFFu)
N#define CSL_MMCSD_MMCTODL_TOD_15_0_SHIFT (0x0000u)
N#define CSL_MMCSD_MMCTODL_TOD_15_0_RESETVAL (0x0000u)
N
N#define CSL_MMCSD_MMCTODL_RESETVAL (0x0000u)
N
N/* MMCTODH */
N
N
N#define CSL_MMCSD_MMCTODH_TOD_23_15_MASK (0x00FFu)
N#define CSL_MMCSD_MMCTODH_TOD_23_15_SHIFT (0x0000u)
N#define CSL_MMCSD_MMCTODH_TOD_23_15_RESETVAL (0x0000u)
N
N#define CSL_MMCSD_MMCTODH_RESETVAL (0x0000u)
N
N/* MMCBLEN */
N
N
N#define CSL_MMCSD_MMCBLEN_BLEN_MASK (0x0FFFu)
N#define CSL_MMCSD_MMCBLEN_BLEN_SHIFT (0x0000u)
N#define CSL_MMCSD_MMCBLEN_BLEN_RESETVAL (0x0200u)
N
N#define CSL_MMCSD_MMCBLEN_RESETVAL (0x0200u)
N
N/* MMCNBLK */
N
N#define CSL_MMCSD_MMCNBLK_NBLK_MASK (0xFFFFu)
N#define CSL_MMCSD_MMCNBLK_NBLK_SHIFT (0x0000u)
N#define CSL_MMCSD_MMCNBLK_NBLK_RESETVAL (0x0000u)
N
N#define CSL_MMCSD_MMCNBLK_RESETVAL (0x0000u)
N
N/* MMCNBLC */
N
N#define CSL_MMCSD_MMCNBLC_NBLC_MASK (0xFFFFu)
N#define CSL_MMCSD_MMCNBLC_NBLC_SHIFT (0x0000u)
N#define CSL_MMCSD_MMCNBLC_NBLC_RESETVAL (0xFFFFu)
N
N#define CSL_MMCSD_MMCNBLC_RESETVAL (0xFFFFu)
N
N/* MMCDRRL */
N
N#define CSL_MMCSD_MMCDRRL_DRRL_MASK (0xFFFFu)
N#define CSL_MMCSD_MMCDRRL_DRRL_SHIFT (0x0000u)
N#define CSL_MMCSD_MMCDRRL_DRRL_RESETVAL (0x0000u)
N
N#define CSL_MMCSD_MMCDRRL_RESETVAL (0x0000u)
N
N/* MMCDRRH */
N
N#define CSL_MMCSD_MMCDRRH_DRRH_MASK (0xFFFFu)
N#define CSL_MMCSD_MMCDRRH_DRRH_SHIFT (0x0000u)
N#define CSL_MMCSD_MMCDRRH_DRRH_RESETVAL (0x0000u)
N
N#define CSL_MMCSD_MMCDRRH_RESETVAL (0x0000u)
N
N/* MMCDXRL */
N
N#define CSL_MMCSD_MMCDXRL_DXRL_MASK (0xFFFFu)
N#define CSL_MMCSD_MMCDXRL_DXRL_SHIFT (0x0000u)
N#define CSL_MMCSD_MMCDXRL_DXRL_RESETVAL (0x0000u)
N
N#define CSL_MMCSD_MMCDXRL_RESETVAL (0x0000u)
N
N/* MMCDXRH */
N
N#define CSL_MMCSD_MMCDXRH_DXRH_MASK (0xFFFFu)
N#define CSL_MMCSD_MMCDXRH_DXRH_SHIFT (0x0000u)
N#define CSL_MMCSD_MMCDXRH_DXRH_RESETVAL (0x0000u)
N
N#define CSL_MMCSD_MMCDXRH_RESETVAL (0x0000u)
N
N/* MMCCMDL */
N
N#define CSL_MMCSD_MMCCMDL_DCLR_MASK (0x8000u)
N#define CSL_MMCSD_MMCCMDL_DCLR_SHIFT (0x000Fu)
N#define CSL_MMCSD_MMCCMDL_DCLR_RESETVAL (0x0000u)
N/*----DCLR Tokens----*/
N#define CSL_MMCSD_MMCCMDL_DCLR_NO (0x0000u)
N#define CSL_MMCSD_MMCCMDL_DCLR_CLEAR (0x0001u)
N
N#define CSL_MMCSD_MMCCMDL_INITCK_MASK (0x4000u)
N#define CSL_MMCSD_MMCCMDL_INITCK_SHIFT (0x000Eu)
N#define CSL_MMCSD_MMCCMDL_INITCK_RESETVAL (0x0000u)
N/*----INITCK Tokens----*/
N#define CSL_MMCSD_MMCCMDL_INITCK_NO (0x0000u)
N#define CSL_MMCSD_MMCCMDL_INITCK_INIT (0x0001u)
N
N#define CSL_MMCSD_MMCCMDL_WDATX_MASK (0x2000u)
N#define CSL_MMCSD_MMCCMDL_WDATX_SHIFT (0x000Du)
N#define CSL_MMCSD_MMCCMDL_WDATX_RESETVAL (0x0000u)
N/*----WDATX Tokens----*/
N#define CSL_MMCSD_MMCCMDL_WDATX_NO (0x0000u)
N#define CSL_MMCSD_MMCCMDL_WDATX_DATA (0x0001u)
N
N#define CSL_MMCSD_MMCCMDL_STRMTP_MASK (0x1000u)
N#define CSL_MMCSD_MMCCMDL_STRMTP_SHIFT (0x000Cu)
N#define CSL_MMCSD_MMCCMDL_STRMTP_RESETVAL (0x0000u)
N/*----STRMTP Tokens----*/
N#define CSL_MMCSD_MMCCMDL_STRMTP_BLOCK (0x0000u)
N#define CSL_MMCSD_MMCCMDL_STRMTP_STREAM (0x0001u)
N#define CSL_MMCSD_MMCCMDL_STRMTP_NO (0x0000u)
N
N#define CSL_MMCSD_MMCCMDL_DTRW_MASK (0x0800u)
N#define CSL_MMCSD_MMCCMDL_DTRW_SHIFT (0x000Bu)
N#define CSL_MMCSD_MMCCMDL_DTRW_RESETVAL (0x0000u)
N/*----DTRW Tokens----*/
N#define CSL_MMCSD_MMCCMDL_DTRW_READ (0x0000u)
N#define CSL_MMCSD_MMCCMDL_DTRW_WRITE (0x0001u)
N#define CSL_MMCSD_MMCCMDL_DTRW_NO (0x0000u)
N
N#define CSL_MMCSD_MMCCMDL_RSPFMT_MASK (0x0600u)
N#define CSL_MMCSD_MMCCMDL_RSPFMT_SHIFT (0x0009u)
N#define CSL_MMCSD_MMCCMDL_RSPFMT_RESETVAL (0x0000u)
N/*----RSPFMT Tokens----*/
N#define CSL_MMCSD_MMCCMDL_RSPFMT_NORSP (0x0000u)
N#define CSL_MMCSD_MMCCMDL_RSPFMT_R1 (0x0001u)
N#define CSL_MMCSD_MMCCMDL_RSPFMT_R2 (0x0002u)
N#define CSL_MMCSD_MMCCMDL_RSPFMT_R3 (0x0003u)
N#define CSL_MMCSD_MMCCMDL_RSPFMT_R4 (0x0001u)
N#define CSL_MMCSD_MMCCMDL_RSPFMT_R5 (0x0001u)
N#define CSL_MMCSD_MMCCMDL_RSPFMT_R6 (0x0001u)
N
N#define CSL_MMCSD_MMCCMDL_BSYEXP_MASK (0x0100u)
N#define CSL_MMCSD_MMCCMDL_BSYEXP_SHIFT (0x0008u)
N#define CSL_MMCSD_MMCCMDL_BSYEXP_RESETVAL (0x0000u)
N/*----BSYEXP Tokens----*/
N#define CSL_MMCSD_MMCCMDL_BSYEXP_NO (0x0000u)
N#define CSL_MMCSD_MMCCMDL_BSYEXP_BUSY (0x0001u)
N
N#define CSL_MMCSD_MMCCMDL_PPLEN_MASK (0x0080u)
N#define CSL_MMCSD_MMCCMDL_PPLEN_SHIFT (0x0007u)
N#define CSL_MMCSD_MMCCMDL_PPLEN_RESETVAL (0x0000u)
N/*----PPLEN Tokens----*/
N#define CSL_MMCSD_MMCCMDL_PPLEN_OD (0x0000u)
N#define CSL_MMCSD_MMCCMDL_PPLEN_PP (0x0001u)
N
N
N#define CSL_MMCSD_MMCCMDL_CMD_MASK (0x003Fu)
N#define CSL_MMCSD_MMCCMDL_CMD_SHIFT (0x0000u)
N#define CSL_MMCSD_MMCCMDL_CMD_RESETVAL (0x0000u)
N
N#define CSL_MMCSD_MMCCMDL_RESETVAL (0x0000u)
N
N/* MMCCMDH */
N
N
N#define CSL_MMCSD_MMCCMDH_DMATRIG_MASK (0x0001u)
N#define CSL_MMCSD_MMCCMDH_DMATRIG_SHIFT (0x0000u)
N#define CSL_MMCSD_MMCCMDH_DMATRIG_RESETVAL (0x0000u)
N/*----DMATRIG Tokens----*/
N#define CSL_MMCSD_MMCCMDH_DMATRIG_NO (0x0000u)
N
N#define CSL_MMCSD_MMCCMDH_RESETVAL (0x0000u)
N
N/* MMCARGL */
N
N#define CSL_MMCSD_MMCARGL_ARGL_MASK (0xFFFFu)
N#define CSL_MMCSD_MMCARGL_ARGL_SHIFT (0x0000u)
N#define CSL_MMCSD_MMCARGL_ARGL_RESETVAL (0x0000u)
N
N#define CSL_MMCSD_MMCARGL_RESETVAL (0x0000u)
N
N/* MMCARGH */
N
N#define CSL_MMCSD_MMCARGH_ARGH_MASK (0xFFFFu)
N#define CSL_MMCSD_MMCARGH_ARGH_SHIFT (0x0000u)
N#define CSL_MMCSD_MMCARGH_ARGH_RESETVAL (0x0000u)
N
N#define CSL_MMCSD_MMCARGH_RESETVAL (0x0000u)
N
N/* MMCRSP0 */
N
N#define CSL_MMCSD_MMCRSP0_MMCRSP0_MASK (0xFFFFu)
N#define CSL_MMCSD_MMCRSP0_MMCRSP0_SHIFT (0x0000u)
N#define CSL_MMCSD_MMCRSP0_MMCRSP0_RESETVAL (0x0000u)
N
N#define CSL_MMCSD_MMCRSP0_RESETVAL (0x0000u)
N
N/* MMCRSP1 */
N
N#define CSL_MMCSD_MMCRSP1_MMCRSP1_MASK (0xFFFFu)
N#define CSL_MMCSD_MMCRSP1_MMCRSP1_SHIFT (0x0000u)
N#define CSL_MMCSD_MMCRSP1_MMCRSP1_RESETVAL (0x0000u)
N
N#define CSL_MMCSD_MMCRSP1_RESETVAL (0x0000u)
N
N/* MMCRSP2 */
N
N#define CSL_MMCSD_MMCRSP2_MMCRSP2_MASK (0xFFFFu)
N#define CSL_MMCSD_MMCRSP2_MMCRSP2_SHIFT (0x0000u)
N#define CSL_MMCSD_MMCRSP2_MMCRSP2_RESETVAL (0x0000u)
N
N#define CSL_MMCSD_MMCRSP2_RESETVAL (0x0000u)
N
N/* MMCRSP3 */
N
N#define CSL_MMCSD_MMCRSP3_MMCRSP3_MASK (0xFFFFu)
N#define CSL_MMCSD_MMCRSP3_MMCRSP3_SHIFT (0x0000u)
N#define CSL_MMCSD_MMCRSP3_MMCRSP3_RESETVAL (0x0000u)
N
N#define CSL_MMCSD_MMCRSP3_RESETVAL (0x0000u)
N
N/* MMCRSP4 */
N
N#define CSL_MMCSD_MMCRSP4_MMCRSP4_MASK (0xFFFFu)
N#define CSL_MMCSD_MMCRSP4_MMCRSP4_SHIFT (0x0000u)
N#define CSL_MMCSD_MMCRSP4_MMCRSP4_RESETVAL (0x0000u)
N
N#define CSL_MMCSD_MMCRSP4_RESETVAL (0x0000u)
N
N/* MMCRSP5 */
N
N#define CSL_MMCSD_MMCRSP5_MMCRSP5_MASK (0xFFFFu)
N#define CSL_MMCSD_MMCRSP5_MMCRSP5_SHIFT (0x0000u)
N#define CSL_MMCSD_MMCRSP5_MMCRSP5_RESETVAL (0x0000u)
N
N#define CSL_MMCSD_MMCRSP5_RESETVAL (0x0000u)
N
N/* MMCRSP6 */
N
N#define CSL_MMCSD_MMCRSP6_MMCRSP6_MASK (0xFFFFu)
N#define CSL_MMCSD_MMCRSP6_MMCRSP6_SHIFT (0x0000u)
N#define CSL_MMCSD_MMCRSP6_MMCRSP6_RESETVAL (0x0000u)
N
N#define CSL_MMCSD_MMCRSP6_RESETVAL (0x0000u)
N
N/* MMCRSP7 */
N
N#define CSL_MMCSD_MMCRSP7_MMCRSP7_MASK (0xFFFFu)
N#define CSL_MMCSD_MMCRSP7_MMCRSP7_SHIFT (0x0000u)
N#define CSL_MMCSD_MMCRSP7_MMCRSP7_RESETVAL (0x0000u)
N
N#define CSL_MMCSD_MMCRSP7_RESETVAL (0x0000u)
N
N/* MMCDRSP */
N
N
N#define CSL_MMCSD_MMCDRSP_DRSP_MASK (0x00FFu)
N#define CSL_MMCSD_MMCDRSP_DRSP_SHIFT (0x0000u)
N#define CSL_MMCSD_MMCDRSP_DRSP_RESETVAL (0x0000u)
N
N#define CSL_MMCSD_MMCDRSP_RESETVAL (0x0000u)
N
N/* MMCETOK */
N
N
N#define CSL_MMCSD_MMCETOK_RESETVAL (0x0000u)
N
N/* MMCCIDX */
N
N
N#define CSL_MMCSD_MMCCIDX_STRT_MASK (0x0080u)
N#define CSL_MMCSD_MMCCIDX_STRT_SHIFT (0x0007u)
N#define CSL_MMCSD_MMCCIDX_STRT_RESETVAL (0x0000u)
N
N#define CSL_MMCSD_MMCCIDX_XMIT_MASK (0x0040u)
N#define CSL_MMCSD_MMCCIDX_XMIT_SHIFT (0x0006u)
N#define CSL_MMCSD_MMCCIDX_XMIT_RESETVAL (0x0000u)
N
N#define CSL_MMCSD_MMCCIDX_CIDX_MASK (0x003Fu)
N#define CSL_MMCSD_MMCCIDX_CIDX_SHIFT (0x0000u)
N#define CSL_MMCSD_MMCCIDX_CIDX_RESETVAL (0x0000u)
N
N#define CSL_MMCSD_MMCCIDX_RESETVAL (0x0000u)
N
N/* MMCCKC */
N
N
N#define CSL_MMCSD_MMCCKC_CPHA_MASK (0x00FFu)
N#define CSL_MMCSD_MMCCKC_CPHA_SHIFT (0x0000u)
N#define CSL_MMCSD_MMCCKC_CPHA_RESETVAL (0x0000u)
N
N#define CSL_MMCSD_MMCCKC_RESETVAL (0x0000u)
N
N/* MMCTORC */
N
N
N#define CSL_MMCSD_MMCTORC_TODC_20_16_MASK (0x1F00u)
N#define CSL_MMCSD_MMCTORC_TODC_20_16_SHIFT (0x0008u)
N#define CSL_MMCSD_MMCTORC_TODC_20_16_RESETVAL (0x0000u)
N
N#define CSL_MMCSD_MMCTORC_TORC_MASK (0x00FFu)
N#define CSL_MMCSD_MMCTORC_TORC_SHIFT (0x0000u)
N#define CSL_MMCSD_MMCTORC_TORC_RESETVAL (0x0000u)
N
N#define CSL_MMCSD_MMCTORC_RESETVAL (0x0000u)
N
N/* MMCTODC */
N
N#define CSL_MMCSD_MMCTODC_TODC_15_0_MASK (0xFFFFu)
N#define CSL_MMCSD_MMCTODC_TODC_15_0_SHIFT (0x0000u)
N#define CSL_MMCSD_MMCTODC_TODC_15_0_RESETVAL (0x0000u)
N
N#define CSL_MMCSD_MMCTODC_RESETVAL (0x0000u)
N
N/* MMCBLNC */
N
N
N#define CSL_MMCSD_MMCBLNC_BLNC_MASK (0x00FFu)
N#define CSL_MMCSD_MMCBLNC_BLNC_SHIFT (0x0000u)
N#define CSL_MMCSD_MMCBLNC_BLNC_RESETVAL (0x0001u)
N
N#define CSL_MMCSD_MMCBLNC_RESETVAL (0x0001u)
N
N/* SDIOCTL */
N
N
N#define CSL_MMCSD_SDIOCTL_RDWTCR_MASK (0x0002u)
N#define CSL_MMCSD_SDIOCTL_RDWTCR_SHIFT (0x0001u)
N#define CSL_MMCSD_SDIOCTL_RDWTCR_RESETVAL (0x0000u)
N/*----RDWTCR Tokens----*/
N#define CSL_MMCSD_SDIOCTL_RDWTCR_ENABLE (0x0001u)
N
N#define CSL_MMCSD_SDIOCTL_RDWTRQ_MASK (0x0001u)
N#define CSL_MMCSD_SDIOCTL_RDWTRQ_SHIFT (0x0000u)
N#define CSL_MMCSD_SDIOCTL_RDWTRQ_RESETVAL (0x0000u)
N
N#define CSL_MMCSD_SDIOCTL_RESETVAL (0x0000u)
N
N/* SDIOST0 */
N
N
N#define CSL_MMCSD_SDIOST0_RDWTST_MASK (0x0004u)
N#define CSL_MMCSD_SDIOST0_RDWTST_SHIFT (0x0002u)
N#define CSL_MMCSD_SDIOST0_RDWTST_RESETVAL (0x0000u)
N
N#define CSL_MMCSD_SDIOST0_INTPRD_MASK (0x0002u)
N#define CSL_MMCSD_SDIOST0_INTPRD_SHIFT (0x0001u)
N#define CSL_MMCSD_SDIOST0_INTPRD_RESETVAL (0x0000u)
N
N#define CSL_MMCSD_SDIOST0_DAT1_MASK (0x0001u)
N#define CSL_MMCSD_SDIOST0_DAT1_SHIFT (0x0000u)
N#define CSL_MMCSD_SDIOST0_DAT1_RESETVAL (0x0001u)
N
N#define CSL_MMCSD_SDIOST0_RESETVAL (0x0001u)
N
N/* SDIOIEN */
N
N
N#define CSL_MMCSD_SDIOIEN_RWSEN_MASK (0x0002u)
N#define CSL_MMCSD_SDIOIEN_RWSEN_SHIFT (0x0001u)
N#define CSL_MMCSD_SDIOIEN_RWSEN_RESETVAL (0x0000u)
N/*----RWSEN Tokens----*/
N#define CSL_MMCSD_SDIOIEN_RWSEN_DISABLE (0x0000u)
N#define CSL_MMCSD_SDIOIEN_RWSEN_ENABLE (0x0001u)
N
N#define CSL_MMCSD_SDIOIEN_IOINTEN_MASK (0x0001u)
N#define CSL_MMCSD_SDIOIEN_IOINTEN_SHIFT (0x0000u)
N#define CSL_MMCSD_SDIOIEN_IOINTEN_RESETVAL (0x0000u)
N/*----IOINTEN Tokens----*/
N#define CSL_MMCSD_SDIOIEN_IOINTEN_DISABLE (0x0000u)
N#define CSL_MMCSD_SDIOIEN_IOINTEN_ENABLE (0x0001u)
N
N#define CSL_MMCSD_SDIOIEN_RESETVAL (0x0000u)
N
N/* SDIOIST  */
N
N
N#define CSL_MMCSD_SDIOIST_RWS_MASK (0x0002u)
N#define CSL_MMCSD_SDIOIST_RWS_SHIFT (0x0001u)
N#define CSL_MMCSD_SDIOIST_RWS_RESETVAL (0x0000u)
N
N#define CSL_MMCSD_SDIOIST_IOINT_MASK (0x0001u)
N#define CSL_MMCSD_SDIOIST_IOINT_SHIFT (0x0000u)
N#define CSL_MMCSD_SDIOIST_IOINT_RESETVAL (0x0000u)
N
N#define CSL_MMCSD_SDIOIST_RESETVAL (0x0000u)
N
N/* MMCFIFOCTL */
N
N
N#define CSL_MMCSD_MMCFIFOCTL_ACCWD_MASK (0x0018u)
N#define CSL_MMCSD_MMCFIFOCTL_ACCWD_SHIFT (0x0003u)
N#define CSL_MMCSD_MMCFIFOCTL_ACCWD_RESETVAL (0x0000u)
N/*----ACCWD Tokens----*/
N#define CSL_MMCSD_MMCFIFOCTL_ACCWD_4BYTES (0x0000u)
N#define CSL_MMCSD_MMCFIFOCTL_ACCWD_3BYTES (0x0001u)
N#define CSL_MMCSD_MMCFIFOCTL_ACCWD_2BYTES (0x0002u)
N#define CSL_MMCSD_MMCFIFOCTL_ACCWD_1BYTE (0x0003u)
N
N#define CSL_MMCSD_MMCFIFOCTL_FIFOLEV_MASK (0x0004u)
N#define CSL_MMCSD_MMCFIFOCTL_FIFOLEV_SHIFT (0x0002u)
N#define CSL_MMCSD_MMCFIFOCTL_FIFOLEV_RESETVAL (0x0000u)
N/*----FIFOLEV Tokens----*/
N#define CSL_MMCSD_MMCFIFOCTL_FIFOLEV_128BIT (0x0000u)
N#define CSL_MMCSD_MMCFIFOCTL_FIFOLEV_256BIT (0x0001u)
N
N#define CSL_MMCSD_MMCFIFOCTL_FIFODIR_MASK (0x0002u)
N#define CSL_MMCSD_MMCFIFOCTL_FIFODIR_SHIFT (0x0001u)
N#define CSL_MMCSD_MMCFIFOCTL_FIFODIR_RESETVAL (0x0000u)
N/*----FIFODIR Tokens----*/
N#define CSL_MMCSD_MMCFIFOCTL_FIFODIR_READ (0x0000u)
N#define CSL_MMCSD_MMCFIFOCTL_FIFODIR_WRITE (0x0001u)
N
N#define CSL_MMCSD_MMCFIFOCTL_FIFORST_MASK (0x0001u)
N#define CSL_MMCSD_MMCFIFOCTL_FIFORST_SHIFT (0x0000u)
N#define CSL_MMCSD_MMCFIFOCTL_FIFORST_RESETVAL (0x0000u)
N/*----FIFORST Tokens----*/
N#define CSL_MMCSD_MMCFIFOCTL_FIFORST_RESET (0x0001u)
N
N#define CSL_MMCSD_MMCFIFOCTL_RESETVAL (0x0000u)
N
N#endif
L 62 "../common_inc/corazon.h" 2
N#include "cslr_lcdc_001.h"
L 1 "..\common_inc\cslr_lcdc_001.h" 1
N/*****************************************************************************
N * File Name : cslr_lcdc_001.h 
N *
N * Brief	 : This file contains the Register Desciptions for LCDC
N *
N * Copyright (C) 2009 -2010 Texas Instruments Incorporated - http://www.ti.com/ 
N * 
N * 
N *  Redistribution and use in source and binary forms, with or without 
N *  modification, are permitted provided that the following conditions 
N *  are met:
N * 
N *    Redistributions of source code must retain the above copyright 
N *    notice, this list of conditions and the following disclaimer.
N * 
N *    Redistributions in binary form must reproduce the above copyright
N *    notice, this list of conditions and the following disclaimer in the 
N *    documentation and/or other materials provided with the   
N *    distribution.
N *
N *    Neither the name of Texas Instruments Incorporated nor the names of
N *    its contributors may be used to endorse or promote products derived
N *    from this software without specific prior written permission.
N *
N *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
N *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
N *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
N *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
N *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
N *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
N *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
N *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
N *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
N *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
N *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
N *
N******************************************************************************/
N#ifndef _CSLR_LCDC_1_H_
N#define _CSLR_LCDC_1_H_
N
N#include <cslr.h>
N
N/* Minimum unit = 2 bytes */
N
N//#include <tistdtypes.h>
N
N/**************************************************************************\
N* Register Overlay Structure
X
N\**************************************************************************/
Ntypedef struct  {
N    volatile Uint16 LCDRRL;
N    volatile Uint16 LCDRRM;
N    volatile Uint16 RSVD0[2];
N    volatile Uint16 LCDCRL;
N    volatile Uint16 LCDCRM;
N    volatile Uint16 RSVD1[2];
N    volatile Uint16 LCDSRL;
N    volatile Uint16 LCDSRM;
N    volatile Uint16 RSVD2[2];
N    volatile Uint16 LCDLIDDCRL;
N    volatile Uint16 LCDLIDDCRM;
N    volatile Uint16 RSVD3[2];
N    volatile Uint16 LCDLIDDCS0CONFIGL;
N    volatile Uint16 LCDLIDDCS0CONFIGM;
N    volatile Uint16 RSVD4[2];
N    volatile Uint16 LCDLIDDCS0ADDRL;
N    volatile Uint16 LCDLIDDCS0ADDRM;
N    volatile Uint16 RSVD5[2];
N    volatile Uint16 LCDLIDDCS0DATAL;
N    volatile Uint16 LCDLIDDCS0DATAM;
N    volatile Uint16 RSVD6[2];
N    volatile Uint16 LCDLIDDCS1CONFIGL;
N    volatile Uint16 LCDLIDDCS1CONFIGM;
N    volatile Uint16 RSVD7[2];
N    volatile Uint16 LCDLIDDCS1ADDRL;
N    volatile Uint16 LCDLIDDCS1ADDRM;
N    volatile Uint16 RSVD8[2];
N    volatile Uint16 LCDLIDDCS1DATAL;
N    volatile Uint16 LCDLIDDCS1DATAM;
N    volatile Uint16 RSVD9[2];
N    volatile Uint16 LCDRASTCRL;
N    volatile Uint16 LCDRASTCRM;
N    volatile Uint16 RSVD10[2];
N    volatile Uint16 LCDRASTT0RL;
N    volatile Uint16 LCDRASTT0RM;
N    volatile Uint16 RSVD11[2];
N    volatile Uint16 LCDRASTT1RL;
N    volatile Uint16 LCDRASTT1RM;
N    volatile Uint16 RSVD12[2];
N    volatile Uint16 LCDRASTT2RL;
N    volatile Uint16 LCDRASTT2RM;
N    volatile Uint16 RSVD13[2];
N    volatile Uint16 LCDRASTSUBPANDISPL;
N    volatile Uint16 LCDRASTSUBPANDISPM;
N    volatile Uint16 RSVD14[6];
N    volatile Uint16 LCDDMACRL;
N    volatile Uint16 LCDDMACRM;
N    volatile Uint16 RSVD15[2];
N    volatile Uint16 LCDDMAFB0BARL;
N    volatile Uint16 LCDDMAFB0BARM;
N    volatile Uint16 RSVD16[2];
N    volatile Uint16 LCDDMAFB0CARL;
N    volatile Uint16 LCDDMAFB0CARM;
N    volatile Uint16 RSVD17[2];
N    volatile Uint16 LCDDMAFB1BARL;
N    volatile Uint16 LCDDMAFB1BARM;
N    volatile Uint16 RSVD18[2];
N    volatile Uint16 LCDDMAFB1CARL;
N    volatile Uint16 LCDDMAFB1CARM;
N} CSL_LcdcRegs;
N
N/**************************************************************************\
N* Field Definition Macros
X
N\**************************************************************************/
N
N/* LCDRRL */
N
N#define CSL_LCDC_LCDRRL_REVMIN_MASK (0xFFFFu)
N#define CSL_LCDC_LCDRRL_REVMIN_SHIFT (0x0000u)
N#define CSL_LCDC_LCDRRL_REVMIN_RESETVAL (0x0000u)
N
N#define CSL_LCDC_LCDRRL_RESETVAL (0x0000u)
N
N/* LCDRRM */
N
N#define CSL_LCDC_LCDRRM_REVMAJ_MASK (0xFFFFu)
N#define CSL_LCDC_LCDRRM_REVMAJ_SHIFT (0x0000u)
N#define CSL_LCDC_LCDRRM_REVMAJ_RESETVAL (0x0001u)
N
N#define CSL_LCDC_LCDRRM_RESETVAL (0x0001u)
N
N/* LCDCRL */
N
N#define CSL_LCDC_LCDCRL_CLKDIV_MASK (0xFF00u)
N#define CSL_LCDC_LCDCRL_CLKDIV_SHIFT (0x0008u)
N#define CSL_LCDC_LCDCRL_CLKDIV_RESETVAL (0x0000u)
N
N
N#define CSL_LCDC_LCDCRL_MODESEL_MASK (0x0001u)
N#define CSL_LCDC_LCDCRL_MODESEL_SHIFT (0x0000u)
N#define CSL_LCDC_LCDCRL_MODESEL_RESETVAL (0x0000u)
N/*----MODESEL Tokens----*/
N#define CSL_LCDC_LCDCRL_MODESEL_LIDD (0x0000u)
N
N#define CSL_LCDC_LCDCRL_RESETVAL (0x0000u)
N
N/* LCDCRM */
N
N
N#define CSL_LCDC_LCDCRM_RESETVAL (0x0000u)
N
N/* LCDSRL */
N
N
N#define CSL_LCDC_LCDSRL_EOF1_MASK (0x0200u)
N#define CSL_LCDC_LCDSRL_EOF1_SHIFT (0x0009u)
N#define CSL_LCDC_LCDSRL_EOF1_RESETVAL (0x0000u)
N
N#define CSL_LCDC_LCDSRL_EOF0_MASK (0x0100u)
N#define CSL_LCDC_LCDSRL_EOF0_SHIFT (0x0008u)
N#define CSL_LCDC_LCDSRL_EOF0_RESETVAL (0x0000u)
N
N
N#define CSL_LCDC_LCDSRL_PL_MASK (0x0040u)
N#define CSL_LCDC_LCDSRL_PL_SHIFT (0x0006u)
N#define CSL_LCDC_LCDSRL_PL_RESETVAL (0x0000u)
N
N#define CSL_LCDC_LCDSRL_FUF_MASK (0x0020u)
N#define CSL_LCDC_LCDSRL_FUF_SHIFT (0x0005u)
N#define CSL_LCDC_LCDSRL_FUF_RESETVAL (0x0000u)
N
N
N#define CSL_LCDC_LCDSRL_ABC_MASK (0x0008u)
N#define CSL_LCDC_LCDSRL_ABC_SHIFT (0x0003u)
N#define CSL_LCDC_LCDSRL_ABC_RESETVAL (0x0000u)
N
N#define CSL_LCDC_LCDSRL_SYNC_MASK (0x0004u)
N#define CSL_LCDC_LCDSRL_SYNC_SHIFT (0x0002u)
N#define CSL_LCDC_LCDSRL_SYNC_RESETVAL (0x0000u)
N
N
N#define CSL_LCDC_LCDSRL_DONE_MASK (0x0001u)
N#define CSL_LCDC_LCDSRL_DONE_SHIFT (0x0000u)
N#define CSL_LCDC_LCDSRL_DONE_RESETVAL (0x0000u)
N
N#define CSL_LCDC_LCDSRL_RESETVAL (0x0000u)
N
N/* LCDSRM */
N
N
N#define CSL_LCDC_LCDSRM_RESETVAL (0x0000u)
N
N/* LCDLIDDCRL */
N
N
N#define CSL_LCDC_LCDLIDDCRL_DONE_INT_EN_MASK (0x0400u)
N#define CSL_LCDC_LCDLIDDCRL_DONE_INT_EN_SHIFT (0x000Au)
N#define CSL_LCDC_LCDLIDDCRL_DONE_INT_EN_RESETVAL (0x0000u)
N
N#define CSL_LCDC_LCDLIDDCRL_DMA_CS0_CS1_MASK (0x0200u)
N#define CSL_LCDC_LCDLIDDCRL_DMA_CS0_CS1_SHIFT (0x0009u)
N#define CSL_LCDC_LCDLIDDCRL_DMA_CS0_CS1_RESETVAL (0x0000u)
N
N#define CSL_LCDC_LCDLIDDCRL_LIDD_DMA_EN_MASK (0x0100u)
N#define CSL_LCDC_LCDLIDDCRL_LIDD_DMA_EN_SHIFT (0x0008u)
N#define CSL_LCDC_LCDLIDDCRL_LIDD_DMA_EN_RESETVAL (0x0000u)
N
N#define CSL_LCDC_LCDLIDDCRL_CS1_E1_POL_MASK (0x0080u)
N#define CSL_LCDC_LCDLIDDCRL_CS1_E1_POL_SHIFT (0x0007u)
N#define CSL_LCDC_LCDLIDDCRL_CS1_E1_POL_RESETVAL (0x0000u)
N
N#define CSL_LCDC_LCDLIDDCRL_CS0_E0_POL_MASK (0x0040u)
N#define CSL_LCDC_LCDLIDDCRL_CS0_E0_POL_SHIFT (0x0006u)
N#define CSL_LCDC_LCDLIDDCRL_CS0_E0_POL_RESETVAL (0x0000u)
N
N#define CSL_LCDC_LCDLIDDCRL_WS_DIR_POL_MASK (0x0020u)
N#define CSL_LCDC_LCDLIDDCRL_WS_DIR_POL_SHIFT (0x0005u)
N#define CSL_LCDC_LCDLIDDCRL_WS_DIR_POL_RESETVAL (0x0000u)
N
N#define CSL_LCDC_LCDLIDDCRL_RS_EN_POL_MASK (0x0010u)
N#define CSL_LCDC_LCDLIDDCRL_RS_EN_POL_SHIFT (0x0004u)
N#define CSL_LCDC_LCDLIDDCRL_RS_EN_POL_RESETVAL (0x0000u)
N
N#define CSL_LCDC_LCDLIDDCRL_ALEPOL_MASK (0x0008u)
N#define CSL_LCDC_LCDLIDDCRL_ALEPOL_SHIFT (0x0003u)
N#define CSL_LCDC_LCDLIDDCRL_ALEPOL_RESETVAL (0x0000u)
N
N#define CSL_LCDC_LCDLIDDCRL_LIDD_MODE_SEL_MASK (0x0007u)
N#define CSL_LCDC_LCDLIDDCRL_LIDD_MODE_SEL_SHIFT (0x0000u)
N#define CSL_LCDC_LCDLIDDCRL_LIDD_MODE_SEL_RESETVAL (0x0000u)
N/*----LIDD_MODE_SEL Tokens----*/
N#define CSL_LCDC_LCDLIDDCRL_LIDD_MODE_SEL_SYNC_MPU68 (0x0000u)
N#define CSL_LCDC_LCDLIDDCRL_LIDD_MODE_SEL_HITACHI (0x0004u)
N
N#define CSL_LCDC_LCDLIDDCRL_RESETVAL (0x0000u)
N
N/* LCDLIDDCRM */
N
N
N#define CSL_LCDC_LCDLIDDCRM_RESETVAL (0x0000u)
N
N/* LCDLIDDCS0CONFIGL */
N
N#define CSL_LCDC_LCDLIDDCS0CONFIGL_R_SU_MASK (0xF000u)
N#define CSL_LCDC_LCDLIDDCS0CONFIGL_R_SU_SHIFT (0x000Cu)
N#define CSL_LCDC_LCDLIDDCS0CONFIGL_R_SU_RESETVAL (0x0000u)
N
N#define CSL_LCDC_LCDLIDDCS0CONFIGL_R_STROBE_MASK (0x0FC0u)
N#define CSL_LCDC_LCDLIDDCS0CONFIGL_R_STROBE_SHIFT (0x0006u)
N#define CSL_LCDC_LCDLIDDCS0CONFIGL_R_STROBE_RESETVAL (0x0001u)
N
N#define CSL_LCDC_LCDLIDDCS0CONFIGL_R_HOLD_MASK (0x003Cu)
N#define CSL_LCDC_LCDLIDDCS0CONFIGL_R_HOLD_SHIFT (0x0002u)
N#define CSL_LCDC_LCDLIDDCS0CONFIGL_R_HOLD_RESETVAL (0x0001u)
N
N#define CSL_LCDC_LCDLIDDCS0CONFIGL_TA_MASK (0x0003u)
N#define CSL_LCDC_LCDLIDDCS0CONFIGL_TA_SHIFT (0x0000u)
N#define CSL_LCDC_LCDLIDDCS0CONFIGL_TA_RESETVAL (0x0000u)
N
N#define CSL_LCDC_LCDLIDDCS0CONFIGL_RESETVAL (0x0044u)
N
N/* LCDLIDDCS0CONFIGM */
N
N#define CSL_LCDC_LCDLIDDCS0CONFIGM_W_SU_MASK (0xF800u)
N#define CSL_LCDC_LCDLIDDCS0CONFIGM_W_SU_SHIFT (0x000Bu)
N#define CSL_LCDC_LCDLIDDCS0CONFIGM_W_SU_RESETVAL (0x0000u)
N
N#define CSL_LCDC_LCDLIDDCS0CONFIGM_W_STROBE_MASK (0x07E0u)
N#define CSL_LCDC_LCDLIDDCS0CONFIGM_W_STROBE_SHIFT (0x0005u)
N#define CSL_LCDC_LCDLIDDCS0CONFIGM_W_STROBE_RESETVAL (0x0001u)
N
N#define CSL_LCDC_LCDLIDDCS0CONFIGM_W_HOLD_MASK (0x001Eu)
N#define CSL_LCDC_LCDLIDDCS0CONFIGM_W_HOLD_SHIFT (0x0001u)
N#define CSL_LCDC_LCDLIDDCS0CONFIGM_W_HOLD_RESETVAL (0x0001u)
N
N#define CSL_LCDC_LCDLIDDCS0CONFIGM_R_SU_MASK (0x0001u)
N#define CSL_LCDC_LCDLIDDCS0CONFIGM_R_SU_SHIFT (0x0000u)
N#define CSL_LCDC_LCDLIDDCS0CONFIGM_R_SU_RESETVAL (0x0000u)
N
N#define CSL_LCDC_LCDLIDDCS0CONFIGM_RESETVAL (0x0022u)
N
N/* LCDLIDDCS0ADDRL */
N
N#define CSL_LCDC_LCDLIDDCS0ADDRL_ADR_INDX_MASK (0xFFFFu)
N#define CSL_LCDC_LCDLIDDCS0ADDRL_ADR_INDX_SHIFT (0x0000u)
N#define CSL_LCDC_LCDLIDDCS0ADDRL_ADR_INDX_RESETVAL (0x0000u)
N
N#define CSL_LCDC_LCDLIDDCS0ADDRL_RESETVAL (0x0000u)
N
N/* LCDLIDDCS0ADDRM */
N
N
N#define CSL_LCDC_LCDLIDDCS0ADDRM_RESETVAL (0x0000u)
N
N/* LCDLIDDCS0DATAL */
N
N#define CSL_LCDC_LCDLIDDCS0DATAL_DATA_MASK (0xFFFFu)
N#define CSL_LCDC_LCDLIDDCS0DATAL_DATA_SHIFT (0x0000u)
N#define CSL_LCDC_LCDLIDDCS0DATAL_DATA_RESETVAL (0x0000u)
N
N#define CSL_LCDC_LCDLIDDCS0DATAL_RESETVAL (0x0000u)
N
N/* LCDLIDDCS0DATAM */
N
N
N#define CSL_LCDC_LCDLIDDCS0DATAM_RESETVAL (0x0000u)
N
N/* LCDLIDDCS1CONFIGL */
N
N#define CSL_LCDC_LCDLIDDCS1CONFIGL_R_SU_MASK (0xF000u)
N#define CSL_LCDC_LCDLIDDCS1CONFIGL_R_SU_SHIFT (0x000Cu)
N#define CSL_LCDC_LCDLIDDCS1CONFIGL_R_SU_RESETVAL (0x0000u)
N
N#define CSL_LCDC_LCDLIDDCS1CONFIGL_R_STROBE_MASK (0x0FC0u)
N#define CSL_LCDC_LCDLIDDCS1CONFIGL_R_STROBE_SHIFT (0x0006u)
N#define CSL_LCDC_LCDLIDDCS1CONFIGL_R_STROBE_RESETVAL (0x0001u)
N
N#define CSL_LCDC_LCDLIDDCS1CONFIGL_R_HOLD_MASK (0x003Cu)
N#define CSL_LCDC_LCDLIDDCS1CONFIGL_R_HOLD_SHIFT (0x0002u)
N#define CSL_LCDC_LCDLIDDCS1CONFIGL_R_HOLD_RESETVAL (0x0001u)
N
N#define CSL_LCDC_LCDLIDDCS1CONFIGL_TA_MASK (0x0003u)
N#define CSL_LCDC_LCDLIDDCS1CONFIGL_TA_SHIFT (0x0000u)
N#define CSL_LCDC_LCDLIDDCS1CONFIGL_TA_RESETVAL (0x0000u)
N
N#define CSL_LCDC_LCDLIDDCS1CONFIGL_RESETVAL (0x0044u)
N
N/* LCDLIDDCS1CONFIGM */
N
N#define CSL_LCDC_LCDLIDDCS1CONFIGM_W_SU_MASK (0xF800u)
N#define CSL_LCDC_LCDLIDDCS1CONFIGM_W_SU_SHIFT (0x000Bu)
N#define CSL_LCDC_LCDLIDDCS1CONFIGM_W_SU_RESETVAL (0x0000u)
N
N#define CSL_LCDC_LCDLIDDCS1CONFIGM_W_STROBE_MASK (0x07E0u)
N#define CSL_LCDC_LCDLIDDCS1CONFIGM_W_STROBE_SHIFT (0x0005u)
N#define CSL_LCDC_LCDLIDDCS1CONFIGM_W_STROBE_RESETVAL (0x0001u)
N
N#define CSL_LCDC_LCDLIDDCS1CONFIGM_W_HOLD_MASK (0x001Eu)
N#define CSL_LCDC_LCDLIDDCS1CONFIGM_W_HOLD_SHIFT (0x0001u)
N#define CSL_LCDC_LCDLIDDCS1CONFIGM_W_HOLD_RESETVAL (0x0001u)
N
N#define CSL_LCDC_LCDLIDDCS1CONFIGM_R_SU_MASK (0x0001u)
N#define CSL_LCDC_LCDLIDDCS1CONFIGM_R_SU_SHIFT (0x0000u)
N#define CSL_LCDC_LCDLIDDCS1CONFIGM_R_SU_RESETVAL (0x0000u)
N
N#define CSL_LCDC_LCDLIDDCS1CONFIGM_RESETVAL (0x0022u)
N
N/* LCDLIDDCS1ADDRL */
N
N#define CSL_LCDC_LCDLIDDCS1ADDRL_ADR_INDX_MASK (0xFFFFu)
N#define CSL_LCDC_LCDLIDDCS1ADDRL_ADR_INDX_SHIFT (0x0000u)
N#define CSL_LCDC_LCDLIDDCS1ADDRL_ADR_INDX_RESETVAL (0x0000u)
N
N#define CSL_LCDC_LCDLIDDCS1ADDRL_RESETVAL (0x0000u)
N
N/* LCDLIDDCS1ADDRM */
N
N
N#define CSL_LCDC_LCDLIDDCS1ADDRM_RESETVAL (0x0000u)
N
N/* LCDLIDDCS1DATAL */
N
N#define CSL_LCDC_LCDLIDDCS1DATAL_DATA_MASK (0xFFFFu)
N#define CSL_LCDC_LCDLIDDCS1DATAL_DATA_SHIFT (0x0000u)
N#define CSL_LCDC_LCDLIDDCS1DATAL_DATA_RESETVAL (0x0000u)
N
N#define CSL_LCDC_LCDLIDDCS1DATAL_RESETVAL (0x0000u)
N
N/* LCDLIDDCS1DATAM */
N
N
N#define CSL_LCDC_LCDLIDDCS1DATAM_RESETVAL (0x0000u)
N
N/* LCDRASTCRL */
N
N#define CSL_LCDC_LCDRASTCRL_REQDLY_MASK (0xF000u)
N#define CSL_LCDC_LCDRASTCRL_REQDLY_SHIFT (0x000Cu)
N#define CSL_LCDC_LCDRASTCRL_REQDLY_RESETVAL (0x0000u)
N
N
N#define CSL_LCDC_LCDRASTCRL_MONO8B_MASK (0x0200u)
N#define CSL_LCDC_LCDRASTCRL_MONO8B_SHIFT (0x0009u)
N#define CSL_LCDC_LCDRASTCRL_MONO8B_RESETVAL (0x0000u)
N
N#define CSL_LCDC_LCDRASTCRL_RDORDER_MASK (0x0100u)
N#define CSL_LCDC_LCDRASTCRL_RDORDER_SHIFT (0x0008u)
N#define CSL_LCDC_LCDRASTCRL_RDORDER_RESETVAL (0x0000u)
N
N#define CSL_LCDC_LCDRASTCRL_LCDTFT_MASK (0x0080u)
N#define CSL_LCDC_LCDRASTCRL_LCDTFT_SHIFT (0x0007u)
N#define CSL_LCDC_LCDRASTCRL_LCDTFT_RESETVAL (0x0000u)
N
N#define CSL_LCDC_LCDRASTCRL_FUFEN_MASK (0x0040u)
N#define CSL_LCDC_LCDRASTCRL_FUFEN_SHIFT (0x0006u)
N#define CSL_LCDC_LCDRASTCRL_FUFEN_RESETVAL (0x0000u)
N/*----FUFEN Tokens----*/
N#define CSL_LCDC_LCDRASTCRL_FUFEN_DISABLE (0x0000u)
N#define CSL_LCDC_LCDRASTCRL_FUFEN_ENABLE (0x0001u)
N
N#define CSL_LCDC_LCDRASTCRL_SYNCEN_MASK (0x0020u)
N#define CSL_LCDC_LCDRASTCRL_SYNCEN_SHIFT (0x0005u)
N#define CSL_LCDC_LCDRASTCRL_SYNCEN_RESETVAL (0x0000u)
N/*----SYNCEN Tokens----*/
N#define CSL_LCDC_LCDRASTCRL_SYNCEN_DISABLE (0x0000u)
N#define CSL_LCDC_LCDRASTCRL_SYNCEN_ENABLE (0x0001u)
N
N#define CSL_LCDC_LCDRASTCRL_LOADEN_MASK (0x0010u)
N#define CSL_LCDC_LCDRASTCRL_LOADEN_SHIFT (0x0004u)
N#define CSL_LCDC_LCDRASTCRL_LOADEN_RESETVAL (0x0000u)
N/*----LOADEN Tokens----*/
N#define CSL_LCDC_LCDRASTCRL_LOADEN_DISABLE (0x0000u)
N#define CSL_LCDC_LCDRASTCRL_LOADEN_ENABLE (0x0001u)
N
N#define CSL_LCDC_LCDRASTCRL_DONEEN_MASK (0x0008u)
N#define CSL_LCDC_LCDRASTCRL_DONEEN_SHIFT (0x0003u)
N#define CSL_LCDC_LCDRASTCRL_DONEEN_RESETVAL (0x0000u)
N/*----DONEEN Tokens----*/
N#define CSL_LCDC_LCDRASTCRL_DONEEN_DISABLE (0x0000u)
N#define CSL_LCDC_LCDRASTCRL_DONEEN_ENABLE (0x0001u)
N
N#define CSL_LCDC_LCDRASTCRL_ABCEN_MASK (0x0004u)
N#define CSL_LCDC_LCDRASTCRL_ABCEN_SHIFT (0x0002u)
N#define CSL_LCDC_LCDRASTCRL_ABCEN_RESETVAL (0x0000u)
N/*----ABCEN Tokens----*/
N#define CSL_LCDC_LCDRASTCRL_ABCEN_DISABLE (0x0000u)
N#define CSL_LCDC_LCDRASTCRL_ABCEN_ENABLE (0x0001u)
N
N#define CSL_LCDC_LCDRASTCRL_LCDBW_MASK (0x0002u)
N#define CSL_LCDC_LCDRASTCRL_LCDBW_SHIFT (0x0001u)
N#define CSL_LCDC_LCDRASTCRL_LCDBW_RESETVAL (0x0000u)
N
N#define CSL_LCDC_LCDRASTCRL_LCDEN_MASK (0x0001u)
N#define CSL_LCDC_LCDRASTCRL_LCDEN_SHIFT (0x0000u)
N#define CSL_LCDC_LCDRASTCRL_LCDEN_RESETVAL (0x0000u)
N/*----LCDEN Tokens----*/
N#define CSL_LCDC_LCDRASTCRL_LCDEN_DISABLE (0x0000u)
N#define CSL_LCDC_LCDRASTCRL_LCDEN_ENABLE (0x0001u)
N
N#define CSL_LCDC_LCDRASTCRL_RESETVAL (0x0000u)
N
N/* LCDRASTCRM */
N
N
N#define CSL_LCDC_LCDRASTCRM_STN565_MASK (0x0100u)
N#define CSL_LCDC_LCDRASTCRM_STN565_SHIFT (0x0008u)
N#define CSL_LCDC_LCDRASTCRM_STN565_RESETVAL (0x0000u)
N
N#define CSL_LCDC_LCDRASTCRM_TFTMAP_MASK (0x0080u)
N#define CSL_LCDC_LCDRASTCRM_TFTMAP_SHIFT (0x0007u)
N#define CSL_LCDC_LCDRASTCRM_TFTMAP_RESETVAL (0x0000u)
N
N#define CSL_LCDC_LCDRASTCRM_NIBMODE_MASK (0x0040u)
N#define CSL_LCDC_LCDRASTCRM_NIBMODE_SHIFT (0x0006u)
N#define CSL_LCDC_LCDRASTCRM_NIBMODE_RESETVAL (0x0000u)
N
N#define CSL_LCDC_LCDRASTCRM_PALMODE_MASK (0x0030u)
N#define CSL_LCDC_LCDRASTCRM_PALMODE_SHIFT (0x0004u)
N#define CSL_LCDC_LCDRASTCRM_PALMODE_RESETVAL (0x0000u)
N
N#define CSL_LCDC_LCDRASTCRM_REQDLY_MASK (0x000Fu)
N#define CSL_LCDC_LCDRASTCRM_REQDLY_SHIFT (0x0000u)
N#define CSL_LCDC_LCDRASTCRM_REQDLY_RESETVAL (0x0000u)
N
N#define CSL_LCDC_LCDRASTCRM_RESETVAL (0x0000u)
N
N/* LCDRASTT0RL */
N
N#define CSL_LCDC_LCDRASTT0RL_HSW_MASK (0xFC00u)
N#define CSL_LCDC_LCDRASTT0RL_HSW_SHIFT (0x000Au)
N#define CSL_LCDC_LCDRASTT0RL_HSW_RESETVAL (0x0000u)
N
N#define CSL_LCDC_LCDRASTT0RL_PPL_MASK (0x03F0u)
N#define CSL_LCDC_LCDRASTT0RL_PPL_SHIFT (0x0004u)
N#define CSL_LCDC_LCDRASTT0RL_PPL_RESETVAL (0x0000u)
N
N
N#define CSL_LCDC_LCDRASTT0RL_RESETVAL (0x0000u)
N
N/* LCDRASTT0RM */
N
N#define CSL_LCDC_LCDRASTT0RM_HBP_MASK (0xFF00u)
N#define CSL_LCDC_LCDRASTT0RM_HBP_SHIFT (0x0008u)
N#define CSL_LCDC_LCDRASTT0RM_HBP_RESETVAL (0x0000u)
N
N#define CSL_LCDC_LCDRASTT0RM_HFP_MASK (0x00FFu)
N#define CSL_LCDC_LCDRASTT0RM_HFP_SHIFT (0x0000u)
N#define CSL_LCDC_LCDRASTT0RM_HFP_RESETVAL (0x0000u)
N
N#define CSL_LCDC_LCDRASTT0RM_RESETVAL (0x0000u)
N
N/* LCDRASTT1RL */
N
N#define CSL_LCDC_LCDRASTT1RL_VSW_MASK (0xFC00u)
N#define CSL_LCDC_LCDRASTT1RL_VSW_SHIFT (0x000Au)
N#define CSL_LCDC_LCDRASTT1RL_VSW_RESETVAL (0x0000u)
N
N#define CSL_LCDC_LCDRASTT1RL_LPP_MASK (0x03FFu)
N#define CSL_LCDC_LCDRASTT1RL_LPP_SHIFT (0x0000u)
N#define CSL_LCDC_LCDRASTT1RL_LPP_RESETVAL (0x0000u)
N
N#define CSL_LCDC_LCDRASTT1RL_RESETVAL (0x0000u)
N
N/* LCDRASTT1RM */
N
N#define CSL_LCDC_LCDRASTT1RM_VBP_MASK (0xFF00u)
N#define CSL_LCDC_LCDRASTT1RM_VBP_SHIFT (0x0008u)
N#define CSL_LCDC_LCDRASTT1RM_VBP_RESETVAL (0x0000u)
N
N#define CSL_LCDC_LCDRASTT1RM_VFP_MASK (0x00FFu)
N#define CSL_LCDC_LCDRASTT1RM_VFP_SHIFT (0x0000u)
N#define CSL_LCDC_LCDRASTT1RM_VFP_RESETVAL (0x0000u)
N
N#define CSL_LCDC_LCDRASTT1RM_RESETVAL (0x0000u)
N
N/* LCDRASTT2RL */
N
N#define CSL_LCDC_LCDRASTT2RL_ACB_MASK (0xFF00u)
N#define CSL_LCDC_LCDRASTT2RL_ACB_SHIFT (0x0008u)
N#define CSL_LCDC_LCDRASTT2RL_ACB_RESETVAL (0x0000u)
N
N
N#define CSL_LCDC_LCDRASTT2RL_RESETVAL (0x0000u)
N
N/* LCDRASTT2RM */
N
N
N#define CSL_LCDC_LCDRASTT2RM_PHSVS_ON_OFF_MASK (0x0200u)
N#define CSL_LCDC_LCDRASTT2RM_PHSVS_ON_OFF_SHIFT (0x0009u)
N#define CSL_LCDC_LCDRASTT2RM_PHSVS_ON_OFF_RESETVAL (0x0000u)
N
N#define CSL_LCDC_LCDRASTT2RM_PHSVS_RF_MASK (0x0100u)
N#define CSL_LCDC_LCDRASTT2RM_PHSVS_RF_SHIFT (0x0008u)
N#define CSL_LCDC_LCDRASTT2RM_PHSVS_RF_RESETVAL (0x0000u)
N
N#define CSL_LCDC_LCDRASTT2RM_IEO_MASK (0x0080u)
N#define CSL_LCDC_LCDRASTT2RM_IEO_SHIFT (0x0007u)
N#define CSL_LCDC_LCDRASTT2RM_IEO_RESETVAL (0x0000u)
N
N#define CSL_LCDC_LCDRASTT2RM_IPC_MASK (0x0040u)
N#define CSL_LCDC_LCDRASTT2RM_IPC_SHIFT (0x0006u)
N#define CSL_LCDC_LCDRASTT2RM_IPC_RESETVAL (0x0000u)
N
N#define CSL_LCDC_LCDRASTT2RM_IHS_MASK (0x0020u)
N#define CSL_LCDC_LCDRASTT2RM_IHS_SHIFT (0x0005u)
N#define CSL_LCDC_LCDRASTT2RM_IHS_RESETVAL (0x0000u)
N
N#define CSL_LCDC_LCDRASTT2RM_IVS_MASK (0x0010u)
N#define CSL_LCDC_LCDRASTT2RM_IVS_SHIFT (0x0004u)
N#define CSL_LCDC_LCDRASTT2RM_IVS_RESETVAL (0x0000u)
N
N#define CSL_LCDC_LCDRASTT2RM_ACBI_MASK (0x000Fu)
N#define CSL_LCDC_LCDRASTT2RM_ACBI_SHIFT (0x0000u)
N#define CSL_LCDC_LCDRASTT2RM_ACBI_RESETVAL (0x0000u)
N
N#define CSL_LCDC_LCDRASTT2RM_RESETVAL (0x0000u)
N
N/* LCDRASTSUBPANDISPL */
N
N#define CSL_LCDC_LCDRASTSUBPANDISPL_DPD_MASK (0xFFF0u)
N#define CSL_LCDC_LCDRASTSUBPANDISPL_DPD_SHIFT (0x0004u)
N#define CSL_LCDC_LCDRASTSUBPANDISPL_DPD_RESETVAL (0x0000u)
N
N
N#define CSL_LCDC_LCDRASTSUBPANDISPL_RESETVAL (0x0000u)
N
N/* LCDRASTSUBPANDISPM */
N
N#define CSL_LCDC_LCDRASTSUBPANDISPM_SPEN_MASK (0x8000u)
N#define CSL_LCDC_LCDRASTSUBPANDISPM_SPEN_SHIFT (0x000Fu)
N#define CSL_LCDC_LCDRASTSUBPANDISPM_SPEN_RESETVAL (0x0000u)
N
N
N#define CSL_LCDC_LCDRASTSUBPANDISPM_HOLS_MASK (0x2000u)
N#define CSL_LCDC_LCDRASTSUBPANDISPM_HOLS_SHIFT (0x000Du)
N#define CSL_LCDC_LCDRASTSUBPANDISPM_HOLS_RESETVAL (0x0000u)
N
N
N#define CSL_LCDC_LCDRASTSUBPANDISPM_LPPT_MASK (0x03FFu)
N#define CSL_LCDC_LCDRASTSUBPANDISPM_LPPT_SHIFT (0x0000u)
N#define CSL_LCDC_LCDRASTSUBPANDISPM_LPPT_RESETVAL (0x0000u)
N
N#define CSL_LCDC_LCDRASTSUBPANDISPM_RESETVAL (0x0000u)
N
N/* LCDDMACRL */
N
N
N#define CSL_LCDC_LCDDMACRL_BURST_SIZE_MASK (0x0070u)
N#define CSL_LCDC_LCDDMACRL_BURST_SIZE_SHIFT (0x0004u)
N#define CSL_LCDC_LCDDMACRL_BURST_SIZE_RESETVAL (0x0000u)
N/*----BURST_SIZE Tokens----*/
N#define CSL_LCDC_LCDDMACRL_BURST_SIZE_BSIZE1 (0x0000u)
N
N#define CSL_LCDC_LCDDMACRL_BYTE_SWP_MASK (0x0008u)
N#define CSL_LCDC_LCDDMACRL_BYTE_SWP_SHIFT (0x0003u)
N#define CSL_LCDC_LCDDMACRL_BYTE_SWP_RESETVAL (0x0000u)
N
N#define CSL_LCDC_LCDDMACRL_EOF_INTEN_MASK (0x0004u)
N#define CSL_LCDC_LCDDMACRL_EOF_INTEN_SHIFT (0x0002u)
N#define CSL_LCDC_LCDDMACRL_EOF_INTEN_RESETVAL (0x0000u)
N
N#define CSL_LCDC_LCDDMACRL_BIGENDIAN_MASK (0x0002u)
N#define CSL_LCDC_LCDDMACRL_BIGENDIAN_SHIFT (0x0001u)
N#define CSL_LCDC_LCDDMACRL_BIGENDIAN_RESETVAL (0x0000u)
N
N#define CSL_LCDC_LCDDMACRL_FRAME_MODE_MASK (0x0001u)
N#define CSL_LCDC_LCDDMACRL_FRAME_MODE_SHIFT (0x0000u)
N#define CSL_LCDC_LCDDMACRL_FRAME_MODE_RESETVAL (0x0000u)
N
N#define CSL_LCDC_LCDDMACRL_RESETVAL (0x0000u)
N
N/* LCDDMACRM */
N
N
N#define CSL_LCDC_LCDDMACRM_RESETVAL (0x0000u)
N
N/* LCDDMAFB0BARL */
N
N#define CSL_LCDC_LCDDMAFB0BARL_FB0_BASE_MASK (0xFFFCu)
N#define CSL_LCDC_LCDDMAFB0BARL_FB0_BASE_SHIFT (0x0002u)
N#define CSL_LCDC_LCDDMAFB0BARL_FB0_BASE_RESETVAL (0x0000u)
N
N
N#define CSL_LCDC_LCDDMAFB0BARL_RESETVAL (0x0000u)
N
N/* LCDDMAFB0BARM */
N
N#define CSL_LCDC_LCDDMAFB0BARM_FB0_BASE_MASK (0xFFFFu)
N#define CSL_LCDC_LCDDMAFB0BARM_FB0_BASE_SHIFT (0x0000u)
N#define CSL_LCDC_LCDDMAFB0BARM_FB0_BASE_RESETVAL (0x0000u)
N
N#define CSL_LCDC_LCDDMAFB0BARM_RESETVAL (0x0000u)
N
N/* LCDDMAFB0CARL */
N
N#define CSL_LCDC_LCDDMAFB0CARL_FB0_CEIL_MASK (0xFFFCu)
N#define CSL_LCDC_LCDDMAFB0CARL_FB0_CEIL_SHIFT (0x0002u)
N#define CSL_LCDC_LCDDMAFB0CARL_FB0_CEIL_RESETVAL (0x0000u)
N
N
N#define CSL_LCDC_LCDDMAFB0CARL_RESETVAL (0x0000u)
N
N/* LCDDMAFB0CARM */
N
N#define CSL_LCDC_LCDDMAFB0CARM_FB0_CEIL_MASK (0xFFFFu)
N#define CSL_LCDC_LCDDMAFB0CARM_FB0_CEIL_SHIFT (0x0000u)
N#define CSL_LCDC_LCDDMAFB0CARM_FB0_CEIL_RESETVAL (0x0000u)
N
N#define CSL_LCDC_LCDDMAFB0CARM_RESETVAL (0x0000u)
N
N/* LCDDMAFB1BARL */
N
N#define CSL_LCDC_LCDDMAFB1BARL_FB1_BASE_MASK (0xFFFCu)
N#define CSL_LCDC_LCDDMAFB1BARL_FB1_BASE_SHIFT (0x0002u)
N#define CSL_LCDC_LCDDMAFB1BARL_FB1_BASE_RESETVAL (0x0000u)
N
N
N#define CSL_LCDC_LCDDMAFB1BARL_RESETVAL (0x0000u)
N
N/* LCDDMAFB1BARM */
N
N#define CSL_LCDC_LCDDMAFB1BARM_FB1_BASE_MASK (0xFFFFu)
N#define CSL_LCDC_LCDDMAFB1BARM_FB1_BASE_SHIFT (0x0000u)
N#define CSL_LCDC_LCDDMAFB1BARM_FB1_BASE_RESETVAL (0x0000u)
N
N#define CSL_LCDC_LCDDMAFB1BARM_RESETVAL (0x0000u)
N
N/* LCDDMAFB1CARL */
N
N#define CSL_LCDC_LCDDMAFB1CARL_FB1_CEIL_MASK (0xFFFCu)
N#define CSL_LCDC_LCDDMAFB1CARL_FB1_CEIL_SHIFT (0x0002u)
N#define CSL_LCDC_LCDDMAFB1CARL_FB1_CEIL_RESETVAL (0x0000u)
N
N
N#define CSL_LCDC_LCDDMAFB1CARL_RESETVAL (0x0000u)
N
N/* LCDDMAFB1CARM */
N
N#define CSL_LCDC_LCDDMAFB1CARM_FB1_CEIL_MASK (0xFFFFu)
N#define CSL_LCDC_LCDDMAFB1CARM_FB1_CEIL_SHIFT (0x0000u)
N#define CSL_LCDC_LCDDMAFB1CARM_FB1_CEIL_RESETVAL (0x0000u)
N
N#define CSL_LCDC_LCDDMAFB1CARM_RESETVAL (0x0000u)
N
N#endif
L 63 "../common_inc/corazon.h" 2
N#include "cslr_rtc_001.h"
L 1 "..\common_inc\cslr_rtc_001.h" 1
N/*****************************************************************************
N * File Name : cslr_rtc_001.h 
N *
N * Brief	 : Define RTC register strcuture
N *
N * Copyright (C) 2009 -2010 Texas Instruments Incorporated - http://www.ti.com/ 
N * 
N * 
N *  Redistribution and use in source and binary forms, with or without 
N *  modification, are permitted provided that the following conditions 
N *  are met:
N *
N *    Redistributions of source code must retain the above copyright 
N *    notice, this list of conditions and the following disclaimer.
N *
N *    Redistributions in binary form must reproduce the above copyright
N *    notice, this list of conditions and the following disclaimer in the 
N *    documentation and/or other materials provided with the   
N *    distribution.
N *
N *    Neither the name of Texas Instruments Incorporated nor the names of
N *    its contributors may be used to endorse or promote products derived
N *    from this software without specific prior written permission.
N *
N *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
N *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
N *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
N *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
N *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
N *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
N *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
N *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
N *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
N *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
N *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
N *
N******************************************************************************/
N#ifndef _CSLR_RTC_1_H_
N#define _CSLR_RTC_1_H_
N
N#include <cslr.h>
N
N/* Minimum unit = 2 bytes */
N
N/**************************************************************************\
N* Register Overlay Structure
X
N\**************************************************************************/
Ntypedef struct  {
N    volatile Uint16 CTRL1;
N    volatile Uint16 CTRL2;
N    volatile Uint16 RSVD0[2];
N    volatile Uint16 MS;
N    volatile Uint16 MSALARM;
N    volatile Uint16 RSVD1[2];
N    volatile Uint16 SEC;
N    volatile Uint16 SECALARM;
N    volatile Uint16 RSVD2[2];
N    volatile Uint16 MINS;
N    volatile Uint16 MINALARM;
N    volatile Uint16 RSVD3[2];
N    volatile Uint16 HRS;
N    volatile Uint16 HRALARM;
N    volatile Uint16 RSVD4[2];
N    volatile Uint16 DAY;
N    volatile Uint16 DAYALARM;
N    volatile Uint16 RSVD5[2];
N    volatile Uint16 MONTH;
N    volatile Uint16 MONTHALARM;
N    volatile Uint16 RSVD6[2];
N    volatile Uint16 YEAR;
N    volatile Uint16 YEARALARM;
N    volatile Uint16 RSVD7[2];
N    volatile Uint16 STATUS1;
N    volatile Uint16 STATUS2;
N    volatile Uint16 RSVD8[2];
N    volatile Uint16 INTCR;
N    volatile Uint16 RSVD9[3];
N    volatile Uint16 COMP;
N    volatile Uint16 RSVD10[3];
N    volatile Uint16 OSC;
N    volatile Uint16 RSVD11[3];
N    volatile Uint16 PMGT;
N    volatile Uint16 RSVD12[47];
N    volatile Uint16 SCRATC0_LSW;
N    volatile Uint16 SCRATCH0_MSW;
N    volatile Uint16 RSVD13[2];
N    volatile Uint16 SCRATCH1_LSW;
N    volatile Uint16 SCRATCH1_MSW;
N} CSL_RtcRegs;
N
N/**************************************************************************\
N* Field Definition Macros
X
N\**************************************************************************/
N
N/* CTRL1 */
N
N
N#define CSL_RTC_CTRL1_VBUSP_INT_EN_MASK (0x0001u)
N#define CSL_RTC_CTRL1_VBUSP_INT_EN_SHIFT (0x0000u)
N#define CSL_RTC_CTRL1_VBUSP_INT_EN_RESETVAL (0x0000u)
N
N#define CSL_RTC_CTRL1_RESETVAL (0x0000u)
N
N/* CTRL2 */
N
N#define CSL_RTC_CTRL2_UPDT_TIME_MASK (0x8000u)
N#define CSL_RTC_CTRL2_UPDT_TIME_SHIFT (0x000Fu)
N#define CSL_RTC_CTRL2_UPDT_TIME_RESETVAL (0x0000u)
N
N#define CSL_RTC_CTRL2_UPDT_ALARM_MASK (0x4000u)
N#define CSL_RTC_CTRL2_UPDT_ALARM_SHIFT (0x000Eu)
N#define CSL_RTC_CTRL2_UPDT_ALARM_RESETVAL (0x0000u)
N
N
N#define CSL_RTC_CTRL2_RESETVAL (0x0000u)
N
N/* MS */
N
N
N#define CSL_RTC_MS_SEC_MASK (0x1FFFu)
N#define CSL_RTC_MS_SEC_SHIFT (0x0000u)
N#define CSL_RTC_MS_SEC_RESETVAL (0x0000u)
N
N#define CSL_RTC_MS_RESETVAL (0x0000u)
N
N/* MSALARM */
N
N
N#define CSL_RTC_MSALARM_SEC_ALARM_MASK (0x1FFFu)
N#define CSL_RTC_MSALARM_SEC_ALARM_SHIFT (0x0000u)
N#define CSL_RTC_MSALARM_SEC_ALARM_RESETVAL (0x0000u)
N
N#define CSL_RTC_MSALARM_RESETVAL (0x0000u)
N
N/* SEC */
N
N
N
N#define CSL_RTC_SEC_SEC_MASK (0x007Fu)
N#define CSL_RTC_SEC_SEC_SHIFT (0x0000u)
N#define CSL_RTC_SEC_SEC_RESETVAL (0x0000u)
N
N#define CSL_RTC_SEC_RESETVAL (0x0000u)
N
N/* SECALARM */
N
N
N
N#define CSL_RTC_SECALARM_SEC_ALARM_MASK (0x007Fu)
N#define CSL_RTC_SECALARM_SEC_ALARM_SHIFT (0x0000u)
N#define CSL_RTC_SECALARM_SEC_ALARM_RESETVAL (0x0000u)
N
N#define CSL_RTC_SECALARM_RESETVAL (0x0000u)
N
N/* MINS */
N
N
N
N#define CSL_RTC_MINS_MIN_MASK (0x007Fu)
N#define CSL_RTC_MINS_MIN_SHIFT (0x0000u)
N#define CSL_RTC_MINS_MIN_RESETVAL (0x0000u)
N
N#define CSL_RTC_MINS_RESETVAL (0x0000u)
N
N/* MINALARM */
N
N
N
N#define CSL_RTC_MINALARM_MIN_ALARM_MASK (0x007Fu)
N#define CSL_RTC_MINALARM_MIN_ALARM_SHIFT (0x0000u)
N#define CSL_RTC_MINALARM_MIN_ALARM_RESETVAL (0x0000u)
N
N#define CSL_RTC_MINALARM_RESETVAL (0x0000u)
N
N/* HRS */
N
N
N#define CSL_RTC_HRS_HR_MASK (0x003Fu)
N#define CSL_RTC_HRS_HR_SHIFT (0x0000u)
N#define CSL_RTC_HRS_HR_RESETVAL (0x0000u)
N
N#define CSL_RTC_HRS_RESETVAL (0x0000u)
N
N/* HRALARM */
N
N
N#define CSL_RTC_HRALARM_HR_ALARM_MASK (0x003Fu)
N#define CSL_RTC_HRALARM_HR_ALARM_SHIFT (0x0000u)
N#define CSL_RTC_HRALARM_HR_ALARM_RESETVAL (0x0000u)
N
N#define CSL_RTC_HRALARM_RESETVAL (0x0000u)
N
N/* DAY */
N
N
N#define CSL_RTC_DAY_DAY_MASK (0x003Fu)
N#define CSL_RTC_DAY_DAY_SHIFT (0x0000u)
N#define CSL_RTC_DAY_DAY_RESETVAL (0x0001u)
N
N#define CSL_RTC_DAY_RESETVAL (0x0001u)
N
N/* DAYALARM */
N
N
N#define CSL_RTC_DAYALARM_DAY_ALARM_MASK (0x003Fu)
N#define CSL_RTC_DAYALARM_DAY_ALARM_SHIFT (0x0000u)
N#define CSL_RTC_DAYALARM_DAY_ALARM_RESETVAL (0x0001u)
N
N#define CSL_RTC_DAYALARM_RESETVAL (0x0001u)
N
N/* MONTH */
N
N
N#define CSL_RTC_MONTH_MONTH_MASK (0x001Fu)
N#define CSL_RTC_MONTH_MONTH_SHIFT (0x0000u)
N#define CSL_RTC_MONTH_MONTH_RESETVAL (0x0000u)
N
N#define CSL_RTC_MONTH_RESETVAL (0x0000u)
N
N/* MONTHALARM */
N
N
N#define CSL_RTC_MONTHALARM_MONTH_ALARM_MASK (0x001Fu)
N#define CSL_RTC_MONTHALARM_MONTH_ALARM_SHIFT (0x0000u)
N#define CSL_RTC_MONTHALARM_MONTH_ALARM_RESETVAL (0x0000u)
N
N#define CSL_RTC_MONTHALARM_RESETVAL (0x0000u)
N
N/* YEAR */
N
N
N#define CSL_RTC_YEAR_YEAR_MASK (0x00FFu)
N#define CSL_RTC_YEAR_YEAR_SHIFT (0x0000u)
N#define CSL_RTC_YEAR_YEAR_RESETVAL (0x0000u)
N
N#define CSL_RTC_YEAR_RESETVAL (0x0000u)
N
N/* YEARALARM */
N
N
N#define CSL_RTC_YEARALARM_YEAR_ALARM_MASK (0x00FFu)
N#define CSL_RTC_YEARALARM_YEAR_ALARM_SHIFT (0x0000u)
N#define CSL_RTC_YEARALARM_YEAR_ALARM_RESETVAL (0x0000u)
N
N#define CSL_RTC_YEARALARM_RESETVAL (0x0000u)
N
N/* STATUS1 */
N
N#define CSL_RTC_STATUS1_ALARM_MASK (0x8000u)
N#define CSL_RTC_STATUS1_ALARM_SHIFT (0x000Fu)
N#define CSL_RTC_STATUS1_ALARM_RESETVAL (0x0000u)
N
N
N#define CSL_RTC_STATUS1_EXT_EVT_MASK (0x0020u)
N#define CSL_RTC_STATUS1_EXT_EVT_SHIFT (0x0005u)
N#define CSL_RTC_STATUS1_EXT_EVT_RESETVAL (0x0000u)
N
N#define CSL_RTC_STATUS1_DAY_EVT_MASK (0x0010u)
N#define CSL_RTC_STATUS1_DAY_EVT_SHIFT (0x0004u)
N#define CSL_RTC_STATUS1_DAY_EVT_RESETVAL (0x0000u)
N
N#define CSL_RTC_STATUS1_HR_EVT_MASK (0x0008u)
N#define CSL_RTC_STATUS1_HR_EVT_SHIFT (0x0003u)
N#define CSL_RTC_STATUS1_HR_EVT_RESETVAL (0x0000u)
N
N#define CSL_RTC_STATUS1_MIN_EVT_MASK (0x0004u)
N#define CSL_RTC_STATUS1_MIN_EVT_SHIFT (0x0002u)
N#define CSL_RTC_STATUS1_MIN_EVT_RESETVAL (0x0000u)
N
N#define CSL_RTC_STATUS1_SEC_EVT_MASK (0x0002u)
N#define CSL_RTC_STATUS1_SEC_EVT_SHIFT (0x0001u)
N#define CSL_RTC_STATUS1_SEC_EVT_RESETVAL (0x0000u)
N
N#define CSL_RTC_STATUS1_MS_EVT_MASK (0x0001u)
N#define CSL_RTC_STATUS1_MS_EVT_SHIFT (0x0000u)
N#define CSL_RTC_STATUS1_MS_EVT_RESETVAL (0x0000u)
N
N#define CSL_RTC_STATUS1_RESETVAL (0x0000u)
N
N/* STATUS2 */
N
N
N
N
N#define CSL_RTC_STATUS2_PWRUP_MASK (0x0001u)
N#define CSL_RTC_STATUS2_PWRUP_SHIFT (0x0000u)
N#define CSL_RTC_STATUS2_PWRUP_RESETVAL (0x0001u)
N
N#define CSL_RTC_STATUS2_RESETVAL (0x0001u)
N
N/* INTCR */
N
N#define CSL_RTC_INTCR_ALARM_INT_MASK (0x8000u)
N#define CSL_RTC_INTCR_ALARM_INT_SHIFT (0x000Fu)
N#define CSL_RTC_INTCR_ALARM_INT_RESETVAL (0x0000u)
N
N
N#define CSL_RTC_INTCR_EXTEVT_INT_MASK (0x0020u)
N#define CSL_RTC_INTCR_EXTEVT_INT_SHIFT (0x0005u)
N#define CSL_RTC_INTCR_EXTEVT_INT_RESETVAL (0x0000u)
N
N#define CSL_RTC_INTCR_DAYEVT_INT_MASK (0x0010u)
N#define CSL_RTC_INTCR_DAYEVT_INT_SHIFT (0x0004u)
N#define CSL_RTC_INTCR_DAYEVT_INT_RESETVAL (0x0000u)
N
N#define CSL_RTC_INTCR_HREVT_INT_MASK (0x0008u)
N#define CSL_RTC_INTCR_HREVT_INT_SHIFT (0x0003u)
N#define CSL_RTC_INTCR_HREVT_INT_RESETVAL (0x0000u)
N
N#define CSL_RTC_INTCR_MINEVT_INT_MASK (0x0004u)
N#define CSL_RTC_INTCR_MINEVT_INT_SHIFT (0x0002u)
N#define CSL_RTC_INTCR_MINEVT_INT_RESETVAL (0x0000u)
N
N#define CSL_RTC_INTCR_SECEVT_INT_MASK (0x0002u)
N#define CSL_RTC_INTCR_SECEVT_INT_SHIFT (0x0001u)
N#define CSL_RTC_INTCR_SECEVT_INT_RESETVAL (0x0000u)
N
N#define CSL_RTC_INTCR_MSEVT_INT_MASK (0x0001u)
N#define CSL_RTC_INTCR_MSEVT_INT_SHIFT (0x0000u)
N#define CSL_RTC_INTCR_MSEVT_INT_RESETVAL (0x0000u)
N
N#define CSL_RTC_INTCR_RESETVAL (0x0000u)
N
N/* COMP */
N
N#define CSL_RTC_COMP_DRIFT_MASK (0x8000u)
N#define CSL_RTC_COMP_DRIFT_SHIFT (0x000Fu)
N#define CSL_RTC_COMP_DRIFT_RESETVAL (0x0000u)
N
N
N#define CSL_RTC_COMP_COMP_MASK (0x1FFFu)
N#define CSL_RTC_COMP_COMP_SHIFT (0x0000u)
N#define CSL_RTC_COMP_COMP_RESETVAL (0x0000u)
N
N#define CSL_RTC_COMP_RESETVAL (0x0000u)
N
N/* OSC */
N
N#define CSL_RTC_OSC_SW_RESET_MASK (0x8000u)
N#define CSL_RTC_OSC_SW_RESET_SHIFT (0x000Fu)
N#define CSL_RTC_OSC_SW_RESET_RESETVAL (0x0000u)
N
N
N#define CSL_RTC_OSC_OSC32K_PWRD_MASK (0x0010u)
N#define CSL_RTC_OSC_OSC32K_PWRD_SHIFT (0x0004u)
N#define CSL_RTC_OSC_OSC32K_PWRD_RESETVAL (0x0000u)
N
N#define CSL_RTC_OSC_RESET_MASK (0x000Fu)
N#define CSL_RTC_OSC_RESET_SHIFT (0x0000u)
N#define CSL_RTC_OSC_RESET_RESETVAL (0x0008u)
N
N#define CSL_RTC_OSC_RESETVAL (0x0008u)
N
N/* PMGT */
N
N
N#define CSL_RTC_PMGT_WU_DOUT_MASK (0x0010u)
N#define CSL_RTC_PMGT_WU_DOUT_SHIFT (0x0004u)
N#define CSL_RTC_PMGT_WU_DOUT_RESETVAL (0x0000u)
N
N#define CSL_RTC_PMGT_WU_DIR_MASK (0x0008u)
N#define CSL_RTC_PMGT_WU_DIR_SHIFT (0x0003u)
N#define CSL_RTC_PMGT_WU_DIR_RESETVAL (0x0000u)
N
N#define CSL_RTC_PMGT_BG_PD_MASK (0x0004u)
N#define CSL_RTC_PMGT_BG_PD_SHIFT (0x0002u)
N#define CSL_RTC_PMGT_BG_PD_RESETVAL (0x0000u)
N
N#define CSL_RTC_PMGT_LDO_PD_MASK (0x0002u)
N#define CSL_RTC_PMGT_LDO_PD_SHIFT (0x0001u)
N#define CSL_RTC_PMGT_LDO_PD_RESETVAL (0x0000u)
N
N#define CSL_RTC_PMGT_CLKOUTEN_MASK (0x0001u)
N#define CSL_RTC_PMGT_CLKOUTEN_SHIFT (0x0000u)
N#define CSL_RTC_PMGT_CLKOUTEN_RESETVAL (0x0000u)
N
N#define CSL_RTC_PMGT_RESETVAL (0x0000u)
N
N/* SCRATC0_LSW */
N
N#define CSL_RTC_SCRATC0_LSW_LSWSCRATCH0_MASK (0xFFFFu)
N#define CSL_RTC_SCRATC0_LSW_LSWSCRATCH0_SHIFT (0x0000u)
N#define CSL_RTC_SCRATC0_LSW_LSWSCRATCH0_RESETVAL (0x0000u)
N
N#define CSL_RTC_SCRATC0_LSW_RESETVAL (0x0000u)
N
N/* SCRATCH0_MSW */
N
N#define CSL_RTC_SCRATCH0_MSW_MSWSCRATCH0_MASK (0xFFFFu)
N#define CSL_RTC_SCRATCH0_MSW_MSWSCRATCH0_SHIFT (0x0000u)
N#define CSL_RTC_SCRATCH0_MSW_MSWSCRATCH0_RESETVAL (0x0000u)
N
N#define CSL_RTC_SCRATCH0_MSW_RESETVAL (0x0000u)
N
N/* SCRATCH1_LSW */
N
N#define CSL_RTC_SCRATCH1_LSW_LSWSCRATCH1_MASK (0xFFFFu)
N#define CSL_RTC_SCRATCH1_LSW_LSWSCRATCH1_SHIFT (0x0000u)
N#define CSL_RTC_SCRATCH1_LSW_LSWSCRATCH1_RESETVAL (0x0000u)
N
N#define CSL_RTC_SCRATCH1_LSW_RESETVAL (0x0000u)
N
N/* SCRATCH1_MSW */
N
N#define CSL_RTC_SCRATCH1_MSW_MSWSCRATCH1_MASK (0xFFFFu)
N#define CSL_RTC_SCRATCH1_MSW_MSWSCRATCH1_SHIFT (0x0000u)
N#define CSL_RTC_SCRATCH1_MSW_MSWSCRATCH1_RESETVAL (0x0000u)
N
N#define CSL_RTC_SCRATCH1_MSW_IOINT_MASK (0x0001u)
N#define CSL_RTC_SCRATCH1_MSW_IOINT_SHIFT (0x0000u)
N#define CSL_RTC_SCRATCH1_MSW_IOINT_RESETVAL (0x0000u)
N
N#define CSL_RTC_SCRATCH1_MSW_RESETVAL (0x0000u)
N
N#endif
N
L 64 "../common_inc/corazon.h" 2
N#include "cslr_dma_001.h"
L 1 "..\common_inc\cslr_dma_001.h" 1
N/*****************************************************************************
N * File Name : cslr_dma_001.h 
N *
N * Brief	 : Define DMA register structure
N *
N * Copyright (C) 2009 -2010 Texas Instruments Incorporated - http://www.ti.com/ 
N * 
N * 
N *  Redistribution and use in source and binary forms, with or without 
N *  modification, are permitted provided that the following conditions 
N *  are met:
N *
N *    Redistributions of source code must retain the above copyright 
N *    notice, this list of conditions and the following disclaimer.
N *
N *    Redistributions in binary form must reproduce the above copyright
N *    notice, this list of conditions and the following disclaimer in the 
N *    documentation and/or other materials provided with the   
N *    distribution.
N *
N *    Neither the name of Texas Instruments Incorporated nor the names of
N *    its contributors may be used to endorse or promote products derived
N *    from this software without specific prior written permission.
N *
N *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
N *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
N *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
N *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
N *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
N *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
N *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
N *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
N *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
N *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
N *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
N *
N******************************************************************************/
N#ifndef _CSLR__DMA_H_
N#define _CSLR__DMA_H_
N
N#include <cslr.h>
N
N
N/* Minimum unit = 2 bytes */
N
N/**************************************************************************\
N* Register Overlay Structure
X
N\**************************************************************************/
Ntypedef struct  {
N    volatile Uint16 DMACH0SADR0;
N    volatile Uint16 DMACH0SADR1;
N    volatile Uint16 DMACH0DADR0;
N    volatile Uint16 DMACH0DADR1;
N    volatile Uint16 DMACH0TC0;
N    volatile Uint16 DMACH0TC1;
N    volatile Uint16 RSVD0[26];
N    volatile Uint16 DMACH1SADR0;
N    volatile Uint16 DMACH1SADR1;
N    volatile Uint16 DMACH1DADR0;
N    volatile Uint16 DMACH1DADR1;
N    volatile Uint16 DMACH1TC0;
N    volatile Uint16 DMACH1TC1;
N    volatile Uint16 RSVD1[26];
N    volatile Uint16 DMACH2SADR0;
N    volatile Uint16 DMACH2SADR1;
N    volatile Uint16 DMACH2DADR0;
N    volatile Uint16 DMACH2DADR1;
N    volatile Uint16 DMACH2TC0;
N    volatile Uint16 DMACH2TC1;
N    volatile Uint16 RSVD2[26];
N    volatile Uint16 DMACH3SADR0;
N    volatile Uint16 DMACH3SADR1;
N    volatile Uint16 DMACH3DADR0;
N    volatile Uint16 DMACH3DADR1;
N    volatile Uint16 DMACH3TC0;
N    volatile Uint16 DMACH3TC1;
N} CSL_DmaRegs;
N
N/**************************************************************************\
N* Field Definition Macros
X
N\**************************************************************************/
N
N/* DMACH0SADR0 */
N
N#define CSL_DMA_DMACH0SADR0_LSWADDR_MASK (0xFFFFu)
N#define CSL_DMA_DMACH0SADR0_LSWADDR_SHIFT (0x0000u)
N#define CSL_DMA_DMACH0SADR0_LSWADDR_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMACH0SADR0_RESETVAL (0x0000u)
N
N/* DMACH0SADR1 */
N
N#define CSL_DMA_DMACH0SADR1_MSWADDR_MASK (0xFFFFu)
N#define CSL_DMA_DMACH0SADR1_MSWADDR_SHIFT (0x0000u)
N#define CSL_DMA_DMACH0SADR1_MSWADDR_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMACH0SADR1_RESETVAL (0x0000u)
N
N/* DMACH0DADR0 */
N
N#define CSL_DMA_DMACH0DADR0_LSWADDR_MASK (0xFFFFu)
N#define CSL_DMA_DMACH0DADR0_LSWADDR_SHIFT (0x0000u)
N#define CSL_DMA_DMACH0DADR0_LSWADDR_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMACH0DADR0_RESETVAL (0x0000u)
N
N/* DMACH0DADR1 */
N
N#define CSL_DMA_DMACH0DADR1_MSWADDR_MASK (0xFFFFu)
N#define CSL_DMA_DMACH0DADR1_MSWADDR_SHIFT (0x0000u)
N#define CSL_DMA_DMACH0DADR1_MSWADDR_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMACH0DADR1_RESETVAL (0x0000u)
N
N/* DMACH0TC0 */
N
N#define CSL_DMA_DMACH0TC0_TXLEN_MASK (0xFFFFu)
N#define CSL_DMA_DMACH0TC0_TXLEN_SHIFT (0x0000u)
N#define CSL_DMA_DMACH0TC0_TXLEN_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMACH0TC0_RESETVAL (0x0000u)
N
N/* DMACH0TC1 */
N
N#define CSL_DMA_DMACH0TC1_DMASTART_MASK (0x8000u)
N#define CSL_DMA_DMACH0TC1_DMASTART_SHIFT (0x000Fu)
N#define CSL_DMA_DMACH0TC1_DMASTART_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMACH0TC1_DMASTATUS_MASK (0x4000u)
N#define CSL_DMA_DMACH0TC1_DMASTATUS_SHIFT (0x000Eu)
N#define CSL_DMA_DMACH0TC1_DMASTATUS_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMACH0TC1_INTEN_MASK (0x2000u)
N#define CSL_DMA_DMACH0TC1_INTEN_SHIFT (0x000Du)
N#define CSL_DMA_DMACH0TC1_INTEN_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMACH0TC1_AUTORELOAD_MASK (0x1000u)
N#define CSL_DMA_DMACH0TC1_AUTORELOAD_SHIFT (0x000Cu)
N#define CSL_DMA_DMACH0TC1_AUTORELOAD_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMACH0TC1_DESTDATASIZE_MASK (0x0C00u)
N#define CSL_DMA_DMACH0TC1_DESTDATASIZE_SHIFT (0x000Au)
N#define CSL_DMA_DMACH0TC1_DESTDATASIZE_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMACH0TC1_DESTADRMODE_MASK (0x0300u)
N#define CSL_DMA_DMACH0TC1_DESTADRMODE_SHIFT (0x0008u)
N#define CSL_DMA_DMACH0TC1_DESTADRMODE_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMACH0TC1_SRCADRMODE_MASK (0x00C0u)
N#define CSL_DMA_DMACH0TC1_SRCADRMODE_SHIFT (0x0006u)
N#define CSL_DMA_DMACH0TC1_SRCADRMODE_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMACH0TC1_BURSTMODE_MASK (0x0038u)
N#define CSL_DMA_DMACH0TC1_BURSTMODE_SHIFT (0x0003u)
N#define CSL_DMA_DMACH0TC1_BURSTMODE_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMACH0TC1_SYNC_MASK (0x0004u)
N#define CSL_DMA_DMACH0TC1_SYNC_SHIFT (0x0002u)
N#define CSL_DMA_DMACH0TC1_SYNC_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMACH0TC1_SRCDATASIZE_MASK (0x0003u)
N#define CSL_DMA_DMACH0TC1_SRCDATASIZE_SHIFT (0x0000u)
N#define CSL_DMA_DMACH0TC1_SRCDATASIZE_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMACH0TC1_RESETVAL (0x0000u)
N
N/* DMACH1SADR0 */
N
N#define CSL_DMA_DMACH1SADR0_LSWADDR_MASK (0xFFFFu)
N#define CSL_DMA_DMACH1SADR0_LSWADDR_SHIFT (0x0000u)
N#define CSL_DMA_DMACH1SADR0_LSWADDR_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMACH1SADR0_RESETVAL (0x0000u)
N
N/* DMACH1SADR1 */
N
N#define CSL_DMA_DMACH1SADR1_MSWADDR_MASK (0xFFFFu)
N#define CSL_DMA_DMACH1SADR1_MSWADDR_SHIFT (0x0000u)
N#define CSL_DMA_DMACH1SADR1_MSWADDR_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMACH1SADR1_RESETVAL (0x0000u)
N
N/* DMACH1DADR0 */
N
N#define CSL_DMA_DMACH1DADR0_LSWADDR_MASK (0xFFFFu)
N#define CSL_DMA_DMACH1DADR0_LSWADDR_SHIFT (0x0000u)
N#define CSL_DMA_DMACH1DADR0_LSWADDR_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMACH1DADR0_RESETVAL (0x0000u)
N
N/* DMACH1DADR1 */
N
N#define CSL_DMA_DMACH1DADR1_MSWADDR_MASK (0xFFFFu)
N#define CSL_DMA_DMACH1DADR1_MSWADDR_SHIFT (0x0000u)
N#define CSL_DMA_DMACH1DADR1_MSWADDR_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMACH1DADR1_RESETVAL (0x0000u)
N
N/* DMACH1TC0 */
N
N#define CSL_DMA_DMACH1TC0_TXLEN_MASK (0xFFFFu)
N#define CSL_DMA_DMACH1TC0_TXLEN_SHIFT (0x0000u)
N#define CSL_DMA_DMACH1TC0_TXLEN_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMACH1TC0_RESETVAL (0x0000u)
N
N/* DMACH1TC1 */
N
N#define CSL_DMA_DMACH1TC1_DMASTART_MASK (0x8000u)
N#define CSL_DMA_DMACH1TC1_DMASTART_SHIFT (0x000Fu)
N#define CSL_DMA_DMACH1TC1_DMASTART_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMACH1TC1_DMASTATUS_MASK (0x4000u)
N#define CSL_DMA_DMACH1TC1_DMASTATUS_SHIFT (0x000Eu)
N#define CSL_DMA_DMACH1TC1_DMASTATUS_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMACH1TC1_INTEN_MASK (0x2000u)
N#define CSL_DMA_DMACH1TC1_INTEN_SHIFT (0x000Du)
N#define CSL_DMA_DMACH1TC1_INTEN_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMACH1TC1_AUTORELOAD_MASK (0x1000u)
N#define CSL_DMA_DMACH1TC1_AUTORELOAD_SHIFT (0x000Cu)
N#define CSL_DMA_DMACH1TC1_AUTORELOAD_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMACH1TC1_DESTDATASIZE_MASK (0x0C00u)
N#define CSL_DMA_DMACH1TC1_DESTDATASIZE_SHIFT (0x000Au)
N#define CSL_DMA_DMACH1TC1_DESTDATASIZE_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMACH1TC1_DESTADRMODE_MASK (0x0300u)
N#define CSL_DMA_DMACH1TC1_DESTADRMODE_SHIFT (0x0008u)
N#define CSL_DMA_DMACH1TC1_DESTADRMODE_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMACH1TC1_SRCADRMODE_MASK (0x00C0u)
N#define CSL_DMA_DMACH1TC1_SRCADRMODE_SHIFT (0x0006u)
N#define CSL_DMA_DMACH1TC1_SRCADRMODE_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMACH1TC1_BURSTMODE_MASK (0x0038u)
N#define CSL_DMA_DMACH1TC1_BURSTMODE_SHIFT (0x0003u)
N#define CSL_DMA_DMACH1TC1_BURSTMODE_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMACH1TC1_SYNC_MASK (0x0004u)
N#define CSL_DMA_DMACH1TC1_SYNC_SHIFT (0x0002u)
N#define CSL_DMA_DMACH1TC1_SYNC_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMACH1TC1_SRCDATASIZE_MASK (0x0003u)
N#define CSL_DMA_DMACH1TC1_SRCDATASIZE_SHIFT (0x0000u)
N#define CSL_DMA_DMACH1TC1_SRCDATASIZE_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMACH1TC1_RESETVAL (0x0000u)
N
N/* DMACH2SADR0 */
N
N#define CSL_DMA_DMACH2SADR0_LSWADDR_MASK (0xFFFFu)
N#define CSL_DMA_DMACH2SADR0_LSWADDR_SHIFT (0x0000u)
N#define CSL_DMA_DMACH2SADR0_LSWADDR_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMACH2SADR0_RESETVAL (0x0000u)
N
N/* DMACH2SADR1 */
N
N#define CSL_DMA_DMACH2SADR1_MSWADDR_MASK (0xFFFFu)
N#define CSL_DMA_DMACH2SADR1_MSWADDR_SHIFT (0x0000u)
N#define CSL_DMA_DMACH2SADR1_MSWADDR_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMACH2SADR1_RESETVAL (0x0000u)
N
N/* DMACH2DADR0 */
N
N#define CSL_DMA_DMACH2DADR0_LSWADDR_MASK (0xFFFFu)
N#define CSL_DMA_DMACH2DADR0_LSWADDR_SHIFT (0x0000u)
N#define CSL_DMA_DMACH2DADR0_LSWADDR_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMACH2DADR0_RESETVAL (0x0000u)
N
N/* DMACH2DADR1 */
N
N#define CSL_DMA_DMACH2DADR1_MSWADDR_MASK (0xFFFFu)
N#define CSL_DMA_DMACH2DADR1_MSWADDR_SHIFT (0x0000u)
N#define CSL_DMA_DMACH2DADR1_MSWADDR_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMACH2DADR1_RESETVAL (0x0000u)
N
N/* DMACH2TC0 */
N
N#define CSL_DMA_DMACH2TC0_TXLEN_MASK (0xFFFFu)
N#define CSL_DMA_DMACH2TC0_TXLEN_SHIFT (0x0000u)
N#define CSL_DMA_DMACH2TC0_TXLEN_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMACH2TC0_RESETVAL (0x0000u)
N
N/* DMACH2TC1 */
N
N#define CSL_DMA_DMACH2TC1_DMASTART_MASK (0x8000u)
N#define CSL_DMA_DMACH2TC1_DMASTART_SHIFT (0x000Fu)
N#define CSL_DMA_DMACH2TC1_DMASTART_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMACH2TC1_DMASTATUS_MASK (0x4000u)
N#define CSL_DMA_DMACH2TC1_DMASTATUS_SHIFT (0x000Eu)
N#define CSL_DMA_DMACH2TC1_DMASTATUS_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMACH2TC1_INTEN_MASK (0x2000u)
N#define CSL_DMA_DMACH2TC1_INTEN_SHIFT (0x000Du)
N#define CSL_DMA_DMACH2TC1_INTEN_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMACH2TC1_AUTORELOAD_MASK (0x1000u)
N#define CSL_DMA_DMACH2TC1_AUTORELOAD_SHIFT (0x000Cu)
N#define CSL_DMA_DMACH2TC1_AUTORELOAD_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMACH2TC1_DESTDATASIZE_MASK (0x0C00u)
N#define CSL_DMA_DMACH2TC1_DESTDATASIZE_SHIFT (0x000Au)
N#define CSL_DMA_DMACH2TC1_DESTDATASIZE_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMACH2TC1_DESTADRMODE_MASK (0x0300u)
N#define CSL_DMA_DMACH2TC1_DESTADRMODE_SHIFT (0x0008u)
N#define CSL_DMA_DMACH2TC1_DESTADRMODE_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMACH2TC1_SRCADRMODE_MASK (0x00C0u)
N#define CSL_DMA_DMACH2TC1_SRCADRMODE_SHIFT (0x0006u)
N#define CSL_DMA_DMACH2TC1_SRCADRMODE_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMACH2TC1_BURSTMODE_MASK (0x0038u)
N#define CSL_DMA_DMACH2TC1_BURSTMODE_SHIFT (0x0003u)
N#define CSL_DMA_DMACH2TC1_BURSTMODE_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMACH2TC1_SYNC_MASK (0x0004u)
N#define CSL_DMA_DMACH2TC1_SYNC_SHIFT (0x0002u)
N#define CSL_DMA_DMACH2TC1_SYNC_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMACH2TC1_SRCDATASIZE_MASK (0x0003u)
N#define CSL_DMA_DMACH2TC1_SRCDATASIZE_SHIFT (0x0000u)
N#define CSL_DMA_DMACH2TC1_SRCDATASIZE_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMACH2TC1_RESETVAL (0x0000u)
N
N/* DMACH3SADR0 */
N
N#define CSL_DMA_DMACH3SADR0_LSWADDR_MASK (0xFFFFu)
N#define CSL_DMA_DMACH3SADR0_LSWADDR_SHIFT (0x0000u)
N#define CSL_DMA_DMACH3SADR0_LSWADDR_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMACH3SADR0_RESETVAL (0x0000u)
N
N/* DMACH3SADR1 */
N
N#define CSL_DMA_DMACH3SADR1_MSWADDR_MASK (0xFFFFu)
N#define CSL_DMA_DMACH3SADR1_MSWADDR_SHIFT (0x0000u)
N#define CSL_DMA_DMACH3SADR1_MSWADDR_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMACH3SADR1_RESETVAL (0x0000u)
N
N/* DMACH3DADR0 */
N
N#define CSL_DMA_DMACH3DADR0_LSWADDR_MASK (0xFFFFu)
N#define CSL_DMA_DMACH3DADR0_LSWADDR_SHIFT (0x0000u)
N#define CSL_DMA_DMACH3DADR0_LSWADDR_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMACH3DADR0_RESETVAL (0x0000u)
N
N/* DMACH3DADR1 */
N
N#define CSL_DMA_DMACH3DADR1_MSWADDR_MASK (0xFFFFu)
N#define CSL_DMA_DMACH3DADR1_MSWADDR_SHIFT (0x0000u)
N#define CSL_DMA_DMACH3DADR1_MSWADDR_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMACH3DADR1_RESETVAL (0x0000u)
N
N/* DMACH3TC0 */
N
N#define CSL_DMA_DMACH3TC0_TXLEN_MASK (0xFFFFu)
N#define CSL_DMA_DMACH3TC0_TXLEN_SHIFT (0x0000u)
N#define CSL_DMA_DMACH3TC0_TXLEN_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMACH3TC0_RESETVAL (0x0000u)
N
N/* DMACH3TC1 */
N
N#define CSL_DMA_DMACH3TC1_DMASTART_MASK (0x8000u)
N#define CSL_DMA_DMACH3TC1_DMASTART_SHIFT (0x000Fu)
N#define CSL_DMA_DMACH3TC1_DMASTART_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMACH3TC1_DMASTATUS_MASK (0x4000u)
N#define CSL_DMA_DMACH3TC1_DMASTATUS_SHIFT (0x000Eu)
N#define CSL_DMA_DMACH3TC1_DMASTATUS_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMACH3TC1_INTEN_MASK (0x2000u)
N#define CSL_DMA_DMACH3TC1_INTEN_SHIFT (0x000Du)
N#define CSL_DMA_DMACH3TC1_INTEN_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMACH3TC1_AUTORELOAD_MASK (0x1000u)
N#define CSL_DMA_DMACH3TC1_AUTORELOAD_SHIFT (0x000Cu)
N#define CSL_DMA_DMACH3TC1_AUTORELOAD_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMACH3TC1_DESTDATASIZE_MASK (0x0C00u)
N#define CSL_DMA_DMACH3TC1_DESTDATASIZE_SHIFT (0x000Au)
N#define CSL_DMA_DMACH3TC1_DESTDATASIZE_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMACH3TC1_DESTADRMODE_MASK (0x0300u)
N#define CSL_DMA_DMACH3TC1_DESTADRMODE_SHIFT (0x0008u)
N#define CSL_DMA_DMACH3TC1_DESTADRMODE_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMACH3TC1_SRCADRMODE_MASK (0x00C0u)
N#define CSL_DMA_DMACH3TC1_SRCADRMODE_SHIFT (0x0006u)
N#define CSL_DMA_DMACH3TC1_SRCADRMODE_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMACH3TC1_BURSTMODE_MASK (0x0038u)
N#define CSL_DMA_DMACH3TC1_BURSTMODE_SHIFT (0x0003u)
N#define CSL_DMA_DMACH3TC1_BURSTMODE_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMACH3TC1_SYNC_MASK (0x0004u)
N#define CSL_DMA_DMACH3TC1_SYNC_SHIFT (0x0002u)
N#define CSL_DMA_DMACH3TC1_SYNC_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMACH3TC1_SRCDATASIZE_MASK (0x0003u)
N#define CSL_DMA_DMACH3TC1_SRCDATASIZE_SHIFT (0x0000u)
N#define CSL_DMA_DMACH3TC1_SRCDATASIZE_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMACH3TC1_RESETVAL (0x0000u)
N
N#endif
L 65 "../common_inc/corazon.h" 2
N#include "cslr_dmaevtint_001.h"
L 1 "..\common_inc\cslr_dmaevtint_001.h" 1
N/*****************************************************************************
N * File Name : cslr_dmaevtint_001.h 
N *
N * Brief	 : Define DMA EVENT register structure
N *
N * Copyright (C) 2009 -2010 Texas Instruments Incorporated - http://www.ti.com/ 
N * 
N * 
N *  Redistribution and use in source and binary forms, with or without 
N *  modification, are permitted provided that the following conditions 
N *  are met:
N *
N *    Redistributions of source code must retain the above copyright 
N *    notice, this list of conditions and the following disclaimer.
N *
N *    Redistributions in binary form must reproduce the above copyright
N *    notice, this list of conditions and the following disclaimer in the 
N *    documentation and/or other materials provided with the   
N *    distribution.
N *
N *    Neither the name of Texas Instruments Incorporated nor the names of
N *    its contributors may be used to endorse or promote products derived
N *    from this software without specific prior written permission.
N *
N *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
N *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
N *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
N *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
N *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
N *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
N *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
N *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
N *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
N *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
N *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
N *
N******************************************************************************/
N#ifndef _CSLR__DMA_1_H_
N#define _CSLR__DMA_1_H_
N
N#include <cslr.h>
N
N/* Minimum unit = 2 bytes */
N
N/**************************************************************************\
N* Register Overlay Structure
X
N\**************************************************************************/
Ntypedef struct  {
N    volatile Uint16 DMA0EVT0;
N    volatile Uint16 DMA0EVT1;
N    volatile Uint16 DMA1EVT0;
N    volatile Uint16 DMA1EVT1;
N    volatile Uint16 RSVD0[18];
N    volatile Uint16 DMAIFR;
N    volatile Uint16 DMAINTEN;
N    volatile Uint16 RSVD1[4];
N    volatile Uint16 DMA2EVT0;
N    volatile Uint16 DMA2EVT1;
N    volatile Uint16 DMA3EVT0;
N    volatile Uint16 DMA3EVT1;
N} CSL_DmaEvtRegs;
N
N/**************************************************************************\
N* Field Definition Macros
X
N\**************************************************************************/
N
N/* DMA0EVT0 */
N
N
N#define CSL_DMA_DMA0EVT0_CH1EVENT_MASK (0x0F00u)
N#define CSL_DMA_DMA0EVT0_CH1EVENT_SHIFT (0x0008u)
N#define CSL_DMA_DMA0EVT0_CH1EVENT_RESETVAL (0x0000u)
N
N
N#define CSL_DMA_DMA0EVT0_CH0EVENT_MASK (0x000Fu)
N#define CSL_DMA_DMA0EVT0_CH0EVENT_SHIFT (0x0000u)
N#define CSL_DMA_DMA0EVT0_CH0EVENT_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMA0EVT0_RESETVAL (0x0000u)
N
N/* DMA0EVT1 */
N
N
N#define CSL_DMA_DMA0EVT1_CH3EVENT_MASK (0x0F00u)
N#define CSL_DMA_DMA0EVT1_CH3EVENT_SHIFT (0x0008u)
N#define CSL_DMA_DMA0EVT1_CH3EVENT_RESETVAL (0x0000u)
N
N
N#define CSL_DMA_DMA0EVT1_CH2EVENT_MASK (0x000Fu)
N#define CSL_DMA_DMA0EVT1_CH2EVENT_SHIFT (0x0000u)
N#define CSL_DMA_DMA0EVT1_CH2EVENT_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMA0EVT1_RESETVAL (0x0000u)
N
N/* DMA1EVT0 */
N
N
N#define CSL_DMA_DMA1EVT0_CH1EVENT_MASK (0x0F00u)
N#define CSL_DMA_DMA1EVT0_CH1EVENT_SHIFT (0x0008u)
N#define CSL_DMA_DMA1EVT0_CH1EVENT_RESETVAL (0x0000u)
N
N
N#define CSL_DMA_DMA1EVT0_CH0EVENT_MASK (0x000Fu)
N#define CSL_DMA_DMA1EVT0_CH0EVENT_SHIFT (0x0000u)
N#define CSL_DMA_DMA1EVT0_CH0EVENT_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMA1EVT0_RESETVAL (0x0000u)
N
N/* DMA1EVT1 */
N
N
N#define CSL_DMA_DMA1EVT1_CH3EVENT_MASK (0x0F00u)
N#define CSL_DMA_DMA1EVT1_CH3EVENT_SHIFT (0x0008u)
N#define CSL_DMA_DMA1EVT1_CH3EVENT_RESETVAL (0x0000u)
N
N
N#define CSL_DMA_DMA1EVT1_CH2EVENT_MASK (0x000Fu)
N#define CSL_DMA_DMA1EVT1_CH2EVENT_SHIFT (0x0000u)
N#define CSL_DMA_DMA1EVT1_CH2EVENT_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMA1EVT1_RESETVAL (0x0000u)
N
N/* DMAIFR */
N
N#define CSL_DMA_DMAIFR_DMA3CH3INT_MASK (0x8000u)
N#define CSL_DMA_DMAIFR_DMA3CH3INT_SHIFT (0x000Fu)
N#define CSL_DMA_DMAIFR_DMA3CH3INT_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMAIFR_DMA3CH2INT_MASK (0x4000u)
N#define CSL_DMA_DMAIFR_DMA3CH2INT_SHIFT (0x000Eu)
N#define CSL_DMA_DMAIFR_DMA3CH2INT_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMAIFR_DMA3CH1INT_MASK (0x2000u)
N#define CSL_DMA_DMAIFR_DMA3CH1INT_SHIFT (0x000Du)
N#define CSL_DMA_DMAIFR_DMA3CH1INT_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMAIFR_DMA3CH0INT_MASK (0x1000u)
N#define CSL_DMA_DMAIFR_DMA3CH0INT_SHIFT (0x000Cu)
N#define CSL_DMA_DMAIFR_DMA3CH0INT_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMAIFR_DMA2CH3INT_MASK (0x0800u)
N#define CSL_DMA_DMAIFR_DMA2CH3INT_SHIFT (0x000Bu)
N#define CSL_DMA_DMAIFR_DMA2CH3INT_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMAIFR_DMA2CH2INT_MASK (0x0400u)
N#define CSL_DMA_DMAIFR_DMA2CH2INT_SHIFT (0x000Au)
N#define CSL_DMA_DMAIFR_DMA2CH2INT_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMAIFR_DMA2CH1INT_MASK (0x0200u)
N#define CSL_DMA_DMAIFR_DMA2CH1INT_SHIFT (0x0009u)
N#define CSL_DMA_DMAIFR_DMA2CH1INT_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMAIFR_DMA2CH0INT_MASK (0x0100u)
N#define CSL_DMA_DMAIFR_DMA2CH0INT_SHIFT (0x0008u)
N#define CSL_DMA_DMAIFR_DMA2CH0INT_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMAIFR_DMA1CH3INT_MASK (0x0080u)
N#define CSL_DMA_DMAIFR_DMA1CH3INT_SHIFT (0x0007u)
N#define CSL_DMA_DMAIFR_DMA1CH3INT_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMAIFR_DMA1CH2INT_MASK (0x0040u)
N#define CSL_DMA_DMAIFR_DMA1CH2INT_SHIFT (0x0006u)
N#define CSL_DMA_DMAIFR_DMA1CH2INT_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMAIFR_DMA1CH1INT_MASK (0x0020u)
N#define CSL_DMA_DMAIFR_DMA1CH1INT_SHIFT (0x0005u)
N#define CSL_DMA_DMAIFR_DMA1CH1INT_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMAIFR_DMA1CH0INT_MASK (0x0010u)
N#define CSL_DMA_DMAIFR_DMA1CH0INT_SHIFT (0x0004u)
N#define CSL_DMA_DMAIFR_DMA1CH0INT_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMAIFR_DMA0CH3INT_MASK (0x0008u)
N#define CSL_DMA_DMAIFR_DMA0CH3INT_SHIFT (0x0003u)
N#define CSL_DMA_DMAIFR_DMA0CH3INT_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMAIFR_DMA0CH2INT_MASK (0x0004u)
N#define CSL_DMA_DMAIFR_DMA0CH2INT_SHIFT (0x0002u)
N#define CSL_DMA_DMAIFR_DMA0CH2INT_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMAIFR_DMA0CH1INT_MASK (0x0002u)
N#define CSL_DMA_DMAIFR_DMA0CH1INT_SHIFT (0x0001u)
N#define CSL_DMA_DMAIFR_DMA0CH1INT_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMAIFR_DMA0CH0INT_MASK (0x0001u)
N#define CSL_DMA_DMAIFR_DMA0CH0INT_SHIFT (0x0000u)
N#define CSL_DMA_DMAIFR_DMA0CH0INT_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMAIFR_RESETVAL (0x0000u)
N
N/* DMAINTEN */
N
N#define CSL_DMA_DMAINTEN_DMA3CH3INTEN_MASK (0x8000u)
N#define CSL_DMA_DMAINTEN_DMA3CH3INTEN_SHIFT (0x000Fu)
N#define CSL_DMA_DMAINTEN_DMA3CH3INTEN_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMAINTEN_DMA3CH2INTEN_MASK (0x4000u)
N#define CSL_DMA_DMAINTEN_DMA3CH2INTEN_SHIFT (0x000Eu)
N#define CSL_DMA_DMAINTEN_DMA3CH2INTEN_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMAINTEN_DMA3CH1INTEN_MASK (0x2000u)
N#define CSL_DMA_DMAINTEN_DMA3CH1INTEN_SHIFT (0x000Du)
N#define CSL_DMA_DMAINTEN_DMA3CH1INTEN_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMAINTEN_DMA3CH0INTEN_MASK (0x1000u)
N#define CSL_DMA_DMAINTEN_DMA3CH0INTEN_SHIFT (0x000Cu)
N#define CSL_DMA_DMAINTEN_DMA3CH0INTEN_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMAINTEN_DMA2CH3INTEN_MASK (0x0800u)
N#define CSL_DMA_DMAINTEN_DMA2CH3INTEN_SHIFT (0x000Bu)
N#define CSL_DMA_DMAINTEN_DMA2CH3INTEN_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMAINTEN_DMA2CH2INTEN_MASK (0x0400u)
N#define CSL_DMA_DMAINTEN_DMA2CH2INTEN_SHIFT (0x000Au)
N#define CSL_DMA_DMAINTEN_DMA2CH2INTEN_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMAINTEN_DMA2CH1INTEN_MASK (0x0200u)
N#define CSL_DMA_DMAINTEN_DMA2CH1INTEN_SHIFT (0x0009u)
N#define CSL_DMA_DMAINTEN_DMA2CH1INTEN_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMAINTEN_DMA2CH0INTEN_MASK (0x0100u)
N#define CSL_DMA_DMAINTEN_DMA2CH0INTEN_SHIFT (0x0008u)
N#define CSL_DMA_DMAINTEN_DMA2CH0INTEN_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMAINTEN_DMA1CH3INTEN_MASK (0x0080u)
N#define CSL_DMA_DMAINTEN_DMA1CH3INTEN_SHIFT (0x0007u)
N#define CSL_DMA_DMAINTEN_DMA1CH3INTEN_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMAINTEN_DMA1CH2INTEN_MASK (0x0040u)
N#define CSL_DMA_DMAINTEN_DMA1CH2INTEN_SHIFT (0x0006u)
N#define CSL_DMA_DMAINTEN_DMA1CH2INTEN_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMAINTEN_DMA1CH1INTEN_MASK (0x0020u)
N#define CSL_DMA_DMAINTEN_DMA1CH1INTEN_SHIFT (0x0005u)
N#define CSL_DMA_DMAINTEN_DMA1CH1INTEN_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMAINTEN_DMA1CH0INTEN_MASK (0x0010u)
N#define CSL_DMA_DMAINTEN_DMA1CH0INTEN_SHIFT (0x0004u)
N#define CSL_DMA_DMAINTEN_DMA1CH0INTEN_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMAINTEN_DMA0CH3INTEN_MASK (0x0008u)
N#define CSL_DMA_DMAINTEN_DMA0CH3INTEN_SHIFT (0x0003u)
N#define CSL_DMA_DMAINTEN_DMA0CH3INTEN_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMAINTEN_DMA0CH2INTEN_MASK (0x0004u)
N#define CSL_DMA_DMAINTEN_DMA0CH2INTEN_SHIFT (0x0002u)
N#define CSL_DMA_DMAINTEN_DMA0CH2INTEN_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMAINTEN_DMA0CH1INTEN_MASK (0x0002u)
N#define CSL_DMA_DMAINTEN_DMA0CH1INTEN_SHIFT (0x0001u)
N#define CSL_DMA_DMAINTEN_DMA0CH1INTEN_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMAINTEN_DMA0CH0INTEN_MASK (0x0001u)
N#define CSL_DMA_DMAINTEN_DMA0CH0INTEN_SHIFT (0x0000u)
N#define CSL_DMA_DMAINTEN_DMA0CH0INTEN_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMAINTEN_RESETVAL (0x0000u)
N
N/* DMA2EVT0 */
N
N
N#define CSL_DMA_DMA2EVT0_CH1EVENT_MASK (0x0F00u)
N#define CSL_DMA_DMA2EVT0_CH1EVENT_SHIFT (0x0008u)
N#define CSL_DMA_DMA2EVT0_CH1EVENT_RESETVAL (0x0000u)
N
N
N#define CSL_DMA_DMA2EVT0_CH0EVENT_MASK (0x000Fu)
N#define CSL_DMA_DMA2EVT0_CH0EVENT_SHIFT (0x0000u)
N#define CSL_DMA_DMA2EVT0_CH0EVENT_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMA2EVT0_RESETVAL (0x0000u)
N
N/* DMA2EVT1 */
N
N
N#define CSL_DMA_DMA2EVT1_CH3EVENT_MASK (0x0F00u)
N#define CSL_DMA_DMA2EVT1_CH3EVENT_SHIFT (0x0008u)
N#define CSL_DMA_DMA2EVT1_CH3EVENT_RESETVAL (0x0000u)
N
N
N#define CSL_DMA_DMA2EVT1_CH2EVENT_MASK (0x000Fu)
N#define CSL_DMA_DMA2EVT1_CH2EVENT_SHIFT (0x0000u)
N#define CSL_DMA_DMA2EVT1_CH2EVENT_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMA2EVT1_RESETVAL (0x0000u)
N
N/* DMA3EVT0 */
N
N
N#define CSL_DMA_DMA3EVT0_CH1EVENT_MASK (0x0F00u)
N#define CSL_DMA_DMA3EVT0_CH1EVENT_SHIFT (0x0008u)
N#define CSL_DMA_DMA3EVT0_CH1EVENT_RESETVAL (0x0000u)
N
N
N#define CSL_DMA_DMA3EVT0_CH0EVENT_MASK (0x000Fu)
N#define CSL_DMA_DMA3EVT0_CH0EVENT_SHIFT (0x0000u)
N#define CSL_DMA_DMA3EVT0_CH0EVENT_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMA3EVT0_RESETVAL (0x0000u)
N
N/* DMA3EVT1 */
N
N
N#define CSL_DMA_DMA3EVT1_CH3EVENT_MASK (0x0F00u)
N#define CSL_DMA_DMA3EVT1_CH3EVENT_SHIFT (0x0008u)
N#define CSL_DMA_DMA3EVT1_CH3EVENT_RESETVAL (0x0000u)
N
N
N#define CSL_DMA_DMA3EVT1_CH2EVENT_MASK (0x000Fu)
N#define CSL_DMA_DMA3EVT1_CH2EVENT_SHIFT (0x0000u)
N#define CSL_DMA_DMA3EVT1_CH2EVENT_RESETVAL (0x0000u)
N
N#define CSL_DMA_DMA3EVT1_RESETVAL (0x0000u)
N
N#endif
N
N
N
L 66 "../common_inc/corazon.h" 2
N#include "cslr_sar_001.h"
L 1 "..\common_inc\cslr_sar_001.h" 1
N/*****************************************************************************
N * File Name : cslr_sar_001.h 
N *
N * Brief	 : Define SAR register structure
N *
N * Copyright (C) 2009 -2010 Texas Instruments Incorporated - http://www.ti.com/ 
N * 
N * 
N *  Redistribution and use in source and binary forms, with or without 
N *  modification, are permitted provided that the following conditions 
N *  are met:
N *
N *    Redistributions of source code must retain the above copyright 
N *    notice, this list of conditions and the following disclaimer.
N *
N *    Redistributions in binary form must reproduce the above copyright
N *    notice, this list of conditions and the following disclaimer in the 
N *    documentation and/or other materials provided with the   
N *    distribution.
N *
N *    Neither the name of Texas Instruments Incorporated nor the names of
N *    its contributors may be used to endorse or promote products derived
N *    from this software without specific prior written permission.
N *
N *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
N *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
N *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
N *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
N *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
N *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
N *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
N *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
N *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
N *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
N *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
N *
N******************************************************************************/
N#ifndef _CSLR__SAR_1_H_
N#define _CSLR__SAR_1_H_
N
N#include <cslr.h>
N/* Minimum unit = 2 bytes */
N
N/**************************************************************************\
N* Register Overlay Structure
X
N\**************************************************************************/
Ntypedef struct  {
N    volatile Uint16 RSVD0[18];
N    volatile Uint16 SARCTRL;
N    volatile Uint16 RSVD1;
N    volatile Uint16 SARDATA;
N    volatile Uint16 RSVD2;
N    volatile Uint16 SARCLKCTRL;
N    volatile Uint16 RSVD3;
N    volatile Uint16 SARPINCTRL;
N    volatile Uint16 RSVD4;
N    volatile Uint16 SARGPOCTRL;
N} CSL_SarRegs;
N
N/**************************************************************************\
N* Field Definition Macros
X
N\**************************************************************************/
N
N/* SARCTRL */
N
N#define CSL_SAR_SARCTRL_ADCSTART_MASK (0x8000u)
N#define CSL_SAR_SARCTRL_ADCSTART_SHIFT (0x000Fu)
N#define CSL_SAR_SARCTRL_ADCSTART_RESETVAL (0x0000u)
N
N#define CSL_SAR_SARCTRL_CHANSEL_MASK (0x7000u)
N#define CSL_SAR_SARCTRL_CHANSEL_SHIFT (0x000Cu)
N#define CSL_SAR_SARCTRL_CHANSEL_RESETVAL (0x0000u)
N
N#define CSL_SAR_SARCTRL_MULTICHANNEL_MASK (0x0800u)
N#define CSL_SAR_SARCTRL_MULTICHANNEL_SHIFT (0x000Bu)
N#define CSL_SAR_SARCTRL_MULTICHANNEL_RESETVAL (0x0000u)
N
N#define CSL_SAR_SARCTRL_SINGLECYCLE_MASK (0x0400u)
N#define CSL_SAR_SARCTRL_SINGLECYCLE_SHIFT (0x000Au)
N#define CSL_SAR_SARCTRL_SINGLECYCLE_RESETVAL (0x0000u)
N
N#define CSL_SAR_SARCTRL_TESTDATA_MASK (0x03FFu)
N#define CSL_SAR_SARCTRL_TESTDATA_SHIFT (0x0000u)
N#define CSL_SAR_SARCTRL_TESTDATA_RESETVAL (0x0000u)
N
N#define CSL_SAR_SARCTRL_RESETVAL (0x0000u)
N
N/* SARDATA */
N
N#define CSL_SAR_SARDATA_ADCBSY_MASK (0x8000u)
N#define CSL_SAR_SARDATA_ADCBSY_SHIFT (0x000Fu)
N#define CSL_SAR_SARDATA_ADCBSY_RESETVAL (0x0000u)
N
N#define CSL_SAR_SARDATA_CHANSEL_MASK (0x7000u)
N#define CSL_SAR_SARDATA_CHANSEL_SHIFT (0x000Cu)
N#define CSL_SAR_SARDATA_CHANSEL_RESETVAL (0x0000u)
N
N
N#define CSL_SAR_SARDATA_ADCDATA_MASK (0x03FFu)
N#define CSL_SAR_SARDATA_ADCDATA_SHIFT (0x0000u)
N#define CSL_SAR_SARDATA_ADCDATA_RESETVAL (0x0000u)
N
N#define CSL_SAR_SARDATA_RESETVAL (0x0000u)
N
N/* SARCLKCTRL */
N
N
N#define CSL_SAR_SARCLKCTRL_CLKDIV_MASK (0x7FFFu)
N#define CSL_SAR_SARCLKCTRL_CLKDIV_SHIFT (0x0000u)
N#define CSL_SAR_SARCLKCTRL_CLKDIV_RESETVAL (0x0000u)
N
N#define CSL_SAR_SARCLKCTRL_RESETVAL (0x0000u)
N
N/* SARPINCTRL */
N
N#define CSL_SAR_SARPINCTRL_TESTCOMPOUT_MASK (0x8000u)
N#define CSL_SAR_SARPINCTRL_TESTCOMPOUT_SHIFT (0x000Fu)
N#define CSL_SAR_SARPINCTRL_TESTCOMPOUT_RESETVAL (0x0000u)
N
N#define CSL_SAR_SARPINCTRL_STATUSMASK_MASK (0x4000u)
N#define CSL_SAR_SARPINCTRL_STATUSMASK_SHIFT (0x000Eu)
N#define CSL_SAR_SARPINCTRL_STATUSMASK_RESETVAL (0x0000u)
N
N#define CSL_SAR_SARPINCTRL_PWRUPBIAS_MASK (0x2000u)
N#define CSL_SAR_SARPINCTRL_PWRUPBIAS_SHIFT (0x000Du)
N#define CSL_SAR_SARPINCTRL_PWRUPBIAS_RESETVAL (0x0000u)
N
N#define CSL_SAR_SARPINCTRL_SARPWRUP_MASK (0x1000u)
N#define CSL_SAR_SARPINCTRL_SARPWRUP_SHIFT (0x000Cu)
N#define CSL_SAR_SARPINCTRL_SARPWRUP_RESETVAL (0x0000u)
N
N
N#define CSL_SAR_SARPINCTRL_REFBUFFEN_MASK (0x0400u)
N#define CSL_SAR_SARPINCTRL_REFBUFFEN_SHIFT (0x000Au)
N#define CSL_SAR_SARPINCTRL_REFBUFFEN_RESETVAL (0x0000u)
N
N#define CSL_SAR_SARPINCTRL_REFLVSEL_MASK (0x0200u)
N#define CSL_SAR_SARPINCTRL_REFLVSEL_SHIFT (0x0009u)
N#define CSL_SAR_SARPINCTRL_REFLVSEL_RESETVAL (0x0000u)
N
N#define CSL_SAR_SARPINCTRL_REFAVDDSEL_MASK (0x0100u)
N#define CSL_SAR_SARPINCTRL_REFAVDDSEL_SHIFT (0x0008u)
N#define CSL_SAR_SARPINCTRL_REFAVDDSEL_RESETVAL (0x0000u)
N
N
N#define CSL_SAR_SARPINCTRL_SERVOTESTMODE_MASK (0x0020u)
N#define CSL_SAR_SARPINCTRL_SERVOTESTMODE_SHIFT (0x0005u)
N#define CSL_SAR_SARPINCTRL_SERVOTESTMODE_RESETVAL (0x0000u)
N
N#define CSL_SAR_SARPINCTRL_TOUCHSCREENMODE_MASK (0x0010u)
N#define CSL_SAR_SARPINCTRL_TOUCHSCREENMODE_SHIFT (0x0004u)
N#define CSL_SAR_SARPINCTRL_TOUCHSCREENMODE_RESETVAL (0x0000u)
N
N#define CSL_SAR_SARPINCTRL_AVDDMEAS_MASK (0x0008u)
N#define CSL_SAR_SARPINCTRL_AVDDMEAS_SHIFT (0x0003u)
N#define CSL_SAR_SARPINCTRL_AVDDMEAS_RESETVAL (0x0000u)
N
N#define CSL_SAR_SARPINCTRL_NOHV_MASK (0x0004u)
N#define CSL_SAR_SARPINCTRL_NOHV_SHIFT (0x0002u)
N#define CSL_SAR_SARPINCTRL_NOHV_RESETVAL (0x0000u)
N
N#define CSL_SAR_SARPINCTRL_GNDON_MASK (0x0002u)
N#define CSL_SAR_SARPINCTRL_GNDON_SHIFT (0x0001u)
N#define CSL_SAR_SARPINCTRL_GNDON_RESETVAL (0x0000u)
N
N#define CSL_SAR_SARPINCTRL_HALF_MASK (0x0001u)
N#define CSL_SAR_SARPINCTRL_HALF_SHIFT (0x0000u)
N#define CSL_SAR_SARPINCTRL_HALF_RESETVAL (0x0000u)
N
N#define CSL_SAR_SARPINCTRL_RESETVAL (0x0000u)
N
N/* SARGPOCTRL */
N
N
N#define CSL_SAR_SARGPOCTRL_PENIRQSTAT_MASK (0x0200u)
N#define CSL_SAR_SARGPOCTRL_PENIRQSTAT_SHIFT (0x0009u)
N#define CSL_SAR_SARGPOCTRL_PENIRQSTAT_RESETVAL (0x0000u)
N
N#define CSL_SAR_SARGPOCTRL_PENIRQEN_MASK (0x0100u)
N#define CSL_SAR_SARGPOCTRL_PENIRQEN_SHIFT (0x0008u)
N#define CSL_SAR_SARGPOCTRL_PENIRQEN_RESETVAL (0x0000u)
N
N#define CSL_SAR_SARGPOCTRL_GPO3EN_MASK (0x0080u)
N#define CSL_SAR_SARGPOCTRL_GPO3EN_SHIFT (0x0007u)
N#define CSL_SAR_SARGPOCTRL_GPO3EN_RESETVAL (0x0000u)
N
N#define CSL_SAR_SARGPOCTRL_GPO2EN_MASK (0x0040u)
N#define CSL_SAR_SARGPOCTRL_GPO2EN_SHIFT (0x0006u)
N#define CSL_SAR_SARGPOCTRL_GPO2EN_RESETVAL (0x0000u)
N
N#define CSL_SAR_SARGPOCTRL_GPO1EN_MASK (0x0020u)
N#define CSL_SAR_SARGPOCTRL_GPO1EN_SHIFT (0x0005u)
N#define CSL_SAR_SARGPOCTRL_GPO1EN_RESETVAL (0x0000u)
N
N#define CSL_SAR_SARGPOCTRL_GPO0EN_MASK (0x0010u)
N#define CSL_SAR_SARGPOCTRL_GPO0EN_SHIFT (0x0004u)
N#define CSL_SAR_SARGPOCTRL_GPO0EN_RESETVAL (0x0000u)
N
N#define CSL_SAR_SARGPOCTRL_GPO3_MASK (0x0008u)
N#define CSL_SAR_SARGPOCTRL_GPO3_SHIFT (0x0003u)
N#define CSL_SAR_SARGPOCTRL_GPO3_RESETVAL (0x0000u)
N
N#define CSL_SAR_SARGPOCTRL_GPO2_MASK (0x0004u)
N#define CSL_SAR_SARGPOCTRL_GPO2_SHIFT (0x0002u)
N#define CSL_SAR_SARGPOCTRL_GPO2_RESETVAL (0x0000u)
N
N#define CSL_SAR_SARGPOCTRL_GPO1_MASK (0x0002u)
N#define CSL_SAR_SARGPOCTRL_GPO1_SHIFT (0x0001u)
N#define CSL_SAR_SARGPOCTRL_GPO1_RESETVAL (0x0000u)
N
N#define CSL_SAR_SARGPOCTRL_GPO0_MASK (0x0001u)
N#define CSL_SAR_SARGPOCTRL_GPO0_SHIFT (0x0000u)
N#define CSL_SAR_SARGPOCTRL_GPO0_RESETVAL (0x0000u)
N
N#define CSL_SAR_SARGPOCTRL_RESETVAL (0x0000u)
N
N#endif
L 67 "../common_inc/corazon.h" 2
N#include "cslr_usb_001.h"
L 1 "..\common_inc\cslr_usb_001.h" 1
N/*****************************************************************************
N * File Name : cslr_usb_001.h 
N *
N * Brief	 : Define UART register structure
N *
N * Copyright (C) 2009 -2010 Texas Instruments Incorporated - http://www.ti.com/ 
N * 
N * 
N *  Redistribution and use in source and binary forms, with or without 
N *  modification, are permitted provided that the following conditions 
N *  are met:
N *
N *    Redistributions of source code must retain the above copyright 
N *    notice, this list of conditions and the following disclaimer.
N *
N *    Redistributions in binary form must reproduce the above copyright
N *    notice, this list of conditions and the following disclaimer in the 
N *    documentation and/or other materials provided with the   
N *    distribution.
N *
N *    Neither the name of Texas Instruments Incorporated nor the names of
N *    its contributors may be used to endorse or promote products derived
N *    from this software without specific prior written permission.
N *
N *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
N *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
N *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
N *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
N *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
N *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
N *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
N *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
N *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
N *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
N *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
N *
N******************************************************************************/
N#ifndef _CSLR__USB_1_H_
N#define _CSLR__USB_1_H_
N
N//#include <cslr.h>
N
N
N
N#include <tistdtypes.h>
N
N//#define CSL_USB_REGS                    ((CSL_UsbRegsOvly)0x8000)
N
N
N/* Minimum unit = 2 bytes */
N
N/**************************************************************************\
N* Register Overlay Structure for EPTRG
X
N\**************************************************************************/
Ntypedef struct  {
N    volatile Uint16 TXFUNCADDR;
N    volatile Uint16 TXHUBADDR_PORT;
N    volatile Uint16 RSVD0[2];
N    volatile Uint16 RXFUNCADDR;
N    volatile Uint16 RXHUBADDR_PORT;
N    volatile Uint16 RSVD35[2];
N} CSL_UsbEptrgRegs;
N
N/**************************************************************************\
N* Register Overlay Structure for EPCSR
X
N\**************************************************************************/
Ntypedef struct  {
N    volatile Uint16 TXMAXP;
N    volatile Uint16 PERI_CSR0;
N    volatile Uint16 RSVD0[2];
N    volatile Uint16 RXMAXP;
N    volatile Uint16 PERI_RXCSR;
N    volatile Uint16 RSVD1[2];
N    volatile Uint16 COUNT0;
N    volatile Uint16 RSVD2[4];
N    volatile Uint16 CONFIGDATA_FIFOSIZE;
N    volatile Uint16 RSVD37[2];
N} CSL_UsbEpcsrRegs;
N
N/**************************************************************************\
N* Register Overlay Structure for CHANNEL
X
N\**************************************************************************/
Ntypedef struct  {
N    volatile Uint16 TXCHGCFGRL;
N    volatile Uint16 TXCHGCFGRH;
N    volatile Uint16 RSVD0[6];
N    volatile Uint16 RXCHGCRL;
N    volatile Uint16 RXCHGCRH;
N    volatile Uint16 RSVD1[2];
N    volatile Uint16 RXCHHPCRAL;
N    volatile Uint16 RXCHHPCRAH;
N    volatile Uint16 RSVD2[2];
N    volatile Uint16 RXCHHPCRBL;
N    volatile Uint16 RXCHHPCRBH;
N    volatile Uint16 RSVD39[14];
N} CSL_UsbChannelRegs;
N
N/**************************************************************************\
N* Register Overlay Structure for CdmaScheTblWord
X
N\**************************************************************************/
Ntypedef struct  {
N    volatile Uint16 SCHETBLWNL;
N    volatile Uint16 SCHETBLWNH;
N    volatile Uint16 RSVD42[2];
N} CSL_UsbCdmaschetblwordRegs;
N
N/**************************************************************************\
N* Register Overlay Structure for QMMemRegR
X
N\**************************************************************************/
Ntypedef struct  {
N    volatile Uint16 QMMRRBARL;
N    volatile Uint16 QMMRRBARH;
N    volatile Uint16 RSVD0[2];
N    volatile Uint16 QMMRRCRL;
N    volatile Uint16 QMMRRCRH;
N    volatile Uint16 RSVD59[10];
N} CSL_UsbQmmemregrRegs;
N
N/**************************************************************************\
N* Register Overlay Structure for QMQN
X
N\**************************************************************************/
Ntypedef struct  {
N    volatile Uint16 QMMQNRAL;
N    volatile Uint16 QMMQNRAH;
N    volatile Uint16 RSVD0[2];
N    volatile Uint16 QMMQNRBL;
N    volatile Uint16 QMMQNRBH;
N    volatile Uint16 RSVD1[2];
N    volatile Uint16 QMMQNRCL;
N    volatile Uint16 QMMQNRCH;
N    volatile Uint16 RSVD2[2];
N    volatile Uint16 QMMQNRDL;
N    volatile Uint16 QMMQNRDH;
N    volatile Uint16 RSVD61[2];
N} CSL_UsbQmqnRegs;
N
N/**************************************************************************\
N* Register Overlay Structure for QMQNS
X
N\**************************************************************************/
Ntypedef struct  {
N    volatile Uint16 QMMQNSRAL;
N    volatile Uint16 QMMQNSRAH;
N    volatile Uint16 RSVD0[2];
N    volatile Uint16 QMMQNSRBL;
N    volatile Uint16 QMMQNSRBH;
N    volatile Uint16 RSVD1[2];
N    volatile Uint16 QMMQNSRC;
N    volatile Uint16 RSVD63[7];
N} CSL_UsbQmqnsRegs;
N
N/**************************************************************************\
N* Register Overlay Structure
X
N\**************************************************************************/
Ntypedef struct  {
N    volatile Uint16 REVRL;
N    volatile Uint16 REVRH;
N    volatile Uint16 RSVD0[2];
N    volatile Uint16 CTRLRL;
N    volatile Uint16 CTRLRH;
N    volatile Uint16 RSVD1[2];
N    volatile Uint16 STATR;
N    volatile Uint16 RSVD2[3];
N    volatile Uint16 EMUR;
N    volatile Uint16 RSVD3[3];
N    volatile Uint16 MODERL;
N    volatile Uint16 MODERH;
N    volatile Uint16 RSVD4[2];
N    volatile Uint16 AUTOREQ;
N    volatile Uint16 RSVD5[3];
N    volatile Uint16 SRPFIXL;
N    volatile Uint16 SRPFIXH;
N    volatile Uint16 RSVD6[2];
N    volatile Uint16 RXTDOWNL;
N    volatile Uint16 RXTDOWNH;
N    volatile Uint16 RSVD7[2];
N    volatile Uint16 INTSRCRL;
N    volatile Uint16 INTSRCRH;
N    volatile Uint16 RSVD8[2];
N    volatile Uint16 INTSETRL;
N    volatile Uint16 INTSETRH;
N    volatile Uint16 RSVD9[2];
N    volatile Uint16 INTCLRRL;
N    volatile Uint16 INTCLRRH;
N    volatile Uint16 RSVD10[2];
N    volatile Uint16 INTMSKRL;
N    volatile Uint16 INTMSKRH;
N    volatile Uint16 RSVD11[2];
N    volatile Uint16 INTMSKSETRL;
N    volatile Uint16 INTMSKSETRH;
N    volatile Uint16 RSVD12[2];
N    volatile Uint16 INTMSKCLRRL;
N    volatile Uint16 INTMSKCLRRH;
N    volatile Uint16 RSVD13[2];
N    volatile Uint16 INTMASKEDRL;
N    volatile Uint16 INTMASKEDRH;
N    volatile Uint16 RSVD14[2];
N    volatile Uint16 EOIR;
N    volatile Uint16 RSVD15[3];
N    volatile Uint16 INTVECTRL;
N    volatile Uint16 INTVECTRH;
N    volatile Uint16 RSVD16[14];
N    volatile Uint16 GRNDISEP1L;
N    volatile Uint16 GRNDISEP1H;
N    volatile Uint16 RSVD17[2];
N    volatile Uint16 GRNDISEP2L;
N    volatile Uint16 GRNDISEP2H;
N    volatile Uint16 RSVD18[2];
N    volatile Uint16 GRNDISEP3L;
N    volatile Uint16 GRNDISEP3H;
N    volatile Uint16 RSVD19[2];
N    volatile Uint16 GRNDISEP4L;
N    volatile Uint16 GRNDISEP4H;
N    volatile Uint16 RSVD20[930];
N    volatile Uint16 FADDR_POWER;
N    volatile Uint16 INTRTX;
N    volatile Uint16 RSVD21[2];
N    volatile Uint16 INTRRX;
N    volatile Uint16 INTRTXE;
N    volatile Uint16 RSVD22[2];
N    volatile Uint16 INTRRXE;
N    volatile Uint16 INTRUSB_INTRUSBE;
N    volatile Uint16 RSVD23[2];
N    volatile Uint16 FRAME;
N    volatile Uint16 INDEX_TESTMODE;
N    volatile Uint16 RSVD24[2];
N    volatile Uint16 TXMAXP;
N    volatile Uint16 PERI_CSR0;
N    volatile Uint16 RSVD25[2];
N    volatile Uint16 RXMAXP;
N    volatile Uint16 PERI_RXCSR;
N    volatile Uint16 RSVD26[2];
N    volatile Uint16 COUNT0;
N    volatile Uint16 HOST_TYPE0_HOST_NAKLIMIT0;
N    volatile Uint16 RSVD27[2];
N    volatile Uint16 HOST_RXTYPE_HOST_RXINTERVAL;
N    volatile Uint16 CONFIGDATA;
N    volatile Uint16 RSVD28[2];
N    volatile Uint16 FIFO0L;
N    volatile Uint16 FIFO0H;
N    volatile Uint16 RSVD29[2];
N    volatile Uint16 FIFO1L;
N    volatile Uint16 FIFO1H;
N    volatile Uint16 RSVD30[2];
N    volatile Uint16 FIFO2L;
N    volatile Uint16 FIFO2H;
N    volatile Uint16 RSVD31[2];
N    volatile Uint16 FIFO3L;
N    volatile Uint16 FIFO3H;
N    volatile Uint16 RSVD32[2];
N    volatile Uint16 FIFO4L;
N    volatile Uint16 FIFO4H;
N    volatile Uint16 RSVD33[46];
N    volatile Uint16 DEVCTL;
N    volatile Uint16 TXRXFIFOSZ;
N    volatile Uint16 RSVD34[2];
N    volatile Uint16 TXFIFOADDR;
N    volatile Uint16 RXFIFOADDR;
N    volatile Uint16 RSVD36[26];
N    CSL_UsbEptrgRegs EPTRG[5];
N    volatile Uint16 RSVD38[88];
N    CSL_UsbEpcsrRegs EPCSR[5];
N    volatile Uint16 RSVD40[4784];
N    CSL_UsbChannelRegs CHANNEL[4];
N    volatile Uint16 RSVD41[1920];
N    volatile Uint16 SCHECTRLL;
N    volatile Uint16 SCHECTRLH;
N    volatile Uint16 RSVD43[2046];
N    CSL_UsbCdmaschetblwordRegs CDMASCHETBLWORD[64];
N    volatile Uint16 RSVD44[1792];
N    volatile Uint16 INTDRL;
N    volatile Uint16 INTDRH;
N    volatile Uint16 RSVD45[14];
N    volatile Uint16 INTDEOIR;
N    volatile Uint16 RSVD46[495];
N    volatile Uint16 INTDSTATUSR0;
N    volatile Uint16 RSVD47[3];
N    volatile Uint16 INTDSTATUSR1L;
N    volatile Uint16 INTDSTATUSR1H;
N    volatile Uint16 RSVD48[3578];
N    volatile Uint16 QMRRL;
N    volatile Uint16 RSVD49[7];
N    volatile Uint16 QMQRRL;
N    volatile Uint16 QMQRRH;
N    volatile Uint16 RSVD50[22];
N    volatile Uint16 QMFDBSCR0L;
N    volatile Uint16 QMFDBSCR0H;
N    volatile Uint16 RSVD51[2];
N    volatile Uint16 QMFDBSCR1L;
N    volatile Uint16 QMFDBSCR1H;
N    volatile Uint16 RSVD52[2];
N    volatile Uint16 QMFDBSCR2L;
N    volatile Uint16 QMFDBSCR2H;
N    volatile Uint16 RSVD53[2];
N    volatile Uint16 QMFDBSCR3L;
N    volatile Uint16 QMFDBSCR3H;
N    volatile Uint16 RSVD54[82];
N    volatile Uint16 QMLRR0BARL;
N    volatile Uint16 QMLRR0BARH;
N    volatile Uint16 RSVD55[2];
N    volatile Uint16 QMLRR0SRL;
N    volatile Uint16 QMLRR0SRH;
N    volatile Uint16 RSVD56[2];
N    volatile Uint16 QMLRR1BARL;
N    volatile Uint16 QMLRR1BARH;
N    volatile Uint16 RSVD57[6];
N    volatile Uint16 QMQPR0L;
N    volatile Uint16 QMQPR0H;
N    volatile Uint16 RSVD58[2];
N    volatile Uint16 QMQPR1L;
N    volatile Uint16 QMQPR1H;
N    volatile Uint16 RSVD60[3946];
N    CSL_UsbQmmemregrRegs QMMEMREGR[16];
N    volatile Uint16 RSVD62[3840];
N    CSL_UsbQmqnRegs QMQN[24];
N    volatile Uint16 RSVD64[1664];
N    CSL_UsbQmqnsRegs QMQNS[24];
N} CSL_UsbRegs;
N
N/**************************************************************************\
N* Field Definition Macros
X
N\**************************************************************************/
N
N/* TXFUNCADDR */
N
N
N#define CSL_USB_TXFUNCADDR_FUNCADDR_MASK (0x007Fu)
N#define CSL_USB_TXFUNCADDR_FUNCADDR_SHIFT (0x0000u)
N#define CSL_USB_TXFUNCADDR_FUNCADDR_RESETVAL (0x0000u)
N
N#define CSL_USB_TXFUNCADDR_RESETVAL (0x0000u)
N
N/* TXHUBADDR_PORT */
N
N
N
N#define CSL_USB_TXHUBADDR_PORT_MULT_TRANS_MASK (0x0080u)
N#define CSL_USB_TXHUBADDR_PORT_MULT_TRANS_SHIFT (0x0007u)
N#define CSL_USB_TXHUBADDR_PORT_MULT_TRANS_RESETVAL (0x0000u)
N
N#define CSL_USB_TXHUBADDR_PORT_HUBPORT_MASK (0x007Fu)
N#define CSL_USB_TXHUBADDR_PORT_HUBPORT_SHIFT (0x0000u)
N#define CSL_USB_TXHUBADDR_PORT_HUBPORT_RESETVAL (0x0000u)
N
N#define CSL_USB_TXHUBADDR_PORT_HUBADDR_MASK (0x007Fu)
N#define CSL_USB_TXHUBADDR_PORT_HUBADDR_SHIFT (0x0000u)
N#define CSL_USB_TXHUBADDR_PORT_HUBADDR_RESETVAL (0x0000u)
N
N#define CSL_USB_TXHUBADDR_PORT_RESETVAL (0x0000u)
N
N/* RXFUNCADDR */
N
N
N#define CSL_USB_RXFUNCADDR_FUNCADDR_MASK (0x007Fu)
N#define CSL_USB_RXFUNCADDR_FUNCADDR_SHIFT (0x0000u)
N#define CSL_USB_RXFUNCADDR_FUNCADDR_RESETVAL (0x0000u)
N
N#define CSL_USB_RXFUNCADDR_RESETVAL (0x0000u)
N
N/* RXHUBADDR_PORT */
N
N
N#define CSL_USB_RXHUBADDR_PORT_HUBPORT_MASK (0x7F00u)
N#define CSL_USB_RXHUBADDR_PORT_HUBPORT_SHIFT (0x0008u)
N#define CSL_USB_RXHUBADDR_PORT_HUBPORT_RESETVAL (0x0000u)
N
N#define CSL_USB_RXHUBADDR_PORT_MULT_TRANS_MASK (0x0080u)
N#define CSL_USB_RXHUBADDR_PORT_MULT_TRANS_SHIFT (0x0007u)
N#define CSL_USB_RXHUBADDR_PORT_MULT_TRANS_RESETVAL (0x0000u)
N
N#define CSL_USB_RXHUBADDR_PORT_HUBADDR_MASK (0x007Fu)
N#define CSL_USB_RXHUBADDR_PORT_HUBADDR_SHIFT (0x0000u)
N#define CSL_USB_RXHUBADDR_PORT_HUBADDR_RESETVAL (0x0000u)
N
N#define CSL_USB_RXHUBADDR_PORT_RESETVAL (0x0000u)
N
N/* TXMAXP */
N
N
N#define CSL_USB_TXMAXP_MAXPAYLOAD_MASK (0x07FFu)
N#define CSL_USB_TXMAXP_MAXPAYLOAD_SHIFT (0x0000u)
N#define CSL_USB_TXMAXP_MAXPAYLOAD_RESETVAL (0x0000u)
N
N#define CSL_USB_TXMAXP_RESETVAL (0x0000u)
N
N/* PERI_CSR0 */
N
N
N#define CSL_USB_PERI_CSR0_FLUSHFIFO_MASK (0x0100u)
N#define CSL_USB_PERI_CSR0_FLUSHFIFO_SHIFT (0x0008u)
N#define CSL_USB_PERI_CSR0_FLUSHFIFO_RESETVAL (0x0000u)
N
N#define CSL_USB_PERI_CSR0_SERV_SETUPEND_MASK (0x0080u)
N#define CSL_USB_PERI_CSR0_SERV_SETUPEND_SHIFT (0x0007u)
N#define CSL_USB_PERI_CSR0_SERV_SETUPEND_RESETVAL (0x0000u)
N
N#define CSL_USB_PERI_CSR0_SERV_RXPKTRDY_MASK (0x0040u)
N#define CSL_USB_PERI_CSR0_SERV_RXPKTRDY_SHIFT (0x0006u)
N#define CSL_USB_PERI_CSR0_SERV_RXPKTRDY_RESETVAL (0x0000u)
N
N#define CSL_USB_PERI_CSR0_SENDSTALL_MASK (0x0020u)
N#define CSL_USB_PERI_CSR0_SENDSTALL_SHIFT (0x0005u)
N#define CSL_USB_PERI_CSR0_SENDSTALL_RESETVAL (0x0000u)
N
N#define CSL_USB_PERI_CSR0_SETUPEND_MASK (0x0010u)
N#define CSL_USB_PERI_CSR0_SETUPEND_SHIFT (0x0004u)
N#define CSL_USB_PERI_CSR0_SETUPEND_RESETVAL (0x0000u)
N
N#define CSL_USB_PERI_CSR0_DATAEND_MASK (0x0008u)
N#define CSL_USB_PERI_CSR0_DATAEND_SHIFT (0x0003u)
N#define CSL_USB_PERI_CSR0_DATAEND_RESETVAL (0x0000u)
N
N#define CSL_USB_PERI_CSR0_SENTSTALL_MASK (0x0004u)
N#define CSL_USB_PERI_CSR0_SENTSTALL_SHIFT (0x0002u)
N#define CSL_USB_PERI_CSR0_SENTSTALL_RESETVAL (0x0000u)
N
N#define CSL_USB_PERI_CSR0_TXPKTRDY_MASK (0x0002u)
N#define CSL_USB_PERI_CSR0_TXPKTRDY_SHIFT (0x0001u)
N#define CSL_USB_PERI_CSR0_TXPKTRDY_RESETVAL (0x0000u)
N
N#define CSL_USB_PERI_CSR0_RXPKTRDY_MASK (0x0001u)
N#define CSL_USB_PERI_CSR0_RXPKTRDY_SHIFT (0x0000u)
N#define CSL_USB_PERI_CSR0_RXPKTRDY_RESETVAL (0x0000u)
N
N#define CSL_USB_PERI_CSR0_RESETVAL (0x0000u)
N
N/* RXMAXP */
N
N
N#define CSL_USB_RXMAXP_MAXPAYLOAD_MASK (0x07FFu)
N#define CSL_USB_RXMAXP_MAXPAYLOAD_SHIFT (0x0000u)
N#define CSL_USB_RXMAXP_MAXPAYLOAD_RESETVAL (0x0000u)
N
N#define CSL_USB_RXMAXP_RESETVAL (0x0000u)
N
N/* PERI_RXCSR */
N
N#define CSL_USB_PERI_RXCSR_AUTOCLEAR_MASK (0x8000u)
N#define CSL_USB_PERI_RXCSR_AUTOCLEAR_SHIFT (0x000Fu)
N#define CSL_USB_PERI_RXCSR_AUTOCLEAR_RESETVAL (0x0000u)
N
N#define CSL_USB_PERI_RXCSR_ISO_MASK (0x4000u)
N#define CSL_USB_PERI_RXCSR_ISO_SHIFT (0x000Eu)
N#define CSL_USB_PERI_RXCSR_ISO_RESETVAL (0x0000u)
N
N#define CSL_USB_PERI_RXCSR_DMAEN_MASK (0x2000u)
N#define CSL_USB_PERI_RXCSR_DMAEN_SHIFT (0x000Du)
N#define CSL_USB_PERI_RXCSR_DMAEN_RESETVAL (0x0000u)
N
N#define CSL_USB_PERI_RXCSR_DISNYET_MASK (0x1000u)
N#define CSL_USB_PERI_RXCSR_DISNYET_SHIFT (0x000Cu)
N#define CSL_USB_PERI_RXCSR_DISNYET_RESETVAL (0x0000u)
N
N#define CSL_USB_PERI_RXCSR_DMAMODE_MASK (0x0800u)
N#define CSL_USB_PERI_RXCSR_DMAMODE_SHIFT (0x000Bu)
N#define CSL_USB_PERI_RXCSR_DMAMODE_RESETVAL (0x0000u)
N
N
N#define CSL_USB_PERI_RXCSR_CLRDATATOG_MASK (0x0080u)
N#define CSL_USB_PERI_RXCSR_CLRDATATOG_SHIFT (0x0007u)
N#define CSL_USB_PERI_RXCSR_CLRDATATOG_RESETVAL (0x0000u)
N
N#define CSL_USB_PERI_RXCSR_SENTSTALL_MASK (0x0040u)
N#define CSL_USB_PERI_RXCSR_SENTSTALL_SHIFT (0x0006u)
N#define CSL_USB_PERI_RXCSR_SENTSTALL_RESETVAL (0x0000u)
N
N#define CSL_USB_PERI_RXCSR_SENDSTALL_MASK (0x0020u)
N#define CSL_USB_PERI_RXCSR_SENDSTALL_SHIFT (0x0005u)
N#define CSL_USB_PERI_RXCSR_SENDSTALL_RESETVAL (0x0000u)
N
N#define CSL_USB_PERI_RXCSR_FLUSHFIFO_MASK (0x0010u)
N#define CSL_USB_PERI_RXCSR_FLUSHFIFO_SHIFT (0x0004u)
N#define CSL_USB_PERI_RXCSR_FLUSHFIFO_RESETVAL (0x0000u)
N
N#define CSL_USB_PERI_RXCSR_DATAERROR_MASK (0x0008u)
N#define CSL_USB_PERI_RXCSR_DATAERROR_SHIFT (0x0003u)
N#define CSL_USB_PERI_RXCSR_DATAERROR_RESETVAL (0x0000u)
N
N#define CSL_USB_PERI_RXCSR_OVERRUN_MASK (0x0004u)
N#define CSL_USB_PERI_RXCSR_OVERRUN_SHIFT (0x0002u)
N#define CSL_USB_PERI_RXCSR_OVERRUN_RESETVAL (0x0000u)
N
N#define CSL_USB_PERI_RXCSR_FIFOFULL_MASK (0x0002u)
N#define CSL_USB_PERI_RXCSR_FIFOFULL_SHIFT (0x0001u)
N#define CSL_USB_PERI_RXCSR_FIFOFULL_RESETVAL (0x0000u)
N
N#define CSL_USB_PERI_RXCSR_RXPKTRDY_MASK (0x0001u)
N#define CSL_USB_PERI_RXCSR_RXPKTRDY_SHIFT (0x0000u)
N#define CSL_USB_PERI_RXCSR_RXPKTRDY_RESETVAL (0x0000u)
N
N#define CSL_USB_PERI_RXCSR_RESETVAL (0x0000u)
N
N/* COUNT0 */
N
N
N#define CSL_USB_COUNT0_EP0RXCOUNT_MASK (0x007Fu)
N#define CSL_USB_COUNT0_EP0RXCOUNT_SHIFT (0x0000u)
N#define CSL_USB_COUNT0_EP0RXCOUNT_RESETVAL (0x0000u)
N
N#define CSL_USB_COUNT0_RESETVAL (0x0000u)
N
N/* CONFIGDATA_FIFOSIZE */
N
N#define CSL_USB_CONFIGDATA_FIFOSIZE_RXFIFOSZ_MASK (0xF000u)
N#define CSL_USB_CONFIGDATA_FIFOSIZE_RXFIFOSZ_SHIFT (0x000Cu)
N#define CSL_USB_CONFIGDATA_FIFOSIZE_RXFIFOSZ_RESETVAL (0x0000u)
N
N#define CSL_USB_CONFIGDATA_FIFOSIZE_TXFIFOSZ_MASK (0x0F00u)
N#define CSL_USB_CONFIGDATA_FIFOSIZE_TXFIFOSZ_SHIFT (0x0008u)
N#define CSL_USB_CONFIGDATA_FIFOSIZE_TXFIFOSZ_RESETVAL (0x0000u)
N
N
N#define CSL_USB_CONFIGDATA_FIFOSIZE_MPTXE_MASK (0x0040u)
N#define CSL_USB_CONFIGDATA_FIFOSIZE_MPTXE_SHIFT (0x0006u)
N#define CSL_USB_CONFIGDATA_FIFOSIZE_MPTXE_RESETVAL (0x0000u)
N
N#define CSL_USB_CONFIGDATA_FIFOSIZE_BIGENDIAN_MASK (0x0020u)
N#define CSL_USB_CONFIGDATA_FIFOSIZE_BIGENDIAN_SHIFT (0x0005u)
N#define CSL_USB_CONFIGDATA_FIFOSIZE_BIGENDIAN_RESETVAL (0x0000u)
N
N#define CSL_USB_CONFIGDATA_FIFOSIZE_HBRXE_MASK (0x0010u)
N#define CSL_USB_CONFIGDATA_FIFOSIZE_HBRXE_SHIFT (0x0004u)
N#define CSL_USB_CONFIGDATA_FIFOSIZE_HBRXE_RESETVAL (0x0000u)
N
N#define CSL_USB_CONFIGDATA_FIFOSIZE_HBTXE_MASK (0x0008u)
N#define CSL_USB_CONFIGDATA_FIFOSIZE_HBTXE_SHIFT (0x0003u)
N#define CSL_USB_CONFIGDATA_FIFOSIZE_HBTXE_RESETVAL (0x0000u)
N
N#define CSL_USB_CONFIGDATA_FIFOSIZE_DYNFIFO_MASK (0x0004u)
N#define CSL_USB_CONFIGDATA_FIFOSIZE_DYNFIFO_SHIFT (0x0002u)
N#define CSL_USB_CONFIGDATA_FIFOSIZE_DYNFIFO_RESETVAL (0x0000u)
N
N#define CSL_USB_CONFIGDATA_FIFOSIZE_SOFTCONE_MASK (0x0002u)
N#define CSL_USB_CONFIGDATA_FIFOSIZE_SOFTCONE_SHIFT (0x0001u)
N#define CSL_USB_CONFIGDATA_FIFOSIZE_SOFTCONE_RESETVAL (0x0000u)
N
N#define CSL_USB_CONFIGDATA_FIFOSIZE_UTMIDATAWIDTH_MASK (0x0001u)
N#define CSL_USB_CONFIGDATA_FIFOSIZE_UTMIDATAWIDTH_SHIFT (0x0000u)
N#define CSL_USB_CONFIGDATA_FIFOSIZE_UTMIDATAWIDTH_RESETVAL (0x0000u)
N
N#define CSL_USB_CONFIGDATA_FIFOSIZE_RESETVAL (0x0000u)
N
N/* TXCHGCFGRL */
N
N
N#define CSL_USB_TXCHGCFGRL_TXDEFTQMGR_MASK (0x3000u)
N#define CSL_USB_TXCHGCFGRL_TXDEFTQMGR_SHIFT (0x000Cu)
N#define CSL_USB_TXCHGCFGRL_TXDEFTQMGR_RESETVAL (0x0000u)
N
N#define CSL_USB_TXCHGCFGRL_TXDEFTQNUM_MASK (0x0FFFu)
N#define CSL_USB_TXCHGCFGRL_TXDEFTQNUM_SHIFT (0x0000u)
N#define CSL_USB_TXCHGCFGRL_TXDEFTQNUM_RESETVAL (0x0000u)
N
N#define CSL_USB_TXCHGCFGRL_RESETVAL (0x0000u)
N
N/* TXCHGCFGRH */
N
N#define CSL_USB_TXCHGCFGRH_TXENABLE_MASK (0x8000u)
N#define CSL_USB_TXCHGCFGRH_TXENABLE_SHIFT (0x000Fu)
N#define CSL_USB_TXCHGCFGRH_TXENABLE_RESETVAL (0x0000u)
N
N#define CSL_USB_TXCHGCFGRH_TXTEARDOWN_MASK (0x4000u)
N#define CSL_USB_TXCHGCFGRH_TXTEARDOWN_SHIFT (0x000Eu)
N#define CSL_USB_TXCHGCFGRH_TXTEARDOWN_RESETVAL (0x0000u)
N
N
N#define CSL_USB_TXCHGCFGRH_RESETVAL (0x0000u)
N
N/* RXCHGCRL */
N
N
N
N#define CSL_USB_RXCHGCRL_RXDEFTRQQNUM_MASK (0x0FFFu)
N#define CSL_USB_RXCHGCRL_RXDEFTRQQNUM_SHIFT (0x0000u)
N#define CSL_USB_RXCHGCRL_RXDEFTRQQNUM_RESETVAL (0x0000u)
N
N#define CSL_USB_RXCHGCRL_RESETVAL (0x0000u)
N
N/* RXCHGCRH */
N
N#define CSL_USB_RXCHGCRH_RXENABLE_MASK (0x8000u)
N#define CSL_USB_RXCHGCRH_RXENABLE_SHIFT (0x000Fu)
N#define CSL_USB_RXCHGCRH_RXENABLE_RESETVAL (0x0000u)
N
N#define CSL_USB_RXCHGCRH_RXTEARDOWN_MASK (0x4000u)
N#define CSL_USB_RXCHGCRH_RXTEARDOWN_SHIFT (0x000Eu)
N#define CSL_USB_RXCHGCRH_RXTEARDOWN_RESETVAL (0x0000u)
N
N
N#define CSL_USB_RXCHGCRH_RXERRHANDLING_MASK (0x0100u)
N#define CSL_USB_RXCHGCRH_RXERRHANDLING_SHIFT (0x0008u)
N#define CSL_USB_RXCHGCRH_RXERRHANDLING_RESETVAL (0x0000u)
N
N#define CSL_USB_RXCHGCRH_RXSOPOFFSET_MASK (0x00FFu)
N#define CSL_USB_RXCHGCRH_RXSOPOFFSET_SHIFT (0x0000u)
N#define CSL_USB_RXCHGCRH_RXSOPOFFSET_RESETVAL (0x0000u)
N
N#define CSL_USB_RXCHGCRH_RESETVAL (0x0000u)
N
N/* RXCHHPCRAL */
N
N
N
N#define CSL_USB_RXCHHPCRAL_RXHOSTFDQ0QNUM_MASK (0x0FFFu)
N#define CSL_USB_RXCHHPCRAL_RXHOSTFDQ0QNUM_SHIFT (0x0000u)
N#define CSL_USB_RXCHHPCRAL_RXHOSTFDQ0QNUM_RESETVAL (0x0000u)
N
N#define CSL_USB_RXCHHPCRAL_RESETVAL (0x0000u)
N
N/* RXCHHPCRAH */
N
N
N
N#define CSL_USB_RXCHHPCRAH_RXHOSTFDQ1QNUM_MASK (0x0FFFu)
N#define CSL_USB_RXCHHPCRAH_RXHOSTFDQ1QNUM_SHIFT (0x0000u)
N#define CSL_USB_RXCHHPCRAH_RXHOSTFDQ1QNUM_RESETVAL (0x0000u)
N
N#define CSL_USB_RXCHHPCRAH_RESETVAL (0x0000u)
N
N/* RXCHHPCRBL */
N
N
N
N#define CSL_USB_RXCHHPCRBL_RXHOSTFDQ2QNUM_MASK (0x0FFFu)
N#define CSL_USB_RXCHHPCRBL_RXHOSTFDQ2QNUM_SHIFT (0x0000u)
N#define CSL_USB_RXCHHPCRBL_RXHOSTFDQ2QNUM_RESETVAL (0x0000u)
N
N#define CSL_USB_RXCHHPCRBL_RESETVAL (0x0000u)
N
N/* RXCHHPCRBH */
N
N
N
N#define CSL_USB_RXCHHPCRBH_RXHOSTFDQ3QNUM_MASK (0x0FFFu)
N#define CSL_USB_RXCHHPCRBH_RXHOSTFDQ3QNUM_SHIFT (0x0000u)
N#define CSL_USB_RXCHHPCRBH_RXHOSTFDQ3QNUM_RESETVAL (0x0000u)
N
N#define CSL_USB_RXCHHPCRBH_RESETVAL (0x0000u)
N
N/* SCHETBLWNL */
N
N#define CSL_USB_SCHETBLWNL_ENTRY1_RXTX_MASK (0xFFE0u)
N#define CSL_USB_SCHETBLWNL_ENTRY1_RXTX_SHIFT (0x0005u)
N#define CSL_USB_SCHETBLWNL_ENTRY1_RXTX_RESETVAL (0x0000u)
N
N
N#define CSL_USB_SCHETBLWNL_ENTRY1_CHAN_MASK (0x1F00u)
N#define CSL_USB_SCHETBLWNL_ENTRY1_CHAN_SHIFT (0x0008u)
N#define CSL_USB_SCHETBLWNL_ENTRY1_CHAN_RESETVAL (0x0000u)
N
N#define CSL_USB_SCHETBLWNL_ENTRY0_RXTX_MASK (0x0080u)
N#define CSL_USB_SCHETBLWNL_ENTRY0_RXTX_SHIFT (0x0007u)
N#define CSL_USB_SCHETBLWNL_ENTRY0_RXTX_RESETVAL (0x0000u)
N
N
N#define CSL_USB_SCHETBLWNL_ENTRY0_CHAN_MASK (0x001Fu)
N#define CSL_USB_SCHETBLWNL_ENTRY0_CHAN_SHIFT (0x0000u)
N#define CSL_USB_SCHETBLWNL_ENTRY0_CHAN_RESETVAL (0x0000u)
N
N#define CSL_USB_SCHETBLWNL_RESETVAL (0x0000u)
N
N/* SCHETBLWNH */
N
N#define CSL_USB_SCHETBLWNH_ENTRY3_RXTX_MASK (0xFFE0u)
N#define CSL_USB_SCHETBLWNH_ENTRY3_RXTX_SHIFT (0x0005u)
N#define CSL_USB_SCHETBLWNH_ENTRY3_RXTX_RESETVAL (0x0000u)
N
N
N#define CSL_USB_SCHETBLWNH_ENTRY3_CHAN_MASK (0x1F00u)
N#define CSL_USB_SCHETBLWNH_ENTRY3_CHAN_SHIFT (0x0008u)
N#define CSL_USB_SCHETBLWNH_ENTRY3_CHAN_RESETVAL (0x0000u)
N
N#define CSL_USB_SCHETBLWNH_ENTRY2_RXTX_MASK (0x0080u)
N#define CSL_USB_SCHETBLWNH_ENTRY2_RXTX_SHIFT (0x0007u)
N#define CSL_USB_SCHETBLWNH_ENTRY2_RXTX_RESETVAL (0x0000u)
N
N
N#define CSL_USB_SCHETBLWNH_ENTRY2_CHAN_MASK (0x001Fu)
N#define CSL_USB_SCHETBLWNH_ENTRY2_CHAN_SHIFT (0x0000u)
N#define CSL_USB_SCHETBLWNH_ENTRY2_CHAN_RESETVAL (0x0000u)
N
N#define CSL_USB_SCHETBLWNH_RESETVAL (0x0000u)
N
N/* QMMRRBARL */
N
N#define CSL_USB_QMMRRBARL_BASE_MASK (0xFFFFu)
N#define CSL_USB_QMMRRBARL_BASE_SHIFT (0x0000u)
N#define CSL_USB_QMMRRBARL_BASE_RESETVAL (0x0000u)
N
N#define CSL_USB_QMMRRBARL_RESETVAL (0x0000u)
N
N/* QMMRRBARH */
N
N#define CSL_USB_QMMRRBARH_BASE_MASK (0xFFFFu)
N#define CSL_USB_QMMRRBARH_BASE_SHIFT (0x0000u)
N#define CSL_USB_QMMRRBARH_BASE_RESETVAL (0x0000u)
N
N#define CSL_USB_QMMRRBARH_RESETVAL (0x0000u)
N
N/* QMMRRCRL */
N
N
N#define CSL_USB_QMMRRCRL_DESC_SIZE_MASK (0x0F00u)
N#define CSL_USB_QMMRRCRL_DESC_SIZE_SHIFT (0x0008u)
N#define CSL_USB_QMMRRCRL_DESC_SIZE_RESETVAL (0x0000u)
N
N
N#define CSL_USB_QMMRRCRL_REG_SIZE_MASK (0x0007u)
N#define CSL_USB_QMMRRCRL_REG_SIZE_SHIFT (0x0000u)
N#define CSL_USB_QMMRRCRL_REG_SIZE_RESETVAL (0x0000u)
N
N#define CSL_USB_QMMRRCRL_RESETVAL (0x0000u)
N
N/* QMMRRCRH */
N
N
N#define CSL_USB_QMMRRCRH_START_INDEX_MASK (0x3FFFu)
N#define CSL_USB_QMMRRCRH_START_INDEX_SHIFT (0x0000u)
N#define CSL_USB_QMMRRCRH_START_INDEX_RESETVAL (0x0000u)
N
N#define CSL_USB_QMMRRCRH_RESETVAL (0x0000u)
N
N/* QMMQNRAL */
N
N
N#define CSL_USB_QMMQNRAL_Q_ENTRY_COUNT_MASK (0x3FFFu)
N#define CSL_USB_QMMQNRAL_Q_ENTRY_COUNT_SHIFT (0x0000u)
N#define CSL_USB_QMMQNRAL_Q_ENTRY_COUNT_RESETVAL (0x0000u)
N
N#define CSL_USB_QMMQNRAL_RESETVAL (0x0000u)
N
N/* QMMQNRAH */
N
N
N#define CSL_USB_QMMQNRAH_RESETVAL (0x0000u)
N
N/* QMMQNRBL */
N
N#define CSL_USB_QMMQNRBL_Q_BYTE_COUNT_MASK (0xFFFFu)
N#define CSL_USB_QMMQNRBL_Q_BYTE_COUNT_SHIFT (0x0000u)
N#define CSL_USB_QMMQNRBL_Q_BYTE_COUNT_RESETVAL (0x0000u)
N
N#define CSL_USB_QMMQNRBL_RESETVAL (0x0000u)
N
N/* QMMQNRBH */
N
N
N#define CSL_USB_QMMQNRBH_Q_BYTE_COUNT_MASK (0x0FFFu)
N#define CSL_USB_QMMQNRBH_Q_BYTE_COUNT_SHIFT (0x0000u)
N#define CSL_USB_QMMQNRBH_Q_BYTE_COUNT_RESETVAL (0x0000u)
N
N#define CSL_USB_QMMQNRBH_RESETVAL (0x0000u)
N
N/* QMMQNRCL */
N
N
N#define CSL_USB_QMMQNRCL_PACKET_SIZE_MASK (0x3FFFu)
N#define CSL_USB_QMMQNRCL_PACKET_SIZE_SHIFT (0x0000u)
N#define CSL_USB_QMMQNRCL_PACKET_SIZE_RESETVAL (0x0000u)
N
N#define CSL_USB_QMMQNRCL_RESETVAL (0x0000u)
N
N/* QMMQNRCH */
N
N#define CSL_USB_QMMQNRCH_HEAD_TAIL_MASK (0x8000u)
N#define CSL_USB_QMMQNRCH_HEAD_TAIL_SHIFT (0x000Fu)
N#define CSL_USB_QMMQNRCH_HEAD_TAIL_RESETVAL (0x0000u)
N
N
N#define CSL_USB_QMMQNRCH_RESETVAL (0x0000u)
N
N/* QMMQNRDL */
N
N#define CSL_USB_QMMQNRDL_DESC_PTR_MASK (0xFFE0u)
N#define CSL_USB_QMMQNRDL_DESC_PTR_SHIFT (0x0005u)
N#define CSL_USB_QMMQNRDL_DESC_PTR_RESETVAL (0x0000u)
N
N#define CSL_USB_QMMQNRDL_DESC_SIZE_MASK (0x001Fu)
N#define CSL_USB_QMMQNRDL_DESC_SIZE_SHIFT (0x0000u)
N#define CSL_USB_QMMQNRDL_DESC_SIZE_RESETVAL (0x0000u)
N
N#define CSL_USB_QMMQNRDL_RESETVAL (0x0000u)
N
N/* QMMQNRDH */
N
N#define CSL_USB_QMMQNRDH_DESC_PTR_MASK (0xFFFFu)
N#define CSL_USB_QMMQNRDH_DESC_PTR_SHIFT (0x0000u)
N#define CSL_USB_QMMQNRDH_DESC_PTR_RESETVAL (0x0000u)
N
N#define CSL_USB_QMMQNRDH_RESETVAL (0x0000u)
N
N/* QMMQNSRAL */
N
N
N#define CSL_USB_QMMQNSRAL_Q_ENTRY_COUNT_MASK (0x3FFFu)
N#define CSL_USB_QMMQNSRAL_Q_ENTRY_COUNT_SHIFT (0x0000u)
N#define CSL_USB_QMMQNSRAL_Q_ENTRY_COUNT_RESETVAL (0x0000u)
N
N#define CSL_USB_QMMQNSRAL_RESETVAL (0x0000u)
N
N/* QMMQNSRAH */
N
N
N#define CSL_USB_QMMQNSRAH_RESETVAL (0x0000u)
N
N/* QMMQNSRBL */
N
N#define CSL_USB_QMMQNSRBL_Q_BYTE_COUNT_MASK (0xFFFFu)
N#define CSL_USB_QMMQNSRBL_Q_BYTE_COUNT_SHIFT (0x0000u)
N#define CSL_USB_QMMQNSRBL_Q_BYTE_COUNT_RESETVAL (0x0000u)
N
N#define CSL_USB_QMMQNSRBL_RESETVAL (0x0000u)
N
N/* QMMQNSRBH */
N
N
N#define CSL_USB_QMMQNSRBH_Q_BYTE_COUNT_MASK (0x0FFFu)
N#define CSL_USB_QMMQNSRBH_Q_BYTE_COUNT_SHIFT (0x0000u)
N#define CSL_USB_QMMQNSRBH_Q_BYTE_COUNT_RESETVAL (0x0000u)
N
N#define CSL_USB_QMMQNSRBH_RESETVAL (0x0000u)
N
N/* QMMQNSRC */
N
N
N#define CSL_USB_QMMQNSRC_PACKET_SIZE_MASK (0x3FFFu)
N#define CSL_USB_QMMQNSRC_PACKET_SIZE_SHIFT (0x0000u)
N#define CSL_USB_QMMQNSRC_PACKET_SIZE_RESETVAL (0x0000u)
N
N#define CSL_USB_QMMQNSRC_RESETVAL (0x0000u)
N
N/* REVRL */
N
N#define CSL_USB_REVRL_REVRTL_MASK (0xF800u)
N#define CSL_USB_REVRL_REVRTL_SHIFT (0x000Bu)
N#define CSL_USB_REVRL_REVRTL_RESETVAL (0x0001u)
N
N#define CSL_USB_REVRL_REVMAJ_MASK (0x0700u)
N#define CSL_USB_REVRL_REVMAJ_SHIFT (0x0008u)
N#define CSL_USB_REVRL_REVMAJ_RESETVAL (0x0000u)
N
N#define CSL_USB_REVRL_REVMIN_MASK (0x00FFu)
N#define CSL_USB_REVRL_REVMIN_SHIFT (0x0000u)
N#define CSL_USB_REVRL_REVMIN_RESETVAL (0x0000u)
N
N#define CSL_USB_REVRL_RESETVAL (0x0800u)
N
N/* REVRH */
N
N
N#define CSL_USB_REVRH_MODID_MASK (0x0FFFu)
N#define CSL_USB_REVRH_MODID_SHIFT (0x0000u)
N#define CSL_USB_REVRH_MODID_RESETVAL (0x0EA0u)
N
N#define CSL_USB_REVRH_RESETVAL (0x4EA0u)
N
N/* CTRLRL */
N
N
N#define CSL_USB_CTRLRL_RNDIS_MASK (0x0010u)
N#define CSL_USB_CTRLRL_RNDIS_SHIFT (0x0004u)
N#define CSL_USB_CTRLRL_RNDIS_RESETVAL (0x0000u)
N
N#define CSL_USB_CTRLRL_UINT_MASK (0x0008u)
N#define CSL_USB_CTRLRL_UINT_SHIFT (0x0003u)
N#define CSL_USB_CTRLRL_UINT_RESETVAL (0x0000u)
N
N
N#define CSL_USB_CTRLRL_CLKFACK_MASK (0x0002u)
N#define CSL_USB_CTRLRL_CLKFACK_SHIFT (0x0001u)
N#define CSL_USB_CTRLRL_CLKFACK_RESETVAL (0x0000u)
N
N#define CSL_USB_CTRLRL_RESET_MASK (0x0001u)
N#define CSL_USB_CTRLRL_RESET_SHIFT (0x0000u)
N#define CSL_USB_CTRLRL_RESET_RESETVAL (0x0000u)
N
N#define CSL_USB_CTRLRL_RESETVAL (0x0000u)
N
N/* CTRLRH */
N
N#define CSL_USB_CTRLRH_DISDEB_MASK (0x8000u)
N#define CSL_USB_CTRLRH_DISDEB_SHIFT (0x000Fu)
N#define CSL_USB_CTRLRH_DISDEB_RESETVAL (0x0000u)
N
N#define CSL_USB_CTRLRH_DISSRP_MASK (0x4000u)
N#define CSL_USB_CTRLRH_DISSRP_SHIFT (0x000Eu)
N#define CSL_USB_CTRLRH_DISSRP_RESETVAL (0x0000u)
N
N
N#define CSL_USB_CTRLRH_RESETVAL (0x0000u)
N
N/* STATR */
N
N
N#define CSL_USB_STATR_DRVVBUS_MASK (0x0001u)
N#define CSL_USB_STATR_DRVVBUS_SHIFT (0x0000u)
N#define CSL_USB_STATR_DRVVBUS_RESETVAL (0x0000u)
N
N#define CSL_USB_STATR_RESETVAL (0x0000u)
N
N/* EMUR */
N
N
N#define CSL_USB_EMUR_RTSEL_MASK (0x0004u)
N#define CSL_USB_EMUR_RTSEL_SHIFT (0x0002u)
N#define CSL_USB_EMUR_RTSEL_RESETVAL (0x0000u)
N
N#define CSL_USB_EMUR_SOFT_MASK (0x0002u)
N#define CSL_USB_EMUR_SOFT_SHIFT (0x0001u)
N#define CSL_USB_EMUR_SOFT_RESETVAL (0x0001u)
N
N#define CSL_USB_EMUR_FREERUN_MASK (0x0001u)
N#define CSL_USB_EMUR_FREERUN_SHIFT (0x0000u)
N#define CSL_USB_EMUR_FREERUN_RESETVAL (0x0001u)
N
N#define CSL_USB_EMUR_RESETVAL (0x0003u)
N
N/* MODERL */
N
N
N#define CSL_USB_MODERL_TX4MODE_MASK (0x3000u)
N#define CSL_USB_MODERL_TX4MODE_SHIFT (0x000Cu)
N#define CSL_USB_MODERL_TX4MODE_RESETVAL (0x0000u)
N
N
N#define CSL_USB_MODERL_TX3MODE_MASK (0x0300u)
N#define CSL_USB_MODERL_TX3MODE_SHIFT (0x0008u)
N#define CSL_USB_MODERL_TX3MODE_RESETVAL (0x0000u)
N
N
N#define CSL_USB_MODERL_TX2MODE_MASK (0x0030u)
N#define CSL_USB_MODERL_TX2MODE_SHIFT (0x0004u)
N#define CSL_USB_MODERL_TX2MODE_RESETVAL (0x0000u)
N
N
N#define CSL_USB_MODERL_TX1MODE_MASK (0x0003u)
N#define CSL_USB_MODERL_TX1MODE_SHIFT (0x0000u)
N#define CSL_USB_MODERL_TX1MODE_RESETVAL (0x0000u)
N
N#define CSL_USB_MODERL_RESETVAL (0x0000u)
N
N/* MODERH */
N
N
N#define CSL_USB_MODERH_RX4MODE_MASK (0x3000u)
N#define CSL_USB_MODERH_RX4MODE_SHIFT (0x000Cu)
N#define CSL_USB_MODERH_RX4MODE_RESETVAL (0x0000u)
N
N
N#define CSL_USB_MODERH_RX3MODE_MASK (0x0300u)
N#define CSL_USB_MODERH_RX3MODE_SHIFT (0x0008u)
N#define CSL_USB_MODERH_RX3MODE_RESETVAL (0x0000u)
N
N
N#define CSL_USB_MODERH_RX2MODE_MASK (0x0030u)
N#define CSL_USB_MODERH_RX2MODE_SHIFT (0x0004u)
N#define CSL_USB_MODERH_RX2MODE_RESETVAL (0x0000u)
N
N
N#define CSL_USB_MODERH_RX1MODE_MASK (0x0003u)
N#define CSL_USB_MODERH_RX1MODE_SHIFT (0x0000u)
N#define CSL_USB_MODERH_RX1MODE_RESETVAL (0x0000u)
N
N#define CSL_USB_MODERH_RESETVAL (0x0000u)
N
N/* AUTOREQ */
N
N
N#define CSL_USB_AUTOREQ_RX4_AUTOREQ_MASK (0x00C0u)
N#define CSL_USB_AUTOREQ_RX4_AUTOREQ_SHIFT (0x0006u)
N#define CSL_USB_AUTOREQ_RX4_AUTOREQ_RESETVAL (0x0000u)
N
N#define CSL_USB_AUTOREQ_RX3_AUTOREQ_MASK (0x0030u)
N#define CSL_USB_AUTOREQ_RX3_AUTOREQ_SHIFT (0x0004u)
N#define CSL_USB_AUTOREQ_RX3_AUTOREQ_RESETVAL (0x0000u)
N
N#define CSL_USB_AUTOREQ_RX2_AUTOREQ_MASK (0x000Cu)
N#define CSL_USB_AUTOREQ_RX2_AUTOREQ_SHIFT (0x0002u)
N#define CSL_USB_AUTOREQ_RX2_AUTOREQ_RESETVAL (0x0000u)
N
N#define CSL_USB_AUTOREQ_RX1_AUTOREQ_MASK (0x0003u)
N#define CSL_USB_AUTOREQ_RX1_AUTOREQ_SHIFT (0x0000u)
N#define CSL_USB_AUTOREQ_RX1_AUTOREQ_RESETVAL (0x0000u)
N
N#define CSL_USB_AUTOREQ_RESETVAL (0x0000u)
N
N/* SRPFIXL */
N
N#define CSL_USB_SRPFIXL_SRPFIXTIME_MASK (0xFFFFu)
N#define CSL_USB_SRPFIXL_SRPFIXTIME_SHIFT (0x0000u)
N#define CSL_USB_SRPFIXL_SRPFIXTIME_RESETVAL (0xDE80u)
N
N#define CSL_USB_SRPFIXL_RESETVAL (0xDE80u)
N
N/* SRPFIXH */
N
N#define CSL_USB_SRPFIXH_SRPFIXTIME_MASK (0xFFFFu)
N#define CSL_USB_SRPFIXH_SRPFIXTIME_SHIFT (0x0000u)
N#define CSL_USB_SRPFIXH_SRPFIXTIME_RESETVAL (0x0280u)
N
N#define CSL_USB_SRPFIXH_RESETVAL (0x0280u)
N
N/* RXTDOWNL */
N
N
N#define CSL_USB_RXTDOWNL_RX_TDOWN_MASK (0x001Eu)
N#define CSL_USB_RXTDOWNL_RX_TDOWN_SHIFT (0x0001u)
N#define CSL_USB_RXTDOWNL_RX_TDOWN_RESETVAL (0x0000u)
N
N
N#define CSL_USB_RXTDOWNL_RESETVAL (0x0000u)
N
N/* RXTDOWNH */
N
N
N#define CSL_USB_RXTDOWNH_TX_TDOWN_MASK (0x001Eu)
N#define CSL_USB_RXTDOWNH_TX_TDOWN_SHIFT (0x0001u)
N#define CSL_USB_RXTDOWNH_TX_TDOWN_RESETVAL (0x0000u)
N
N
N#define CSL_USB_RXTDOWNH_RESETVAL (0x0000u)
N
N/* INTSRCRL */
N
N
N#define CSL_USB_INTSRCRL_RX_MASK (0x1E00u)
N#define CSL_USB_INTSRCRL_RX_SHIFT (0x0009u)
N#define CSL_USB_INTSRCRL_RX_RESETVAL (0x0000u)
N
N
N#define CSL_USB_INTSRCRL_TX_MASK (0x001Fu)
N#define CSL_USB_INTSRCRL_TX_SHIFT (0x0000u)
N#define CSL_USB_INTSRCRL_TX_RESETVAL (0x0000u)
N
N#define CSL_USB_INTSRCRL_RESETVAL (0x0000u)
N
N/* INTSRCRH */
N
N
N#define CSL_USB_INTSRCRH_USB_MASK (0x01FFu)
N#define CSL_USB_INTSRCRH_USB_SHIFT (0x0000u)
N#define CSL_USB_INTSRCRH_USB_RESETVAL (0x0000u)
N
N#define CSL_USB_INTSRCRH_RESETVAL (0x0000u)
N
N/* INTSETRL */
N
N
N#define CSL_USB_INTSETRL_RX_MASK (0x1F00u)
N#define CSL_USB_INTSETRL_RX_SHIFT (0x0008u)
N#define CSL_USB_INTSETRL_RX_RESETVAL (0x0000u)
N
N
N#define CSL_USB_INTSETRL_TX_MASK (0x001Fu)
N#define CSL_USB_INTSETRL_TX_SHIFT (0x0000u)
N#define CSL_USB_INTSETRL_TX_RESETVAL (0x0000u)
N
N#define CSL_USB_INTSETRL_RESETVAL (0x0000u)
N
N/* INTSETRH */
N
N
N#define CSL_USB_INTSETRH_USB_MASK (0x01FFu)
N#define CSL_USB_INTSETRH_USB_SHIFT (0x0000u)
N#define CSL_USB_INTSETRH_USB_RESETVAL (0x0000u)
N
N#define CSL_USB_INTSETRH_RESETVAL (0x0000u)
N
N/* INTCLRRL */
N
N
N#define CSL_USB_INTCLRRL_RX_MASK (0x1F00u)
N#define CSL_USB_INTCLRRL_RX_SHIFT (0x0008u)
N#define CSL_USB_INTCLRRL_RX_RESETVAL (0x0000u)
N
N
N#define CSL_USB_INTCLRRL_TX_MASK (0x001Fu)
N#define CSL_USB_INTCLRRL_TX_SHIFT (0x0000u)
N#define CSL_USB_INTCLRRL_TX_RESETVAL (0x0000u)
N
N#define CSL_USB_INTCLRRL_RESETVAL (0x0000u)
N
N/* INTCLRRH */
N
N
N#define CSL_USB_INTCLRRH_USB_MASK (0x01FFu)
N#define CSL_USB_INTCLRRH_USB_SHIFT (0x0000u)
N#define CSL_USB_INTCLRRH_USB_RESETVAL (0x0000u)
N
N#define CSL_USB_INTCLRRH_RESETVAL (0x0000u)
N
N/* INTMSKRL */
N
N
N#define CSL_USB_INTMSKRL_RX_MASK (0x1F00u)
N#define CSL_USB_INTMSKRL_RX_SHIFT (0x0008u)
N#define CSL_USB_INTMSKRL_RX_RESETVAL (0x0000u)
N
N
N#define CSL_USB_INTMSKRL_TX_MASK (0x001Fu)
N#define CSL_USB_INTMSKRL_TX_SHIFT (0x0000u)
N#define CSL_USB_INTMSKRL_TX_RESETVAL (0x0000u)
N
N#define CSL_USB_INTMSKRL_RESETVAL (0x0000u)
N
N/* INTMSKRH */
N
N
N#define CSL_USB_INTMSKRH_USB_MASK (0x01FFu)
N#define CSL_USB_INTMSKRH_USB_SHIFT (0x0000u)
N#define CSL_USB_INTMSKRH_USB_RESETVAL (0x0000u)
N
N#define CSL_USB_INTMSKRH_RESETVAL (0x0000u)
N
N/* INTMSKSETRL */
N
N
N#define CSL_USB_INTMSKSETRL_RX_MASK (0x1F00u)
N#define CSL_USB_INTMSKSETRL_RX_SHIFT (0x0008u)
N#define CSL_USB_INTMSKSETRL_RX_RESETVAL (0x0000u)
N
N
N#define CSL_USB_INTMSKSETRL_TX_MASK (0x001Fu)
N#define CSL_USB_INTMSKSETRL_TX_SHIFT (0x0000u)
N#define CSL_USB_INTMSKSETRL_TX_RESETVAL (0x0000u)
N
N#define CSL_USB_INTMSKSETRL_RESETVAL (0x0000u)
N
N/* INTMSKSETRH */
N
N
N#define CSL_USB_INTMSKSETRH_USB_MASK (0x01FFu)
N#define CSL_USB_INTMSKSETRH_USB_SHIFT (0x0000u)
N#define CSL_USB_INTMSKSETRH_USB_RESETVAL (0x0000u)
N
N#define CSL_USB_INTMSKSETRH_RESETVAL (0x0000u)
N
N/* INTMSKCLRRL */
N
N
N#define CSL_USB_INTMSKCLRRL_RX_MASK (0x1F00u)
N#define CSL_USB_INTMSKCLRRL_RX_SHIFT (0x0008u)
N#define CSL_USB_INTMSKCLRRL_RX_RESETVAL (0x0000u)
N
N
N#define CSL_USB_INTMSKCLRRL_TX_MASK (0x001Fu)
N#define CSL_USB_INTMSKCLRRL_TX_SHIFT (0x0000u)
N#define CSL_USB_INTMSKCLRRL_TX_RESETVAL (0x0000u)
N
N#define CSL_USB_INTMSKCLRRL_RESETVAL (0x0000u)
N
N/* INTMSKCLRRH */
N
N
N#define CSL_USB_INTMSKCLRRH_USB_MASK (0x01FFu)
N#define CSL_USB_INTMSKCLRRH_USB_SHIFT (0x0000u)
N#define CSL_USB_INTMSKCLRRH_USB_RESETVAL (0x0000u)
N
N#define CSL_USB_INTMSKCLRRH_RESETVAL (0x0000u)
N
N/* INTMASKEDRL */
N
N
N#define CSL_USB_INTMASKEDRL_RX_MASK (0x1F00u)
N#define CSL_USB_INTMASKEDRL_RX_SHIFT (0x0008u)
N#define CSL_USB_INTMASKEDRL_RX_RESETVAL (0x0000u)
N
N
N#define CSL_USB_INTMASKEDRL_TX_MASK (0x001Fu)
N#define CSL_USB_INTMASKEDRL_TX_SHIFT (0x0000u)
N#define CSL_USB_INTMASKEDRL_TX_RESETVAL (0x0000u)
N
N#define CSL_USB_INTMASKEDRL_RESETVAL (0x0000u)
N
N/* INTMASKEDRH */
N
N
N#define CSL_USB_INTMASKEDRH_USB_MASK (0x01FFu)
N#define CSL_USB_INTMASKEDRH_USB_SHIFT (0x0000u)
N#define CSL_USB_INTMASKEDRH_USB_RESETVAL (0x0000u)
N
N#define CSL_USB_INTMASKEDRH_RESETVAL (0x0000u)
N
N/* EOIR */
N
N
N#define CSL_USB_EOIR_VECTOR_MASK (0x00FFu)
N#define CSL_USB_EOIR_VECTOR_SHIFT (0x0000u)
N#define CSL_USB_EOIR_VECTOR_RESETVAL (0x0000u)
N
N#define CSL_USB_EOIR_RESETVAL (0x0000u)
N
N/* INTVECTRL */
N
N#define CSL_USB_INTVECTRL_VECTOR_MASK (0xFFFFu)
N#define CSL_USB_INTVECTRL_VECTOR_SHIFT (0x0000u)
N#define CSL_USB_INTVECTRL_VECTOR_RESETVAL (0x0000u)
N
N#define CSL_USB_INTVECTRL_RESETVAL (0x0000u)
N
N/* INTVECTRH */
N
N#define CSL_USB_INTVECTRH_VECTOR_MASK (0xFFFFu)
N#define CSL_USB_INTVECTRH_VECTOR_SHIFT (0x0000u)
N#define CSL_USB_INTVECTRH_VECTOR_RESETVAL (0x0000u)
N
N#define CSL_USB_INTVECTRH_RESETVAL (0x0000u)
N
N/* GRNDISEP1L */
N
N#define CSL_USB_GRNDISEP1L_SIZE_MASK (0xFFFFu)
N#define CSL_USB_GRNDISEP1L_SIZE_SHIFT (0x0000u)
N#define CSL_USB_GRNDISEP1L_SIZE_RESETVAL (0x0000u)
N
N#define CSL_USB_GRNDISEP1L_RESETVAL (0x0000u)
N
N/* GRNDISEP1H */
N
N
N#define CSL_USB_GRNDISEP1H_SIZE_MASK (0x0001u)
N#define CSL_USB_GRNDISEP1H_SIZE_SHIFT (0x0000u)
N#define CSL_USB_GRNDISEP1H_SIZE_RESETVAL (0x0000u)
N
N#define CSL_USB_GRNDISEP1H_RESETVAL (0x0000u)
N
N/* GRNDISEP2L */
N
N#define CSL_USB_GRNDISEP2L_SIZE_MASK (0xFFFFu)
N#define CSL_USB_GRNDISEP2L_SIZE_SHIFT (0x0000u)
N#define CSL_USB_GRNDISEP2L_SIZE_RESETVAL (0x0000u)
N
N#define CSL_USB_GRNDISEP2L_RESETVAL (0x0000u)
N
N/* GRNDISEP2H */
N
N
N#define CSL_USB_GRNDISEP2H_SIZE_MASK (0x0001u)
N#define CSL_USB_GRNDISEP2H_SIZE_SHIFT (0x0000u)
N#define CSL_USB_GRNDISEP2H_SIZE_RESETVAL (0x0000u)
N
N#define CSL_USB_GRNDISEP2H_RESETVAL (0x0000u)
N
N/* GRNDISEP3L */
N
N#define CSL_USB_GRNDISEP3L_SIZE_MASK (0xFFFFu)
N#define CSL_USB_GRNDISEP3L_SIZE_SHIFT (0x0000u)
N#define CSL_USB_GRNDISEP3L_SIZE_RESETVAL (0x0000u)
N
N#define CSL_USB_GRNDISEP3L_RESETVAL (0x0000u)
N
N/* GRNDISEP3H */
N
N
N#define CSL_USB_GRNDISEP3H_SIZE_MASK (0x0001u)
N#define CSL_USB_GRNDISEP3H_SIZE_SHIFT (0x0000u)
N#define CSL_USB_GRNDISEP3H_SIZE_RESETVAL (0x0000u)
N
N#define CSL_USB_GRNDISEP3H_RESETVAL (0x0000u)
N
N/* GRNDISEP4L */
N
N#define CSL_USB_GRNDISEP4L_SIZE_MASK (0xFFFFu)
N#define CSL_USB_GRNDISEP4L_SIZE_SHIFT (0x0000u)
N#define CSL_USB_GRNDISEP4L_SIZE_RESETVAL (0x0000u)
N
N#define CSL_USB_GRNDISEP4L_RESETVAL (0x0000u)
N
N/* GRNDISEP4H */
N
N
N#define CSL_USB_GRNDISEP4H_SIZE_MASK (0x0001u)
N#define CSL_USB_GRNDISEP4H_SIZE_SHIFT (0x0000u)
N#define CSL_USB_GRNDISEP4H_SIZE_RESETVAL (0x0000u)
N
N#define CSL_USB_GRNDISEP4H_RESETVAL (0x0000u)
N
N/* FADDR_POWER */
N
N#define CSL_USB_FADDR_POWER_ISOUPDATE_MASK (0x8000u)
N#define CSL_USB_FADDR_POWER_ISOUPDATE_SHIFT (0x000Fu)
N#define CSL_USB_FADDR_POWER_ISOUPDATE_RESETVAL (0x0000u)
N
N#define CSL_USB_FADDR_POWER_SOFTCONN_MASK (0x4000u)
N#define CSL_USB_FADDR_POWER_SOFTCONN_SHIFT (0x000Eu)
N#define CSL_USB_FADDR_POWER_SOFTCONN_RESETVAL (0x0000u)
N
N#define CSL_USB_FADDR_POWER_HSEN_MASK (0x2000u)
N#define CSL_USB_FADDR_POWER_HSEN_SHIFT (0x000Du)
N#define CSL_USB_FADDR_POWER_HSEN_RESETVAL (0x0001u)
N
N#define CSL_USB_FADDR_POWER_HSMODE_MASK (0x1000u)
N#define CSL_USB_FADDR_POWER_HSMODE_SHIFT (0x000Cu)
N#define CSL_USB_FADDR_POWER_HSMODE_RESETVAL (0x0000u)
N
N#define CSL_USB_FADDR_POWER_RESET_MASK (0x0800u)
N#define CSL_USB_FADDR_POWER_RESET_SHIFT (0x000Bu)
N#define CSL_USB_FADDR_POWER_RESET_RESETVAL (0x0000u)
N
N#define CSL_USB_FADDR_POWER_RESUME_MASK (0x0C00u)
N#define CSL_USB_FADDR_POWER_RESUME_SHIFT (0x000Au)
N#define CSL_USB_FADDR_POWER_RESUME_RESETVAL (0x0000u)
N
N#define CSL_USB_FADDR_POWER_SUSPENDM_MASK (0x0200u)
N#define CSL_USB_FADDR_POWER_SUSPENDM_SHIFT (0x0009u)
N#define CSL_USB_FADDR_POWER_SUSPENDM_RESETVAL (0x0000u)
N
N#define CSL_USB_FADDR_POWER_ENSUSPM_MASK (0x0100u)
N#define CSL_USB_FADDR_POWER_ENSUSPM_SHIFT (0x0008u)
N#define CSL_USB_FADDR_POWER_ENSUSPM_RESETVAL (0x0000u)
N
N
N#define CSL_USB_FADDR_POWER_FUNCADDR_MASK (0x007Fu)
N#define CSL_USB_FADDR_POWER_FUNCADDR_SHIFT (0x0000u)
N#define CSL_USB_FADDR_POWER_FUNCADDR_RESETVAL (0x0000u)
N
N#define CSL_USB_FADDR_POWER_RESETVAL (0x2000u)
N
N/* INTRTX */
N
N
N#define CSL_USB_INTRTX_EP4TX_MASK (0x0010u)
N#define CSL_USB_INTRTX_EP4TX_SHIFT (0x0004u)
N#define CSL_USB_INTRTX_EP4TX_RESETVAL (0x0000u)
N
N#define CSL_USB_INTRTX_EP3TX_MASK (0x0008u)
N#define CSL_USB_INTRTX_EP3TX_SHIFT (0x0003u)
N#define CSL_USB_INTRTX_EP3TX_RESETVAL (0x0000u)
N
N#define CSL_USB_INTRTX_EP2TX_MASK (0x0004u)
N#define CSL_USB_INTRTX_EP2TX_SHIFT (0x0002u)
N#define CSL_USB_INTRTX_EP2TX_RESETVAL (0x0000u)
N
N#define CSL_USB_INTRTX_EP1TX_MASK (0x0002u)
N#define CSL_USB_INTRTX_EP1TX_SHIFT (0x0001u)
N#define CSL_USB_INTRTX_EP1TX_RESETVAL (0x0000u)
N
N#define CSL_USB_INTRTX_EP0_MASK (0x0001u)
N#define CSL_USB_INTRTX_EP0_SHIFT (0x0000u)
N#define CSL_USB_INTRTX_EP0_RESETVAL (0x0000u)
N
N#define CSL_USB_INTRTX_RESETVAL (0x0000u)
N
N/* INTRRX */
N
N
N#define CSL_USB_INTRRX_EP4RX_MASK (0x0010u)
N#define CSL_USB_INTRRX_EP4RX_SHIFT (0x0004u)
N#define CSL_USB_INTRRX_EP4RX_RESETVAL (0x0000u)
N
N#define CSL_USB_INTRRX_EP3RX_MASK (0x0008u)
N#define CSL_USB_INTRRX_EP3RX_SHIFT (0x0003u)
N#define CSL_USB_INTRRX_EP3RX_RESETVAL (0x0000u)
N
N#define CSL_USB_INTRRX_EP2RX_MASK (0x0004u)
N#define CSL_USB_INTRRX_EP2RX_SHIFT (0x0002u)
N#define CSL_USB_INTRRX_EP2RX_RESETVAL (0x0000u)
N
N#define CSL_USB_INTRRX_EP1RX_MASK (0x0002u)
N#define CSL_USB_INTRRX_EP1RX_SHIFT (0x0001u)
N#define CSL_USB_INTRRX_EP1RX_RESETVAL (0x0000u)
N
N
N#define CSL_USB_INTRRX_RESETVAL (0x0000u)
N
N/* INTRTXE */
N
N
N#define CSL_USB_INTRTXE_EP4TX_MASK (0x0010u)
N#define CSL_USB_INTRTXE_EP4TX_SHIFT (0x0004u)
N#define CSL_USB_INTRTXE_EP4TX_RESETVAL (0x0001u)
N
N#define CSL_USB_INTRTXE_EP3TX_MASK (0x0008u)
N#define CSL_USB_INTRTXE_EP3TX_SHIFT (0x0003u)
N#define CSL_USB_INTRTXE_EP3TX_RESETVAL (0x0001u)
N
N#define CSL_USB_INTRTXE_EP2TX_MASK (0x0004u)
N#define CSL_USB_INTRTXE_EP2TX_SHIFT (0x0002u)
N#define CSL_USB_INTRTXE_EP2TX_RESETVAL (0x0001u)
N
N#define CSL_USB_INTRTXE_EP1TX_MASK (0x0002u)
N#define CSL_USB_INTRTXE_EP1TX_SHIFT (0x0001u)
N#define CSL_USB_INTRTXE_EP1TX_RESETVAL (0x0001u)
N
N#define CSL_USB_INTRTXE_EP0_MASK (0x0001u)
N#define CSL_USB_INTRTXE_EP0_SHIFT (0x0000u)
N#define CSL_USB_INTRTXE_EP0_RESETVAL (0x0001u)
N
N#define CSL_USB_INTRTXE_RESETVAL (0x001Fu)
N
N/* INTRRXE */
N
N
N#define CSL_USB_INTRRXE_EP4RX_MASK (0x0010u)
N#define CSL_USB_INTRRXE_EP4RX_SHIFT (0x0004u)
N#define CSL_USB_INTRRXE_EP4RX_RESETVAL (0x0001u)
N
N#define CSL_USB_INTRRXE_EP3RX_MASK (0x0008u)
N#define CSL_USB_INTRRXE_EP3RX_SHIFT (0x0003u)
N#define CSL_USB_INTRRXE_EP3RX_RESETVAL (0x0001u)
N
N#define CSL_USB_INTRRXE_EP2RX_MASK (0x0004u)
N#define CSL_USB_INTRRXE_EP2RX_SHIFT (0x0002u)
N#define CSL_USB_INTRRXE_EP2RX_RESETVAL (0x0001u)
N
N#define CSL_USB_INTRRXE_EP1RX_MASK (0x0002u)
N#define CSL_USB_INTRRXE_EP1RX_SHIFT (0x0001u)
N#define CSL_USB_INTRRXE_EP1RX_RESETVAL (0x0001u)
N
N
N#define CSL_USB_INTRRXE_RESETVAL (0x001Eu)
N
N/* INTRUSB_INTRUSBE */
N
N#define CSL_USB_INTRUSB_INTRUSBE_VBUSERR_E_MASK (0x8000u)
N#define CSL_USB_INTRUSB_INTRUSBE_VBUSERR_E_SHIFT (0x000Fu)
N#define CSL_USB_INTRUSB_INTRUSBE_VBUSERR_E_RESETVAL (0x0000u)
N
N#define CSL_USB_INTRUSB_INTRUSBE_SESSREQ_E_MASK (0x4000u)
N#define CSL_USB_INTRUSB_INTRUSBE_SESSREQ_E_SHIFT (0x000Eu)
N#define CSL_USB_INTRUSB_INTRUSBE_SESSREQ_E_RESETVAL (0x0000u)
N
N#define CSL_USB_INTRUSB_INTRUSBE_DISCON_E_MASK (0x2000u)
N#define CSL_USB_INTRUSB_INTRUSBE_DISCON_E_SHIFT (0x000Du)
N#define CSL_USB_INTRUSB_INTRUSBE_DISCON_E_RESETVAL (0x0000u)
N
N#define CSL_USB_INTRUSB_INTRUSBE_CONN_E_MASK (0x1000u)
N#define CSL_USB_INTRUSB_INTRUSBE_CONN_E_SHIFT (0x000Cu)
N#define CSL_USB_INTRUSB_INTRUSBE_CONN_E_RESETVAL (0x0000u)
N
N#define CSL_USB_INTRUSB_INTRUSBE_SOF_E_MASK (0x0800u)
N#define CSL_USB_INTRUSB_INTRUSBE_SOF_E_SHIFT (0x000Bu)
N#define CSL_USB_INTRUSB_INTRUSBE_SOF_E_RESETVAL (0x0000u)
N
N#define CSL_USB_INTRUSB_INTRUSBE_RESET_BABBLE_E_MASK (0x0400u)
N#define CSL_USB_INTRUSB_INTRUSBE_RESET_BABBLE_E_SHIFT (0x000Au)
N#define CSL_USB_INTRUSB_INTRUSBE_RESET_BABBLE_E_RESETVAL (0x0001u)
N
N#define CSL_USB_INTRUSB_INTRUSBE_RESUME_E_MASK (0x0200u)
N#define CSL_USB_INTRUSB_INTRUSBE_RESUME_E_SHIFT (0x0009u)
N#define CSL_USB_INTRUSB_INTRUSBE_RESUME_E_RESETVAL (0x0001u)
N
N#define CSL_USB_INTRUSB_INTRUSBE_SUSPEND_E_MASK (0x0100u)
N#define CSL_USB_INTRUSB_INTRUSBE_SUSPEND_E_SHIFT (0x0008u)
N#define CSL_USB_INTRUSB_INTRUSBE_SUSPEND_E_RESETVAL (0x0000u)
N
N#define CSL_USB_INTRUSB_INTRUSBE_VBUSERR_MASK (0x0080u)
N#define CSL_USB_INTRUSB_INTRUSBE_VBUSERR_SHIFT (0x0007u)
N#define CSL_USB_INTRUSB_INTRUSBE_VBUSERR_RESETVAL (0x0000u)
N
N#define CSL_USB_INTRUSB_INTRUSBE_SESSREQ_MASK (0x0040u)
N#define CSL_USB_INTRUSB_INTRUSBE_SESSREQ_SHIFT (0x0006u)
N#define CSL_USB_INTRUSB_INTRUSBE_SESSREQ_RESETVAL (0x0000u)
N
N#define CSL_USB_INTRUSB_INTRUSBE_DISCON_MASK (0x0020u)
N#define CSL_USB_INTRUSB_INTRUSBE_DISCON_SHIFT (0x0005u)
N#define CSL_USB_INTRUSB_INTRUSBE_DISCON_RESETVAL (0x0000u)
N
N#define CSL_USB_INTRUSB_INTRUSBE_CONN_MASK (0x0010u)
N#define CSL_USB_INTRUSB_INTRUSBE_CONN_SHIFT (0x0004u)
N#define CSL_USB_INTRUSB_INTRUSBE_CONN_RESETVAL (0x0000u)
N
N#define CSL_USB_INTRUSB_INTRUSBE_SOF_MASK (0x0008u)
N#define CSL_USB_INTRUSB_INTRUSBE_SOF_SHIFT (0x0003u)
N#define CSL_USB_INTRUSB_INTRUSBE_SOF_RESETVAL (0x0000u)
N
N#define CSL_USB_INTRUSB_INTRUSBE_RESET_BABBLE_MASK (0x0004u)
N#define CSL_USB_INTRUSB_INTRUSBE_RESET_BABBLE_SHIFT (0x0002u)
N#define CSL_USB_INTRUSB_INTRUSBE_RESET_BABBLE_RESETVAL (0x0000u)
N
N#define CSL_USB_INTRUSB_INTRUSBE_RESUME_MASK (0x0002u)
N#define CSL_USB_INTRUSB_INTRUSBE_RESUME_SHIFT (0x0001u)
N#define CSL_USB_INTRUSB_INTRUSBE_RESUME_RESETVAL (0x0000u)
N
N#define CSL_USB_INTRUSB_INTRUSBE_SUSPEND_MASK (0x0001u)
N#define CSL_USB_INTRUSB_INTRUSBE_SUSPEND_SHIFT (0x0000u)
N#define CSL_USB_INTRUSB_INTRUSBE_SUSPEND_RESETVAL (0x0000u)
N
N#define CSL_USB_INTRUSB_INTRUSBE_RESETVAL (0x0600u)
N
N/* FRAME */
N
N
N#define CSL_USB_FRAME_FRAMENUMBER_MASK (0x07FFu)
N#define CSL_USB_FRAME_FRAMENUMBER_SHIFT (0x0000u)
N#define CSL_USB_FRAME_FRAMENUMBER_RESETVAL (0x0000u)
N
N#define CSL_USB_FRAME_RESETVAL (0x0000u)
N
N/* INDEX_TESTMODE */
N
N#define CSL_USB_INDEX_TESTMODE_FORCE_HOST_MASK (0x8000u)
N#define CSL_USB_INDEX_TESTMODE_FORCE_HOST_SHIFT (0x000Fu)
N#define CSL_USB_INDEX_TESTMODE_FORCE_HOST_RESETVAL (0x0000u)
N
N#define CSL_USB_INDEX_TESTMODE_FIFO_ACCESS_MASK (0x4000u)
N#define CSL_USB_INDEX_TESTMODE_FIFO_ACCESS_SHIFT (0x000Eu)
N#define CSL_USB_INDEX_TESTMODE_FIFO_ACCESS_RESETVAL (0x0000u)
N
N#define CSL_USB_INDEX_TESTMODE_FORCE_FS_MASK (0x2000u)
N#define CSL_USB_INDEX_TESTMODE_FORCE_FS_SHIFT (0x000Du)
N#define CSL_USB_INDEX_TESTMODE_FORCE_FS_RESETVAL (0x0000u)
N
N#define CSL_USB_INDEX_TESTMODE_FORCE_HS_MASK (0x1000u)
N#define CSL_USB_INDEX_TESTMODE_FORCE_HS_SHIFT (0x000Cu)
N#define CSL_USB_INDEX_TESTMODE_FORCE_HS_RESETVAL (0x0000u)
N
N#define CSL_USB_INDEX_TESTMODE_TEST_PACKET_MASK (0x0800u)
N#define CSL_USB_INDEX_TESTMODE_TEST_PACKET_SHIFT (0x000Bu)
N#define CSL_USB_INDEX_TESTMODE_TEST_PACKET_RESETVAL (0x0000u)
N
N#define CSL_USB_INDEX_TESTMODE_TEST_K_MASK (0x0400u)
N#define CSL_USB_INDEX_TESTMODE_TEST_K_SHIFT (0x000Au)
N#define CSL_USB_INDEX_TESTMODE_TEST_K_RESETVAL (0x0000u)
N
N#define CSL_USB_INDEX_TESTMODE_TEST_J_MASK (0x0200u)
N#define CSL_USB_INDEX_TESTMODE_TEST_J_SHIFT (0x0009u)
N#define CSL_USB_INDEX_TESTMODE_TEST_J_RESETVAL (0x0000u)
N
N#define CSL_USB_INDEX_TESTMODE_TEST_SE0_NAK_MASK (0x0100u)
N#define CSL_USB_INDEX_TESTMODE_TEST_SE0_NAK_SHIFT (0x0008u)
N#define CSL_USB_INDEX_TESTMODE_TEST_SE0_NAK_RESETVAL (0x0000u)
N
N
N#define CSL_USB_INDEX_TESTMODE_EPSEL_MASK (0x000Fu)
N#define CSL_USB_INDEX_TESTMODE_EPSEL_SHIFT (0x0000u)
N#define CSL_USB_INDEX_TESTMODE_EPSEL_RESETVAL (0x0000u)
N
N#define CSL_USB_INDEX_TESTMODE_RESETVAL (0x0000u)
N
N/* TXMAXP */
N
N
N#define CSL_USB_TXMAXP_MAXPAYLOAD_MASK (0x07FFu)
N#define CSL_USB_TXMAXP_MAXPAYLOAD_SHIFT (0x0000u)
N#define CSL_USB_TXMAXP_MAXPAYLOAD_RESETVAL (0x0000u)
N
N#define CSL_USB_TXMAXP_RESETVAL (0x0000u)
N
N/* PERI_CSR0 */
N
N
N#define CSL_USB_PERI_CSR0_FLUSHFIFO_MASK (0x0100u)
N#define CSL_USB_PERI_CSR0_FLUSHFIFO_SHIFT (0x0008u)
N#define CSL_USB_PERI_CSR0_FLUSHFIFO_RESETVAL (0x0000u)
N
N#define CSL_USB_PERI_CSR0_SERV_SETUPEND_MASK (0x0080u)
N#define CSL_USB_PERI_CSR0_SERV_SETUPEND_SHIFT (0x0007u)
N#define CSL_USB_PERI_CSR0_SERV_SETUPEND_RESETVAL (0x0000u)
N
N#define CSL_USB_PERI_CSR0_SERV_RXPKTRDY_MASK (0x0040u)
N#define CSL_USB_PERI_CSR0_SERV_RXPKTRDY_SHIFT (0x0006u)
N#define CSL_USB_PERI_CSR0_SERV_RXPKTRDY_RESETVAL (0x0000u)
N
N#define CSL_USB_PERI_CSR0_SENDSTALL_MASK (0x0020u)
N#define CSL_USB_PERI_CSR0_SENDSTALL_SHIFT (0x0005u)
N#define CSL_USB_PERI_CSR0_SENDSTALL_RESETVAL (0x0000u)
N
N#define CSL_USB_PERI_CSR0_SETUPEND_MASK (0x0010u)
N#define CSL_USB_PERI_CSR0_SETUPEND_SHIFT (0x0004u)
N#define CSL_USB_PERI_CSR0_SETUPEND_RESETVAL (0x0000u)
N
N#define CSL_USB_PERI_CSR0_DATAEND_MASK (0x0008u)
N#define CSL_USB_PERI_CSR0_DATAEND_SHIFT (0x0003u)
N#define CSL_USB_PERI_CSR0_DATAEND_RESETVAL (0x0000u)
N
N#define CSL_USB_PERI_CSR0_SENTSTALL_MASK (0x0004u)
N#define CSL_USB_PERI_CSR0_SENTSTALL_SHIFT (0x0002u)
N#define CSL_USB_PERI_CSR0_SENTSTALL_RESETVAL (0x0000u)
N
N#define CSL_USB_PERI_CSR0_TXPKTRDY_MASK (0x0002u)
N#define CSL_USB_PERI_CSR0_TXPKTRDY_SHIFT (0x0001u)
N#define CSL_USB_PERI_CSR0_TXPKTRDY_RESETVAL (0x0000u)
N
N#define CSL_USB_PERI_CSR0_RXPKTRDY_MASK (0x0001u)
N#define CSL_USB_PERI_CSR0_RXPKTRDY_SHIFT (0x0000u)
N#define CSL_USB_PERI_CSR0_RXPKTRDY_RESETVAL (0x0000u)
N
N#define CSL_USB_PERI_CSR0_RESETVAL (0x0000u)
N
N/* HOST_CSR0 */
N
N
N#define CSL_USB_HOST_CSR0_DATATOGWREN_MASK (0x0400u)
N#define CSL_USB_HOST_CSR0_DATATOGWREN_SHIFT (0x000Au)
N#define CSL_USB_HOST_CSR0_DATATOGWREN_RESETVAL (0x0000u)
N
N#define CSL_USB_HOST_CSR0_DATATOG_MASK (0x0200u)
N#define CSL_USB_HOST_CSR0_DATATOG_SHIFT (0x0009u)
N#define CSL_USB_HOST_CSR0_DATATOG_RESETVAL (0x0000u)
N
N#define CSL_USB_HOST_CSR0_FLUSHFIFO_MASK (0x0100u)
N#define CSL_USB_HOST_CSR0_FLUSHFIFO_SHIFT (0x0008u)
N#define CSL_USB_HOST_CSR0_FLUSHFIFO_RESETVAL (0x0000u)
N
N#define CSL_USB_HOST_CSR0_NAK_TIMEOUT_MASK (0x0080u)
N#define CSL_USB_HOST_CSR0_NAK_TIMEOUT_SHIFT (0x0007u)
N#define CSL_USB_HOST_CSR0_NAK_TIMEOUT_RESETVAL (0x0000u)
N
N#define CSL_USB_HOST_CSR0_STATUSPKT_MASK (0x0040u)
N#define CSL_USB_HOST_CSR0_STATUSPKT_SHIFT (0x0006u)
N#define CSL_USB_HOST_CSR0_STATUSPKT_RESETVAL (0x0000u)
N
N#define CSL_USB_HOST_CSR0_REQPKT_MASK (0x0020u)
N#define CSL_USB_HOST_CSR0_REQPKT_SHIFT (0x0005u)
N#define CSL_USB_HOST_CSR0_REQPKT_RESETVAL (0x0000u)
N
N#define CSL_USB_HOST_CSR0_ERROR_MASK (0x0010u)
N#define CSL_USB_HOST_CSR0_ERROR_SHIFT (0x0004u)
N#define CSL_USB_HOST_CSR0_ERROR_RESETVAL (0x0000u)
N
N#define CSL_USB_HOST_CSR0_SETUPPKT_MASK (0x0008u)
N#define CSL_USB_HOST_CSR0_SETUPPKT_SHIFT (0x0003u)
N#define CSL_USB_HOST_CSR0_SETUPPKT_RESETVAL (0x0000u)
N
N#define CSL_USB_HOST_CSR0_RXSTALL_MASK (0x0004u)
N#define CSL_USB_HOST_CSR0_RXSTALL_SHIFT (0x0002u)
N#define CSL_USB_HOST_CSR0_RXSTALL_RESETVAL (0x0000u)
N
N#define CSL_USB_HOST_CSR0_TXPKTRDY_MASK (0x0002u)
N#define CSL_USB_HOST_CSR0_TXPKTRDY_SHIFT (0x0001u)
N#define CSL_USB_HOST_CSR0_TXPKTRDY_RESETVAL (0x0000u)
N
N#define CSL_USB_HOST_CSR0_RXPKTRDY_MASK (0x0001u)
N#define CSL_USB_HOST_CSR0_RXPKTRDY_SHIFT (0x0000u)
N#define CSL_USB_HOST_CSR0_RXPKTRDY_RESETVAL (0x0000u)
N
N#define CSL_USB_HOST_CSR0_RESETVAL (0x0000u)
N
N/* PERI_TXCSR */
N
N#define CSL_USB_PERI_TXCSR_AUTOSET_MASK (0x8000u)
N#define CSL_USB_PERI_TXCSR_AUTOSET_SHIFT (0x000Fu)
N#define CSL_USB_PERI_TXCSR_AUTOSET_RESETVAL (0x0000u)
N
N#define CSL_USB_PERI_TXCSR_ISO_MASK (0x4000u)
N#define CSL_USB_PERI_TXCSR_ISO_SHIFT (0x000Eu)
N#define CSL_USB_PERI_TXCSR_ISO_RESETVAL (0x0000u)
N
N#define CSL_USB_PERI_TXCSR_MODE_MASK (0x2000u)
N#define CSL_USB_PERI_TXCSR_MODE_SHIFT (0x000Du)
N#define CSL_USB_PERI_TXCSR_MODE_RESETVAL (0x0001u)
N
N#define CSL_USB_PERI_TXCSR_DMAEN_MASK (0x1000u)
N#define CSL_USB_PERI_TXCSR_DMAEN_SHIFT (0x000Cu)
N#define CSL_USB_PERI_TXCSR_DMAEN_RESETVAL (0x0000u)
N
N#define CSL_USB_PERI_TXCSR_FRCDATATOG_MASK (0x0800u)
N#define CSL_USB_PERI_TXCSR_FRCDATATOG_SHIFT (0x000Bu)
N#define CSL_USB_PERI_TXCSR_FRCDATATOG_RESETVAL (0x0000u)
N
N#define CSL_USB_PERI_TXCSR_DMAMODE_MASK (0x0400u)
N#define CSL_USB_PERI_TXCSR_DMAMODE_SHIFT (0x000Au)
N#define CSL_USB_PERI_TXCSR_DMAMODE_RESETVAL (0x0000u)
N
N
N#define CSL_USB_PERI_TXCSR_CLRDATATOG_MASK (0x0040u)
N#define CSL_USB_PERI_TXCSR_CLRDATATOG_SHIFT (0x0006u)
N#define CSL_USB_PERI_TXCSR_CLRDATATOG_RESETVAL (0x0000u)
N
N#define CSL_USB_PERI_TXCSR_SENTSTALL_MASK (0x0020u)
N#define CSL_USB_PERI_TXCSR_SENTSTALL_SHIFT (0x0005u)
N#define CSL_USB_PERI_TXCSR_SENTSTALL_RESETVAL (0x0000u)
N
N#define CSL_USB_PERI_TXCSR_SENDSTALL_MASK (0x0010u)
N#define CSL_USB_PERI_TXCSR_SENDSTALL_SHIFT (0x0004u)
N#define CSL_USB_PERI_TXCSR_SENDSTALL_RESETVAL (0x0000u)
N
N#define CSL_USB_PERI_TXCSR_FLUSHFIFO_MASK (0x0008u)
N#define CSL_USB_PERI_TXCSR_FLUSHFIFO_SHIFT (0x0003u)
N#define CSL_USB_PERI_TXCSR_FLUSHFIFO_RESETVAL (0x0000u)
N
N#define CSL_USB_PERI_TXCSR_UNDERRUN_MASK (0x0004u)
N#define CSL_USB_PERI_TXCSR_UNDERRUN_SHIFT (0x0002u)
N#define CSL_USB_PERI_TXCSR_UNDERRUN_RESETVAL (0x0000u)
N
N#define CSL_USB_PERI_TXCSR_FIFONOTEMPTY_MASK (0x0002u)
N#define CSL_USB_PERI_TXCSR_FIFONOTEMPTY_SHIFT (0x0001u)
N#define CSL_USB_PERI_TXCSR_FIFONOTEMPTY_RESETVAL (0x0000u)
N
N#define CSL_USB_PERI_TXCSR_TXPKTRDY_MASK (0x0001u)
N#define CSL_USB_PERI_TXCSR_TXPKTRDY_SHIFT (0x0000u)
N#define CSL_USB_PERI_TXCSR_TXPKTRDY_RESETVAL (0x0000u)
N
N#define CSL_USB_PERI_TXCSR_RESETVAL (0x2000u)
N
N/* HOST_TXCSR */
N
N#define CSL_USB_HOST_TXCSR_AUTOSET_MASK (0x8000u)
N#define CSL_USB_HOST_TXCSR_AUTOSET_SHIFT (0x000Fu)
N#define CSL_USB_HOST_TXCSR_AUTOSET_RESETVAL (0x0000u)
N
N
N#define CSL_USB_HOST_TXCSR_MODE_MASK (0x2000u)
N#define CSL_USB_HOST_TXCSR_MODE_SHIFT (0x000Du)
N#define CSL_USB_HOST_TXCSR_MODE_RESETVAL (0x0000u)
N
N#define CSL_USB_HOST_TXCSR_DMAEN_MASK (0x1000u)
N#define CSL_USB_HOST_TXCSR_DMAEN_SHIFT (0x000Cu)
N#define CSL_USB_HOST_TXCSR_DMAEN_RESETVAL (0x0000u)
N
N#define CSL_USB_HOST_TXCSR_FRCDATATOG_MASK (0x0800u)
N#define CSL_USB_HOST_TXCSR_FRCDATATOG_SHIFT (0x000Bu)
N#define CSL_USB_HOST_TXCSR_FRCDATATOG_RESETVAL (0x0000u)
N
N#define CSL_USB_HOST_TXCSR_DMAMODE_MASK (0x0400u)
N#define CSL_USB_HOST_TXCSR_DMAMODE_SHIFT (0x000Au)
N#define CSL_USB_HOST_TXCSR_DMAMODE_RESETVAL (0x0000u)
N
N#define CSL_USB_HOST_TXCSR_DATATOGWREN_MASK (0x0200u)
N#define CSL_USB_HOST_TXCSR_DATATOGWREN_SHIFT (0x0009u)
N#define CSL_USB_HOST_TXCSR_DATATOGWREN_RESETVAL (0x0000u)
N
N#define CSL_USB_HOST_TXCSR_DATATOG_MASK (0x0100u)
N#define CSL_USB_HOST_TXCSR_DATATOG_SHIFT (0x0008u)
N#define CSL_USB_HOST_TXCSR_DATATOG_RESETVAL (0x0000u)
N
N#define CSL_USB_HOST_TXCSR_NAK_TIMEOUT_MASK (0x0080u)
N#define CSL_USB_HOST_TXCSR_NAK_TIMEOUT_SHIFT (0x0007u)
N#define CSL_USB_HOST_TXCSR_NAK_TIMEOUT_RESETVAL (0x0000u)
N
N#define CSL_USB_HOST_TXCSR_CLRDATATOG_MASK (0x0040u)
N#define CSL_USB_HOST_TXCSR_CLRDATATOG_SHIFT (0x0006u)
N#define CSL_USB_HOST_TXCSR_CLRDATATOG_RESETVAL (0x0000u)
N
N#define CSL_USB_HOST_TXCSR_RXSTALL_MASK (0x0020u)
N#define CSL_USB_HOST_TXCSR_RXSTALL_SHIFT (0x0005u)
N#define CSL_USB_HOST_TXCSR_RXSTALL_RESETVAL (0x0000u)
N
N#define CSL_USB_HOST_TXCSR_SETUPPKT_MASK (0x0010u)
N#define CSL_USB_HOST_TXCSR_SETUPPKT_SHIFT (0x0004u)
N#define CSL_USB_HOST_TXCSR_SETUPPKT_RESETVAL (0x0000u)
N
N#define CSL_USB_HOST_TXCSR_FLUSHFIFO_MASK (0x0008u)
N#define CSL_USB_HOST_TXCSR_FLUSHFIFO_SHIFT (0x0003u)
N#define CSL_USB_HOST_TXCSR_FLUSHFIFO_RESETVAL (0x0000u)
N
N#define CSL_USB_HOST_TXCSR_ERROR_MASK (0x0004u)
N#define CSL_USB_HOST_TXCSR_ERROR_SHIFT (0x0002u)
N#define CSL_USB_HOST_TXCSR_ERROR_RESETVAL (0x0000u)
N
N#define CSL_USB_HOST_TXCSR_FIFONOTEMPTY_MASK (0x0002u)
N#define CSL_USB_HOST_TXCSR_FIFONOTEMPTY_SHIFT (0x0001u)
N#define CSL_USB_HOST_TXCSR_FIFONOTEMPTY_RESETVAL (0x0000u)
N
N#define CSL_USB_HOST_TXCSR_TXPKTRDY_MASK (0x0001u)
N#define CSL_USB_HOST_TXCSR_TXPKTRDY_SHIFT (0x0000u)
N#define CSL_USB_HOST_TXCSR_TXPKTRDY_RESETVAL (0x0000u)
N
N#define CSL_USB_HOST_TXCSR_RESETVAL (0x0000u)
N
N/* RXMAXP */
N
N
N#define CSL_USB_RXMAXP_MAXPAYLOAD_MASK (0x07FFu)
N#define CSL_USB_RXMAXP_MAXPAYLOAD_SHIFT (0x0000u)
N#define CSL_USB_RXMAXP_MAXPAYLOAD_RESETVAL (0x0000u)
N
N#define CSL_USB_RXMAXP_RESETVAL (0x0000u)
N
N/* PERI_RXCSR */
N
N#define CSL_USB_PERI_RXCSR_AUTOCLEAR_MASK (0x8000u)
N#define CSL_USB_PERI_RXCSR_AUTOCLEAR_SHIFT (0x000Fu)
N#define CSL_USB_PERI_RXCSR_AUTOCLEAR_RESETVAL (0x0000u)
N
N#define CSL_USB_PERI_RXCSR_ISO_MASK (0x4000u)
N#define CSL_USB_PERI_RXCSR_ISO_SHIFT (0x000Eu)
N#define CSL_USB_PERI_RXCSR_ISO_RESETVAL (0x0000u)
N
N#define CSL_USB_PERI_RXCSR_DMAEN_MASK (0x2000u)
N#define CSL_USB_PERI_RXCSR_DMAEN_SHIFT (0x000Du)
N#define CSL_USB_PERI_RXCSR_DMAEN_RESETVAL (0x0000u)
N
N#define CSL_USB_PERI_RXCSR_DISNYET_MASK (0x1000u)
N#define CSL_USB_PERI_RXCSR_DISNYET_SHIFT (0x000Cu)
N#define CSL_USB_PERI_RXCSR_DISNYET_RESETVAL (0x0000u)
N
N#define CSL_USB_PERI_RXCSR_DMAMODE_MASK (0x0800u)
N#define CSL_USB_PERI_RXCSR_DMAMODE_SHIFT (0x000Bu)
N#define CSL_USB_PERI_RXCSR_DMAMODE_RESETVAL (0x0000u)
N
N
N#define CSL_USB_PERI_RXCSR_CLRDATATOG_MASK (0x0080u)
N#define CSL_USB_PERI_RXCSR_CLRDATATOG_SHIFT (0x0007u)
N#define CSL_USB_PERI_RXCSR_CLRDATATOG_RESETVAL (0x0000u)
N
N#define CSL_USB_PERI_RXCSR_SENTSTALL_MASK (0x0040u)
N#define CSL_USB_PERI_RXCSR_SENTSTALL_SHIFT (0x0006u)
N#define CSL_USB_PERI_RXCSR_SENTSTALL_RESETVAL (0x0000u)
N
N#define CSL_USB_PERI_RXCSR_SENDSTALL_MASK (0x0020u)
N#define CSL_USB_PERI_RXCSR_SENDSTALL_SHIFT (0x0005u)
N#define CSL_USB_PERI_RXCSR_SENDSTALL_RESETVAL (0x0000u)
N
N#define CSL_USB_PERI_RXCSR_FLUSHFIFO_MASK (0x0010u)
N#define CSL_USB_PERI_RXCSR_FLUSHFIFO_SHIFT (0x0004u)
N#define CSL_USB_PERI_RXCSR_FLUSHFIFO_RESETVAL (0x0000u)
N
N#define CSL_USB_PERI_RXCSR_DATAERROR_MASK (0x0008u)
N#define CSL_USB_PERI_RXCSR_DATAERROR_SHIFT (0x0003u)
N#define CSL_USB_PERI_RXCSR_DATAERROR_RESETVAL (0x0000u)
N
N#define CSL_USB_PERI_RXCSR_OVERRUN_MASK (0x0004u)
N#define CSL_USB_PERI_RXCSR_OVERRUN_SHIFT (0x0002u)
N#define CSL_USB_PERI_RXCSR_OVERRUN_RESETVAL (0x0000u)
N
N#define CSL_USB_PERI_RXCSR_FIFOFULL_MASK (0x0002u)
N#define CSL_USB_PERI_RXCSR_FIFOFULL_SHIFT (0x0001u)
N#define CSL_USB_PERI_RXCSR_FIFOFULL_RESETVAL (0x0000u)
N
N#define CSL_USB_PERI_RXCSR_RXPKTRDY_MASK (0x0001u)
N#define CSL_USB_PERI_RXCSR_RXPKTRDY_SHIFT (0x0000u)
N#define CSL_USB_PERI_RXCSR_RXPKTRDY_RESETVAL (0x0000u)
N
N#define CSL_USB_PERI_RXCSR_RESETVAL (0x0000u)
N
N/* HOST_RXCSR */
N
N#define CSL_USB_HOST_RXCSR_AUTOCLEAR_MASK (0x8000u)
N#define CSL_USB_HOST_RXCSR_AUTOCLEAR_SHIFT (0x000Fu)
N#define CSL_USB_HOST_RXCSR_AUTOCLEAR_RESETVAL (0x0000u)
N
N#define CSL_USB_HOST_RXCSR_AUTOREQ_MASK (0x4000u)
N#define CSL_USB_HOST_RXCSR_AUTOREQ_SHIFT (0x000Eu)
N#define CSL_USB_HOST_RXCSR_AUTOREQ_RESETVAL (0x0000u)
N
N#define CSL_USB_HOST_RXCSR_DMAEN_MASK (0x2000u)
N#define CSL_USB_HOST_RXCSR_DMAEN_SHIFT (0x000Du)
N#define CSL_USB_HOST_RXCSR_DMAEN_RESETVAL (0x0000u)
N
N#define CSL_USB_HOST_RXCSR_DISNYET_MASK (0x1000u)
N#define CSL_USB_HOST_RXCSR_DISNYET_SHIFT (0x000Cu)
N#define CSL_USB_HOST_RXCSR_DISNYET_RESETVAL (0x0000u)
N
N#define CSL_USB_HOST_RXCSR_DMAMODE_MASK (0x0800u)
N#define CSL_USB_HOST_RXCSR_DMAMODE_SHIFT (0x000Bu)
N#define CSL_USB_HOST_RXCSR_DMAMODE_RESETVAL (0x0000u)
N
N#define CSL_USB_HOST_RXCSR_DATATOGWREN_MASK (0x0400u)
N#define CSL_USB_HOST_RXCSR_DATATOGWREN_SHIFT (0x000Au)
N#define CSL_USB_HOST_RXCSR_DATATOGWREN_RESETVAL (0x0000u)
N
N#define CSL_USB_HOST_RXCSR_DATATOG_MASK (0x0200u)
N#define CSL_USB_HOST_RXCSR_DATATOG_SHIFT (0x0009u)
N#define CSL_USB_HOST_RXCSR_DATATOG_RESETVAL (0x0000u)
N
N
N#define CSL_USB_HOST_RXCSR_CLRDATATOG_MASK (0x0080u)
N#define CSL_USB_HOST_RXCSR_CLRDATATOG_SHIFT (0x0007u)
N#define CSL_USB_HOST_RXCSR_CLRDATATOG_RESETVAL (0x0000u)
N
N#define CSL_USB_HOST_RXCSR_RXSTALL_MASK (0x0040u)
N#define CSL_USB_HOST_RXCSR_RXSTALL_SHIFT (0x0006u)
N#define CSL_USB_HOST_RXCSR_RXSTALL_RESETVAL (0x0000u)
N
N#define CSL_USB_HOST_RXCSR_REQPKT_MASK (0x0020u)
N#define CSL_USB_HOST_RXCSR_REQPKT_SHIFT (0x0005u)
N#define CSL_USB_HOST_RXCSR_REQPKT_RESETVAL (0x0000u)
N
N#define CSL_USB_HOST_RXCSR_FLUSHFIFO_MASK (0x0010u)
N#define CSL_USB_HOST_RXCSR_FLUSHFIFO_SHIFT (0x0004u)
N#define CSL_USB_HOST_RXCSR_FLUSHFIFO_RESETVAL (0x0000u)
N
N#define CSL_USB_HOST_RXCSR_DATAERR_NAKTIMEOUT_MASK (0x0008u)
N#define CSL_USB_HOST_RXCSR_DATAERR_NAKTIMEOUT_SHIFT (0x0003u)
N#define CSL_USB_HOST_RXCSR_DATAERR_NAKTIMEOUT_RESETVAL (0x0000u)
N
N#define CSL_USB_HOST_RXCSR_ERROR_MASK (0x0004u)
N#define CSL_USB_HOST_RXCSR_ERROR_SHIFT (0x0002u)
N#define CSL_USB_HOST_RXCSR_ERROR_RESETVAL (0x0000u)
N
N#define CSL_USB_HOST_RXCSR_FIFOFULL_MASK (0x0002u)
N#define CSL_USB_HOST_RXCSR_FIFOFULL_SHIFT (0x0001u)
N#define CSL_USB_HOST_RXCSR_FIFOFULL_RESETVAL (0x0000u)
N
N#define CSL_USB_HOST_RXCSR_RXPKTRDY_MASK (0x0001u)
N#define CSL_USB_HOST_RXCSR_RXPKTRDY_SHIFT (0x0000u)
N#define CSL_USB_HOST_RXCSR_RXPKTRDY_RESETVAL (0x0000u)
N
N#define CSL_USB_HOST_RXCSR_RESETVAL (0x0000u)
N
N/* COUNT0 */
N
N
N#define CSL_USB_COUNT0_EP0RXCOUNT_MASK (0x007Fu)
N#define CSL_USB_COUNT0_EP0RXCOUNT_SHIFT (0x0000u)
N#define CSL_USB_COUNT0_EP0RXCOUNT_RESETVAL (0x0000u)
N
N#define CSL_USB_COUNT0_RESETVAL (0x0000u)
N
N/* RXCOUNT */
N
N
N#define CSL_USB_RXCOUNT_EPRXCOUNT_MASK (0x1FFFu)
N#define CSL_USB_RXCOUNT_EPRXCOUNT_SHIFT (0x0000u)
N#define CSL_USB_RXCOUNT_EPRXCOUNT_RESETVAL (0x0000u)
N
N#define CSL_USB_RXCOUNT_RESETVAL (0x0000u)
N
N/* HOST_TYPE0_HOST_NAKLIMIT0 */
N
N
N#define CSL_USB_HOST_TYPE0_HOST_NAKLIMIT0_EP0NAKLIMIT_MASK (0x1F00u)
N#define CSL_USB_HOST_TYPE0_HOST_NAKLIMIT0_EP0NAKLIMIT_SHIFT (0x0008u)
N#define CSL_USB_HOST_TYPE0_HOST_NAKLIMIT0_EP0NAKLIMIT_RESETVAL (0x0000u)
N
N#define CSL_USB_HOST_TYPE0_HOST_NAKLIMIT0_SPEED_MASK (0x00C0u)
N#define CSL_USB_HOST_TYPE0_HOST_NAKLIMIT0_SPEED_SHIFT (0x0006u)
N#define CSL_USB_HOST_TYPE0_HOST_NAKLIMIT0_SPEED_RESETVAL (0x0000u)
N
N
N#define CSL_USB_HOST_TYPE0_HOST_NAKLIMIT0_RESETVAL (0x0000u)
N
N/* HOST_TXTYPE_HOST_TXINTERVAL */
N
N#define CSL_USB_HOST_TXTYPE_HOST_TXINTERVAL_POLINTVL_NAKLIMIT_MASK (0xFF00u)
N#define CSL_USB_HOST_TXTYPE_HOST_TXINTERVAL_POLINTVL_NAKLIMIT_SHIFT (0x0008u)
N#define CSL_USB_HOST_TXTYPE_HOST_TXINTERVAL_POLINTVL_NAKLIMIT_RESETVAL (0x0000u)
N
N#define CSL_USB_HOST_TXTYPE_HOST_TXINTERVAL_SPEED_MASK (0x00C0u)
N#define CSL_USB_HOST_TXTYPE_HOST_TXINTERVAL_SPEED_SHIFT (0x0006u)
N#define CSL_USB_HOST_TXTYPE_HOST_TXINTERVAL_SPEED_RESETVAL (0x0000u)
N
N#define CSL_USB_HOST_TXTYPE_HOST_TXINTERVAL_PROT_MASK (0x0030u)
N#define CSL_USB_HOST_TXTYPE_HOST_TXINTERVAL_PROT_SHIFT (0x0004u)
N#define CSL_USB_HOST_TXTYPE_HOST_TXINTERVAL_PROT_RESETVAL (0x0000u)
N
N#define CSL_USB_HOST_TXTYPE_HOST_TXINTERVAL_TENDPN_MASK (0x000Fu)
N#define CSL_USB_HOST_TXTYPE_HOST_TXINTERVAL_TENDPN_SHIFT (0x0000u)
N#define CSL_USB_HOST_TXTYPE_HOST_TXINTERVAL_TENDPN_RESETVAL (0x0000u)
N
N#define CSL_USB_HOST_TXTYPE_HOST_TXINTERVAL_RESETVAL (0x0000u)
N
N/* HOST_RXTYPE_HOST_RXINTERVAL */
N
N#define CSL_USB_HOST_RXTYPE_HOST_RXINTERVAL_POLINTVL_NAKLIMIT_MASK (0xFF00u)
N#define CSL_USB_HOST_RXTYPE_HOST_RXINTERVAL_POLINTVL_NAKLIMIT_SHIFT (0x0008u)
N#define CSL_USB_HOST_RXTYPE_HOST_RXINTERVAL_POLINTVL_NAKLIMIT_RESETVAL (0x0000u)
N
N#define CSL_USB_HOST_RXTYPE_HOST_RXINTERVAL_SPEED_MASK (0x00C0u)
N#define CSL_USB_HOST_RXTYPE_HOST_RXINTERVAL_SPEED_SHIFT (0x0006u)
N#define CSL_USB_HOST_RXTYPE_HOST_RXINTERVAL_SPEED_RESETVAL (0x0000u)
N
N#define CSL_USB_HOST_RXTYPE_HOST_RXINTERVAL_PROT_MASK (0x0030u)
N#define CSL_USB_HOST_RXTYPE_HOST_RXINTERVAL_PROT_SHIFT (0x0004u)
N#define CSL_USB_HOST_RXTYPE_HOST_RXINTERVAL_PROT_RESETVAL (0x0000u)
N
N#define CSL_USB_HOST_RXTYPE_HOST_RXINTERVAL_RENDPN_MASK (0x000Fu)
N#define CSL_USB_HOST_RXTYPE_HOST_RXINTERVAL_RENDPN_SHIFT (0x0000u)
N#define CSL_USB_HOST_RXTYPE_HOST_RXINTERVAL_RENDPN_RESETVAL (0x0000u)
N
N#define CSL_USB_HOST_RXTYPE_HOST_RXINTERVAL_RESETVAL (0x0000u)
N
N/* CONFIGDATA */
N
N
N
N#define CSL_USB_CONFIGDATA_MPTXE_MASK (0x0040u)
N#define CSL_USB_CONFIGDATA_MPTXE_SHIFT (0x0006u)
N#define CSL_USB_CONFIGDATA_MPTXE_RESETVAL (0x0000u)
N
N#define CSL_USB_CONFIGDATA_BIGENDIAN_MASK (0x0020u)
N#define CSL_USB_CONFIGDATA_BIGENDIAN_SHIFT (0x0005u)
N#define CSL_USB_CONFIGDATA_BIGENDIAN_RESETVAL (0x0000u)
N
N#define CSL_USB_CONFIGDATA_HBRXE_MASK (0x0010u)
N#define CSL_USB_CONFIGDATA_HBRXE_SHIFT (0x0004u)
N#define CSL_USB_CONFIGDATA_HBRXE_RESETVAL (0x0000u)
N
N#define CSL_USB_CONFIGDATA_HBTXE_MASK (0x0008u)
N#define CSL_USB_CONFIGDATA_HBTXE_SHIFT (0x0003u)
N#define CSL_USB_CONFIGDATA_HBTXE_RESETVAL (0x0000u)
N
N#define CSL_USB_CONFIGDATA_DYNFIFO_MASK (0x0004u)
N#define CSL_USB_CONFIGDATA_DYNFIFO_SHIFT (0x0002u)
N#define CSL_USB_CONFIGDATA_DYNFIFO_RESETVAL (0x0000u)
N
N#define CSL_USB_CONFIGDATA_SOFTCONE_MASK (0x0002u)
N#define CSL_USB_CONFIGDATA_SOFTCONE_SHIFT (0x0001u)
N#define CSL_USB_CONFIGDATA_SOFTCONE_RESETVAL (0x0000u)
N
N#define CSL_USB_CONFIGDATA_UTMIDATAWIDTH_MASK (0x0001u)
N#define CSL_USB_CONFIGDATA_UTMIDATAWIDTH_SHIFT (0x0000u)
N#define CSL_USB_CONFIGDATA_UTMIDATAWIDTH_RESETVAL (0x0000u)
N
N#define CSL_USB_CONFIGDATA_RESETVAL (0x0000u)
N
N/* FIFOSIZE */
N
N
N#define CSL_USB_FIFOSIZE_RXFIFOSZ_MASK (0x00F0u)
N#define CSL_USB_FIFOSIZE_RXFIFOSZ_SHIFT (0x0004u)
N#define CSL_USB_FIFOSIZE_RXFIFOSZ_RESETVAL (0x0000u)
N
N#define CSL_USB_FIFOSIZE_TXFIFOSZ_MASK (0x000Fu)
N#define CSL_USB_FIFOSIZE_TXFIFOSZ_SHIFT (0x0000u)
N#define CSL_USB_FIFOSIZE_TXFIFOSZ_RESETVAL (0x0000u)
N
N#define CSL_USB_FIFOSIZE_RESETVAL (0x0000u)
N
N/* FIFO0L */
N
N#define CSL_USB_FIFO0L_DATA_MASK (0xFFFFu)
N#define CSL_USB_FIFO0L_DATA_SHIFT (0x0000u)
N#define CSL_USB_FIFO0L_DATA_RESETVAL (0x0000u)
N
N#define CSL_USB_FIFO0L_RESETVAL (0x0000u)
N
N/* FIFO0H */
N
N#define CSL_USB_FIFO0H_DATA_MASK (0xFFFFu)
N#define CSL_USB_FIFO0H_DATA_SHIFT (0x0000u)
N#define CSL_USB_FIFO0H_DATA_RESETVAL (0x0000u)
N
N#define CSL_USB_FIFO0H_RESETVAL (0x0000u)
N
N/* FIFO1L */
N
N#define CSL_USB_FIFO1L_DATA_MASK (0xFFFFu)
N#define CSL_USB_FIFO1L_DATA_SHIFT (0x0000u)
N#define CSL_USB_FIFO1L_DATA_RESETVAL (0x0000u)
N
N#define CSL_USB_FIFO1L_RESETVAL (0x0000u)
N
N/* FIFO1H */
N
N#define CSL_USB_FIFO1H_DATA_MASK (0xFFFFu)
N#define CSL_USB_FIFO1H_DATA_SHIFT (0x0000u)
N#define CSL_USB_FIFO1H_DATA_RESETVAL (0x0000u)
N
N#define CSL_USB_FIFO1H_RESETVAL (0x0000u)
N
N/* FIFO2L */
N
N#define CSL_USB_FIFO2L_DATA_MASK (0xFFFFu)
N#define CSL_USB_FIFO2L_DATA_SHIFT (0x0000u)
N#define CSL_USB_FIFO2L_DATA_RESETVAL (0x0000u)
N
N#define CSL_USB_FIFO2L_RESETVAL (0x0000u)
N
N/* FIFO2H */
N
N#define CSL_USB_FIFO2H_DATA_MASK (0xFFFFu)
N#define CSL_USB_FIFO2H_DATA_SHIFT (0x0000u)
N#define CSL_USB_FIFO2H_DATA_RESETVAL (0x0000u)
N
N#define CSL_USB_FIFO2H_RESETVAL (0x0000u)
N
N/* FIFO3L */
N
N#define CSL_USB_FIFO3L_DATA_MASK (0xFFFFu)
N#define CSL_USB_FIFO3L_DATA_SHIFT (0x0000u)
N#define CSL_USB_FIFO3L_DATA_RESETVAL (0x0000u)
N
N#define CSL_USB_FIFO3L_RESETVAL (0x0000u)
N
N/* FIFO3H */
N
N#define CSL_USB_FIFO3H_DATA_MASK (0xFFFFu)
N#define CSL_USB_FIFO3H_DATA_SHIFT (0x0000u)
N#define CSL_USB_FIFO3H_DATA_RESETVAL (0x0000u)
N
N#define CSL_USB_FIFO3H_RESETVAL (0x0000u)
N
N/* FIFO4L */
N
N#define CSL_USB_FIFO4L_DATA_MASK (0xFFFFu)
N#define CSL_USB_FIFO4L_DATA_SHIFT (0x0000u)
N#define CSL_USB_FIFO4L_DATA_RESETVAL (0x0000u)
N
N#define CSL_USB_FIFO4L_RESETVAL (0x0000u)
N
N/* FIFO4H */
N
N#define CSL_USB_FIFO4H_DATA_MASK (0xFFFFu)
N#define CSL_USB_FIFO4H_DATA_SHIFT (0x0000u)
N#define CSL_USB_FIFO4H_DATA_RESETVAL (0x0000u)
N
N#define CSL_USB_FIFO4H_RESETVAL (0x0000u)
N
N/* DEVCTL */
N
N
N#define CSL_USB_DEVCTL_BDEVICE_MASK (0x0080u)
N#define CSL_USB_DEVCTL_BDEVICE_SHIFT (0x0007u)
N#define CSL_USB_DEVCTL_BDEVICE_RESETVAL (0x0001u)
N
N#define CSL_USB_DEVCTL_FSDEV_MASK (0x0040u)
N#define CSL_USB_DEVCTL_FSDEV_SHIFT (0x0006u)
N#define CSL_USB_DEVCTL_FSDEV_RESETVAL (0x0000u)
N
N#define CSL_USB_DEVCTL_LSDEV_MASK (0x0020u)
N#define CSL_USB_DEVCTL_LSDEV_SHIFT (0x0005u)
N#define CSL_USB_DEVCTL_LSDEV_RESETVAL (0x0000u)
N
N#define CSL_USB_DEVCTL_VBUS_MASK (0x0018u)
N#define CSL_USB_DEVCTL_VBUS_SHIFT (0x0003u)
N#define CSL_USB_DEVCTL_VBUS_RESETVAL (0x0000u)
N
N#define CSL_USB_DEVCTL_HOSTMODE_MASK (0x0004u)
N#define CSL_USB_DEVCTL_HOSTMODE_SHIFT (0x0002u)
N#define CSL_USB_DEVCTL_HOSTMODE_RESETVAL (0x0000u)
N
N#define CSL_USB_DEVCTL_HOSTREQ_MASK (0x0002u)
N#define CSL_USB_DEVCTL_HOSTREQ_SHIFT (0x0001u)
N#define CSL_USB_DEVCTL_HOSTREQ_RESETVAL (0x0000u)
N
N#define CSL_USB_DEVCTL_SESSION_MASK (0x0001u)
N#define CSL_USB_DEVCTL_SESSION_SHIFT (0x0000u)
N#define CSL_USB_DEVCTL_SESSION_RESETVAL (0x0000u)
N
N#define CSL_USB_DEVCTL_RESETVAL (0x0080u)
N
N/* TXRXFIFOSZ */
N
N
N
N
N#define CSL_USB_TXRXFIFOSZ_DPB_MASK (0x0010u)
N#define CSL_USB_TXRXFIFOSZ_DPB_SHIFT (0x0004u)
N#define CSL_USB_TXRXFIFOSZ_DPB_RESETVAL (0x0000u)
N
N#define CSL_USB_TXRXFIFOSZ_DPB_MASK (0x0010u)
N#define CSL_USB_TXRXFIFOSZ_DPB_SHIFT (0x0004u)
N#define CSL_USB_TXRXFIFOSZ_DPB_RESETVAL (0x0000u)
N
N#define CSL_USB_TXRXFIFOSZ_SZ_MASK (0x000Fu)
N#define CSL_USB_TXRXFIFOSZ_SZ_SHIFT (0x0000u)
N#define CSL_USB_TXRXFIFOSZ_SZ_RESETVAL (0x0000u)
N
N#define CSL_USB_TXRXFIFOSZ_SZ_MASK (0x000Fu)
N#define CSL_USB_TXRXFIFOSZ_SZ_SHIFT (0x0000u)
N#define CSL_USB_TXRXFIFOSZ_SZ_RESETVAL (0x0000u)
N
N#define CSL_USB_TXRXFIFOSZ_RESETVAL (0x0000u)
N
N/* TXFIFOADDR */
N
N
N#define CSL_USB_TXFIFOADDR_ADDR_MASK (0x1FFFu)
N#define CSL_USB_TXFIFOADDR_ADDR_SHIFT (0x0000u)
N#define CSL_USB_TXFIFOADDR_ADDR_RESETVAL (0x0000u)
N
N#define CSL_USB_TXFIFOADDR_RESETVAL (0x0000u)
N
N/* RXFIFOADDR */
N
N
N#define CSL_USB_RXFIFOADDR_ADDR_MASK (0x1FFFu)
N#define CSL_USB_RXFIFOADDR_ADDR_SHIFT (0x0000u)
N#define CSL_USB_RXFIFOADDR_ADDR_RESETVAL (0x0000u)
N
N#define CSL_USB_RXFIFOADDR_RESETVAL (0x0000u)
N
N/* SCHECTRLL */
N
N
N#define CSL_USB_SCHECTRLL_LASTENTRY_MASK (0x00FFu)
N#define CSL_USB_SCHECTRLL_LASTENTRY_SHIFT (0x0000u)
N#define CSL_USB_SCHECTRLL_LASTENTRY_RESETVAL (0x0000u)
N
N#define CSL_USB_SCHECTRLL_RESETVAL (0x0000u)
N
N/* SCHECTRLH */
N
N#define CSL_USB_SCHECTRLH_ENABLE_MASK (0x8000u)
N#define CSL_USB_SCHECTRLH_ENABLE_SHIFT (0x000Fu)
N#define CSL_USB_SCHECTRLH_ENABLE_RESETVAL (0x0000u)
N
N
N#define CSL_USB_SCHECTRLH_RESETVAL (0x0000u)
N
N/* INTDRL */
N
N#define CSL_USB_INTDRL_REVRTL_MASK (0xF800u)
N#define CSL_USB_INTDRL_REVRTL_SHIFT (0x000Bu)
N#define CSL_USB_INTDRL_REVRTL_RESETVAL (0x000Du)
N
N#define CSL_USB_INTDRL_REVMAJ_MASK (0x0700u)
N#define CSL_USB_INTDRL_REVMAJ_SHIFT (0x0008u)
N#define CSL_USB_INTDRL_REVMAJ_RESETVAL (0x0001u)
N
N#define CSL_USB_INTDRL_REVMIN_MASK (0x01FFu)
N#define CSL_USB_INTDRL_REVMIN_SHIFT (0x0000u)
N#define CSL_USB_INTDRL_REVMIN_RESETVAL (0x0000u)
N
N#define CSL_USB_INTDRL_RESETVAL (0x6900u)
N
N/* INTDRH */
N
N#define CSL_USB_INTDRH_SCHEME_MASK (0xC000u)
N#define CSL_USB_INTDRH_SCHEME_SHIFT (0x000Eu)
N#define CSL_USB_INTDRH_SCHEME_RESETVAL (0x0001u)
N
N#define CSL_USB_INTDRH_MODID_MASK (0x3FFFu)
N#define CSL_USB_INTDRH_MODID_SHIFT (0x0000u)
N#define CSL_USB_INTDRH_MODID_RESETVAL (0x0E83u)
N
N#define CSL_USB_INTDRH_RESETVAL (0x4E83u)
N
N/* INTDEOIR */
N
N
N#define CSL_USB_INTDEOIR_EOI_VECTOR_MASK (0x00FFu)
N#define CSL_USB_INTDEOIR_EOI_VECTOR_SHIFT (0x0000u)
N#define CSL_USB_INTDEOIR_EOI_VECTOR_RESETVAL (0x0000u)
N
N#define CSL_USB_INTDEOIR_RESETVAL (0x0000u)
N
N/* INTDSTATUSR0 */
N
N
N
N#define CSL_USB_INTDSTATUSR0_RX_SBUF_MASK (0x0001u)
N#define CSL_USB_INTDSTATUSR0_RX_SBUF_SHIFT (0x0000u)
N#define CSL_USB_INTDSTATUSR0_RX_SBUF_RESETVAL (0x0000u)
N
N#define CSL_USB_INTDSTATUSR0_RX_SDES_MASK (0x0001u)
N#define CSL_USB_INTDSTATUSR0_RX_SDES_SHIFT (0x0000u)
N#define CSL_USB_INTDSTATUSR0_RX_SDES_RESETVAL (0x0000u)
N
N#define CSL_USB_INTDSTATUSR0_RESETVAL (0x0000u)
N
N/* INTDSTATUSR1L */
N
N#define CSL_USB_INTDSTATUSR1L_RX_EP1_MASK (0x8000u)
N#define CSL_USB_INTDSTATUSR1L_RX_EP1_SHIFT (0x000Fu)
N#define CSL_USB_INTDSTATUSR1L_RX_EP1_RESETVAL (0x0000u)
N
N#define CSL_USB_INTDSTATUSR1L_RX_EP0_MASK (0x4000u)
N#define CSL_USB_INTDSTATUSR1L_RX_EP0_SHIFT (0x000Eu)
N#define CSL_USB_INTDSTATUSR1L_RX_EP0_RESETVAL (0x0000u)
N
N#define CSL_USB_INTDSTATUSR1L_TX_EP4_MASK (0x2000u)
N#define CSL_USB_INTDSTATUSR1L_TX_EP4_SHIFT (0x000Du)
N#define CSL_USB_INTDSTATUSR1L_TX_EP4_RESETVAL (0x0000u)
N
N#define CSL_USB_INTDSTATUSR1L_TX_EP3_MASK (0x1000u)
N#define CSL_USB_INTDSTATUSR1L_TX_EP3_SHIFT (0x000Cu)
N#define CSL_USB_INTDSTATUSR1L_TX_EP3_RESETVAL (0x0000u)
N
N#define CSL_USB_INTDSTATUSR1L_TX_EP2_MASK (0x0800u)
N#define CSL_USB_INTDSTATUSR1L_TX_EP2_SHIFT (0x000Bu)
N#define CSL_USB_INTDSTATUSR1L_TX_EP2_RESETVAL (0x0000u)
N
N#define CSL_USB_INTDSTATUSR1L_TX_EP1_MASK (0x0400u)
N#define CSL_USB_INTDSTATUSR1L_TX_EP1_SHIFT (0x000Au)
N#define CSL_USB_INTDSTATUSR1L_TX_EP1_RESETVAL (0x0000u)
N
N#define CSL_USB_INTDSTATUSR1L_TX_EP0_MASK (0x0200u)
N#define CSL_USB_INTDSTATUSR1L_TX_EP0_SHIFT (0x0009u)
N#define CSL_USB_INTDSTATUSR1L_TX_EP0_RESETVAL (0x0000u)
N
N#define CSL_USB_INTDSTATUSR1L_USB_INT8_MASK (0x0100u)
N#define CSL_USB_INTDSTATUSR1L_USB_INT8_SHIFT (0x0008u)
N#define CSL_USB_INTDSTATUSR1L_USB_INT8_RESETVAL (0x0000u)
N
N#define CSL_USB_INTDSTATUSR1L_USB_INT7_MASK (0x0080u)
N#define CSL_USB_INTDSTATUSR1L_USB_INT7_SHIFT (0x0007u)
N#define CSL_USB_INTDSTATUSR1L_USB_INT7_RESETVAL (0x0000u)
N
N#define CSL_USB_INTDSTATUSR1L_USB_INT6_MASK (0x0040u)
N#define CSL_USB_INTDSTATUSR1L_USB_INT6_SHIFT (0x0006u)
N#define CSL_USB_INTDSTATUSR1L_USB_INT6_RESETVAL (0x0000u)
N
N#define CSL_USB_INTDSTATUSR1L_USB_INT5_MASK (0x0020u)
N#define CSL_USB_INTDSTATUSR1L_USB_INT5_SHIFT (0x0005u)
N#define CSL_USB_INTDSTATUSR1L_USB_INT5_RESETVAL (0x0000u)
N
N#define CSL_USB_INTDSTATUSR1L_USB_INT4_MASK (0x0010u)
N#define CSL_USB_INTDSTATUSR1L_USB_INT4_SHIFT (0x0004u)
N#define CSL_USB_INTDSTATUSR1L_USB_INT4_RESETVAL (0x0000u)
N
N#define CSL_USB_INTDSTATUSR1L_USB_INT3_MASK (0x0008u)
N#define CSL_USB_INTDSTATUSR1L_USB_INT3_SHIFT (0x0003u)
N#define CSL_USB_INTDSTATUSR1L_USB_INT3_RESETVAL (0x0000u)
N
N#define CSL_USB_INTDSTATUSR1L_USB_INT2_MASK (0x0004u)
N#define CSL_USB_INTDSTATUSR1L_USB_INT2_SHIFT (0x0002u)
N#define CSL_USB_INTDSTATUSR1L_USB_INT2_RESETVAL (0x0000u)
N
N#define CSL_USB_INTDSTATUSR1L_USB_INT1_MASK (0x0002u)
N#define CSL_USB_INTDSTATUSR1L_USB_INT1_SHIFT (0x0001u)
N#define CSL_USB_INTDSTATUSR1L_USB_INT1_RESETVAL (0x0000u)
N
N#define CSL_USB_INTDSTATUSR1L_USB_INT0_MASK (0x0001u)
N#define CSL_USB_INTDSTATUSR1L_USB_INT0_SHIFT (0x0000u)
N#define CSL_USB_INTDSTATUSR1L_USB_INT0_RESETVAL (0x0000u)
N
N#define CSL_USB_INTDSTATUSR1L_RESETVAL (0x0000u)
N
N/* INTDSTATUSR1H */
N
N
N#define CSL_USB_INTDSTATUSR1H_USB_MASK (0x0008u)
N#define CSL_USB_INTDSTATUSR1H_USB_SHIFT (0x0003u)
N#define CSL_USB_INTDSTATUSR1H_USB_RESETVAL (0x0000u)
N
N#define CSL_USB_INTDSTATUSR1H_RX_EP4_MASK (0x0004u)
N#define CSL_USB_INTDSTATUSR1H_RX_EP4_SHIFT (0x0002u)
N#define CSL_USB_INTDSTATUSR1H_RX_EP4_RESETVAL (0x0000u)
N
N#define CSL_USB_INTDSTATUSR1H_RX_EP3_MASK (0x0002u)
N#define CSL_USB_INTDSTATUSR1H_RX_EP3_SHIFT (0x0001u)
N#define CSL_USB_INTDSTATUSR1H_RX_EP3_RESETVAL (0x0000u)
N
N#define CSL_USB_INTDSTATUSR1H_RX_EP2_MASK (0x0001u)
N#define CSL_USB_INTDSTATUSR1H_RX_EP2_SHIFT (0x0000u)
N#define CSL_USB_INTDSTATUSR1H_RX_EP2_RESETVAL (0x0000u)
N
N#define CSL_USB_INTDSTATUSR1H_RESETVAL (0x0000u)
N
N/* QMRRL */
N
N#define CSL_USB_QMRRL_REVRTL_MASK (0xF800u)
N#define CSL_USB_QMRRL_REVRTL_SHIFT (0x000Bu)
N#define CSL_USB_QMRRL_REVRTL_RESETVAL (0x0000u)
N
N#define CSL_USB_QMRRL_REVMAJ_MASK (0x0700u)
N#define CSL_USB_QMRRL_REVMAJ_SHIFT (0x0008u)
N#define CSL_USB_QMRRL_REVMAJ_RESETVAL (0x0000u)
N
N#define CSL_USB_QMRRL_REVCUSTOM_MASK (0x00C0u)
N#define CSL_USB_QMRRL_REVCUSTOM_SHIFT (0x0006u)
N#define CSL_USB_QMRRL_REVCUSTOM_RESETVAL (0x0000u)
N
N#define CSL_USB_QMRRL_REVMIN_MASK (0x003Fu)
N#define CSL_USB_QMRRL_REVMIN_SHIFT (0x0000u)
N#define CSL_USB_QMRRL_REVMIN_RESETVAL (0x0000u)
N
N#define CSL_USB_QMRRL_RESETVAL (0x0000u)
N
N/* QMRRH */
N
N#define CSL_USB_QMRRH_SCHEME_MASK (0xC000u)
N#define CSL_USB_QMRRH_SCHEME_SHIFT (0x000Eu)
N#define CSL_USB_QMRRH_SCHEME_RESETVAL (0x0000u)
N
N#define CSL_USB_QMRRH_FUNCTION_MASK (0x3FFFu)
N#define CSL_USB_QMRRH_FUNCTION_SHIFT (0x0000u)
N#define CSL_USB_QMRRH_FUNCTION_RESETVAL (0x0000u)
N
N#define CSL_USB_QMRRH_RESETVAL (0x0000u)
N
N/* QMQRRL */
N
N
N
N#define CSL_USB_QMQRRL_SRC_QNUM_MASK (0x003Fu)
N#define CSL_USB_QMQRRL_SRC_QNUM_SHIFT (0x0000u)
N#define CSL_USB_QMQRRL_SRC_QNUM_RESETVAL (0x0000u)
N
N#define CSL_USB_QMQRRL_RESETVAL (0x0000u)
N
N/* QMQRRH */
N
N#define CSL_USB_QMQRRH_HEAD_TAIL_MASK (0x8000u)
N#define CSL_USB_QMQRRH_HEAD_TAIL_SHIFT (0x000Fu)
N#define CSL_USB_QMQRRH_HEAD_TAIL_RESETVAL (0x0000u)
N
N
N
N#define CSL_USB_QMQRRH_DST_QNUM_MASK (0x003Fu)
N#define CSL_USB_QMQRRH_DST_QNUM_SHIFT (0x0000u)
N#define CSL_USB_QMQRRH_DST_QNUM_RESETVAL (0x0000u)
N
N#define CSL_USB_QMQRRH_RESETVAL (0x0000u)
N
N/* QMFDBSCR0L */
N
N#define CSL_USB_QMFDBSCR0L_FDBQ1_STARVE_CNT_MASK (0xFF00u)
N#define CSL_USB_QMFDBSCR0L_FDBQ1_STARVE_CNT_SHIFT (0x0008u)
N#define CSL_USB_QMFDBSCR0L_FDBQ1_STARVE_CNT_RESETVAL (0x0000u)
N
N#define CSL_USB_QMFDBSCR0L_FDBQ0_STARVE_CNT_MASK (0x00FFu)
N#define CSL_USB_QMFDBSCR0L_FDBQ0_STARVE_CNT_SHIFT (0x0000u)
N#define CSL_USB_QMFDBSCR0L_FDBQ0_STARVE_CNT_RESETVAL (0x0000u)
N
N#define CSL_USB_QMFDBSCR0L_RESETVAL (0x0000u)
N
N/* QMFDBSCR0H */
N
N#define CSL_USB_QMFDBSCR0H_FDBQ3_STARVE_CNT_MASK (0xFF00u)
N#define CSL_USB_QMFDBSCR0H_FDBQ3_STARVE_CNT_SHIFT (0x0008u)
N#define CSL_USB_QMFDBSCR0H_FDBQ3_STARVE_CNT_RESETVAL (0x0000u)
N
N#define CSL_USB_QMFDBSCR0H_FDBQ2_STARVE_CNT_MASK (0x00FFu)
N#define CSL_USB_QMFDBSCR0H_FDBQ2_STARVE_CNT_SHIFT (0x0000u)
N#define CSL_USB_QMFDBSCR0H_FDBQ2_STARVE_CNT_RESETVAL (0x0000u)
N
N#define CSL_USB_QMFDBSCR0H_RESETVAL (0x0000u)
N
N/* QMFDBSCR1L */
N
N#define CSL_USB_QMFDBSCR1L_FDBQ5_STARVE_CNT_MASK (0xFF00u)
N#define CSL_USB_QMFDBSCR1L_FDBQ5_STARVE_CNT_SHIFT (0x0008u)
N#define CSL_USB_QMFDBSCR1L_FDBQ5_STARVE_CNT_RESETVAL (0x0000u)
N
N#define CSL_USB_QMFDBSCR1L_FDBQ4_STARVE_CNT_MASK (0x00FFu)
N#define CSL_USB_QMFDBSCR1L_FDBQ4_STARVE_CNT_SHIFT (0x0000u)
N#define CSL_USB_QMFDBSCR1L_FDBQ4_STARVE_CNT_RESETVAL (0x0000u)
N
N#define CSL_USB_QMFDBSCR1L_RESETVAL (0x0000u)
N
N/* QMFDBSCR1H */
N
N#define CSL_USB_QMFDBSCR1H_FDBQ7_STARVE_CNT_MASK (0xFF00u)
N#define CSL_USB_QMFDBSCR1H_FDBQ7_STARVE_CNT_SHIFT (0x0008u)
N#define CSL_USB_QMFDBSCR1H_FDBQ7_STARVE_CNT_RESETVAL (0x0000u)
N
N#define CSL_USB_QMFDBSCR1H_FDBQ6_STARVE_CNT_MASK (0x00FFu)
N#define CSL_USB_QMFDBSCR1H_FDBQ6_STARVE_CNT_SHIFT (0x0000u)
N#define CSL_USB_QMFDBSCR1H_FDBQ6_STARVE_CNT_RESETVAL (0x0000u)
N
N#define CSL_USB_QMFDBSCR1H_RESETVAL (0x0000u)
N
N/* QMFDBSCR2L */
N
N#define CSL_USB_QMFDBSCR2L_FDBQ9_STARVE_CNT_MASK (0xFF00u)
N#define CSL_USB_QMFDBSCR2L_FDBQ9_STARVE_CNT_SHIFT (0x0008u)
N#define CSL_USB_QMFDBSCR2L_FDBQ9_STARVE_CNT_RESETVAL (0x0000u)
N
N#define CSL_USB_QMFDBSCR2L_FDBQ8_STARVE_CNT_MASK (0x00FFu)
N#define CSL_USB_QMFDBSCR2L_FDBQ8_STARVE_CNT_SHIFT (0x0000u)
N#define CSL_USB_QMFDBSCR2L_FDBQ8_STARVE_CNT_RESETVAL (0x0000u)
N
N#define CSL_USB_QMFDBSCR2L_RESETVAL (0x0000u)
N
N/* QMFDBSCR2H */
N
N#define CSL_USB_QMFDBSCR2H_FDBQ11_STARVE_CNT_MASK (0xFF00u)
N#define CSL_USB_QMFDBSCR2H_FDBQ11_STARVE_CNT_SHIFT (0x0008u)
N#define CSL_USB_QMFDBSCR2H_FDBQ11_STARVE_CNT_RESETVAL (0x0000u)
N
N#define CSL_USB_QMFDBSCR2H_FDBQ10_STARVE_CNT_MASK (0x00FFu)
N#define CSL_USB_QMFDBSCR2H_FDBQ10_STARVE_CNT_SHIFT (0x0000u)
N#define CSL_USB_QMFDBSCR2H_FDBQ10_STARVE_CNT_RESETVAL (0x0000u)
N
N#define CSL_USB_QMFDBSCR2H_RESETVAL (0x0000u)
N
N/* QMFDBSCR3L */
N
N#define CSL_USB_QMFDBSCR3L_FDBQ13_STARVE_CNT_MASK (0xFF00u)
N#define CSL_USB_QMFDBSCR3L_FDBQ13_STARVE_CNT_SHIFT (0x0008u)
N#define CSL_USB_QMFDBSCR3L_FDBQ13_STARVE_CNT_RESETVAL (0x0000u)
N
N#define CSL_USB_QMFDBSCR3L_FDBQ12_STARVE_CNT_MASK (0x00FFu)
N#define CSL_USB_QMFDBSCR3L_FDBQ12_STARVE_CNT_SHIFT (0x0000u)
N#define CSL_USB_QMFDBSCR3L_FDBQ12_STARVE_CNT_RESETVAL (0x0000u)
N
N#define CSL_USB_QMFDBSCR3L_RESETVAL (0x0000u)
N
N/* QMFDBSCR3H */
N
N#define CSL_USB_QMFDBSCR3H_FDBQ15_STARVE_CNT_MASK (0xFF00u)
N#define CSL_USB_QMFDBSCR3H_FDBQ15_STARVE_CNT_SHIFT (0x0008u)
N#define CSL_USB_QMFDBSCR3H_FDBQ15_STARVE_CNT_RESETVAL (0x0000u)
N
N#define CSL_USB_QMFDBSCR3H_FDBQ14_STARVE_CNT_MASK (0x00FFu)
N#define CSL_USB_QMFDBSCR3H_FDBQ14_STARVE_CNT_SHIFT (0x0000u)
N#define CSL_USB_QMFDBSCR3H_FDBQ14_STARVE_CNT_RESETVAL (0x0000u)
N
N#define CSL_USB_QMFDBSCR3H_RESETVAL (0x0000u)
N
N/* QMLRR0BARL */
N
N#define CSL_USB_QMLRR0BARL_BASE_MASK (0xFFFFu)
N#define CSL_USB_QMLRR0BARL_BASE_SHIFT (0x0000u)
N#define CSL_USB_QMLRR0BARL_BASE_RESETVAL (0x0000u)
N
N#define CSL_USB_QMLRR0BARL_RESETVAL (0x0000u)
N
N/* QMLRR0BARH */
N
N#define CSL_USB_QMLRR0BARH_BASE_MASK (0xFFFFu)
N#define CSL_USB_QMLRR0BARH_BASE_SHIFT (0x0000u)
N#define CSL_USB_QMLRR0BARH_BASE_RESETVAL (0x0000u)
N
N#define CSL_USB_QMLRR0BARH_RESETVAL (0x0000u)
N
N/* QMLRR0SRL */
N
N
N#define CSL_USB_QMLRR0SRL_SIZE_MASK (0x3FFFu)
N#define CSL_USB_QMLRR0SRL_SIZE_SHIFT (0x0000u)
N#define CSL_USB_QMLRR0SRL_SIZE_RESETVAL (0x0000u)
N
N#define CSL_USB_QMLRR0SRL_RESETVAL (0x0000u)
N
N/* QMLRR0SRH */
N
N
N#define CSL_USB_QMLRR0SRH_RESETVAL (0x0000u)
N
N/* QMLRR1BARL */
N
N#define CSL_USB_QMLRR1BARL_BASE_MASK (0xFFFFu)
N#define CSL_USB_QMLRR1BARL_BASE_SHIFT (0x0000u)
N#define CSL_USB_QMLRR1BARL_BASE_RESETVAL (0x0000u)
N
N#define CSL_USB_QMLRR1BARL_RESETVAL (0x0000u)
N
N/* QMLRR1BARH */
N
N#define CSL_USB_QMLRR1BARH_BASE_MASK (0xFFFFu)
N#define CSL_USB_QMLRR1BARH_BASE_SHIFT (0x0000u)
N#define CSL_USB_QMLRR1BARH_BASE_RESETVAL (0x0000u)
N
N#define CSL_USB_QMLRR1BARH_RESETVAL (0x0000u)
N
N/* QMQPR0L */
N
N#define CSL_USB_QMQPR0L_QPEND_MASK (0xFFFFu)
N#define CSL_USB_QMQPR0L_QPEND_SHIFT (0x0000u)
N#define CSL_USB_QMQPR0L_QPEND_RESETVAL (0x0000u)
N
N#define CSL_USB_QMQPR0L_RESETVAL (0x0000u)
N
N/* QMQPR0H */
N
N#define CSL_USB_QMQPR0H_QPEND_MASK (0xFFFFu)
N#define CSL_USB_QMQPR0H_QPEND_SHIFT (0x0000u)
N#define CSL_USB_QMQPR0H_QPEND_RESETVAL (0x0000u)
N
N#define CSL_USB_QMQPR0H_RESETVAL (0x0000u)
N
N/* QMQPR1L */
N
N#define CSL_USB_QMQPR1L_QPEND_MASK (0xFFFFu)
N#define CSL_USB_QMQPR1L_QPEND_SHIFT (0x0000u)
N#define CSL_USB_QMQPR1L_QPEND_RESETVAL (0x0000u)
N
N#define CSL_USB_QMQPR1L_RESETVAL (0x0000u)
N
N/* QMQPR1H */
N
N#define CSL_USB_QMQPR1H_QPEND_MASK (0xFFFFu)
N#define CSL_USB_QMQPR1H_QPEND_SHIFT (0x0000u)
N#define CSL_USB_QMQPR1H_QPEND_RESETVAL (0x0000u)
N
N#define CSL_USB_QMQPR1H_RESETVAL (0x0000u)
N
N#endif
L 68 "../common_inc/corazon.h" 2
N#include "cslr_pll_001.h"
L 1 "..\common_inc\cslr_pll_001.h" 1
N/*****************************************************************************
N * File Name : cslr_pll_001.h 
N *
N * Brief	 : Define PLL register structure
N *
N * Copyright (C) 2009 -2010 Texas Instruments Incorporated - http://www.ti.com/ 
N * 
N * 
N *  Redistribution and use in source and binary forms, with or without 
N *  modification, are permitted provided that the following conditions 
N *  are met:
N *
N *    Redistributions of source code must retain the above copyright 
N *    notice, this list of conditions and the following disclaimer.
N *
N *    Redistributions in binary form must reproduce the above copyright
N *    notice, this list of conditions and the following disclaimer in the 
N *    documentation and/or other materials provided with the   
N *    distribution.
N *
N *    Neither the name of Texas Instruments Incorporated nor the names of
N *    its contributors may be used to endorse or promote products derived
N *    from this software without specific prior written permission.
N *
N *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
N *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
N *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
N *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
N *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
N *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
N *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
N *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
N *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
N *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
N *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
N *
N******************************************************************************/
N#ifndef _CSLR__PLL_1_H_
N#define _CSLR__PLL_1_H_
N
N#include <cslr.h>
N
N
N/* Minimum unit = 2 bytes */
N
N/**************************************************************************\
N* Register Overlay Structure
X
N\**************************************************************************/
Ntypedef struct  {
N    volatile Uint16 PLLCNTL1;               //1
N    volatile Uint16 PLLINCNTL;              //2
N    volatile Uint16 PLLCNTL2;               //3
N    volatile Uint16 PLLOUTCNTL;             //4
N    volatile Uint16 CLK_OUT; 
N} CSL_PllRegs;
N
N/**************************************************************************\
N* Field Definition Macros
X
N\**************************************************************************/
N
N/* PLLCNTL1 */
N
N#define CSL_PLL_PLLCNTL1_CLR_CNTL_MASK (0x8000u)
N#define CSL_PLL_PLLCNTL1_CLR_CNTL_SHIFT (0x000Fu)
N#define CSL_PLL_PLLCNTL1_CLR_CNTL_RESETVAL (0x0001u)
N
N
N#define CSL_PLL_PLLCNTL1_PLL_STANDBY_MASK (0x2000u)
N#define CSL_PLL_PLLCNTL1_PLL_STANDBY_SHIFT (0x000Du)
N#define CSL_PLL_PLLCNTL1_PLL_STANDBY_RESETVAL (0x0001u)
N
N#define CSL_PLL_PLLCNTL1_PLL_PWRDN_MASK (0x1000u)
N#define CSL_PLL_PLLCNTL1_PLL_PWRDN_SHIFT (0x000Cu)
N#define CSL_PLL_PLLCNTL1_PLL_PWRDN_RESETVAL (0x0000u)
N
N
N#define CSL_PLL_PLLCNTL1_VP_MASK (0x03FFu)
N#define CSL_PLL_PLLCNTL1_VP_SHIFT (0x0000u)
N#define CSL_PLL_PLLCNTL1_VP_RESETVAL (0x0000u)
N
N#define CSL_PLL_PLLCNTL1_RESETVAL (0xA000u)
N
N/* PLLINCNTL */
N
N#define CSL_PLL_PLLINCNTL_RP_BYPASS_MASK (0x8000u)
N#define CSL_PLL_PLLINCNTL_RP_BYPASS_SHIFT (0x000Fu)
N#define CSL_PLL_PLLINCNTL_RP_BYPASS_RESETVAL (0x0000u)
N
N
N#define CSL_PLL_PLLINCNTL_VS_MASK (0x3000u)
N#define CSL_PLL_PLLINCNTL_VS_SHIFT (0x000Cu)
N#define CSL_PLL_PLLINCNTL_VS_RESETVAL (0x0000u)
N
N#define CSL_PLL_PLLINCNTL_RD_MASK (0x0FFFu)
N#define CSL_PLL_PLLINCNTL_RD_SHIFT (0x0000u)
N#define CSL_PLL_PLLINCNTL_RD_RESETVAL (0x0000u)
N
N#define CSL_PLL_PLLINCNTL_RESETVAL (0x0000u)
N
N/* PLLCNTL2 */
N
N#define CSL_PLL_PLLCNTL2_PLL_DIS_MASK (0x8000u)
N#define CSL_PLL_PLLCNTL2_PLL_DIS_SHIFT (0x000Fu)
N#define CSL_PLL_PLLCNTL2_PLL_DIS_RESETVAL (0x0000u)
N
N#define CSL_PLL_PLLCNTL2_CLK_DIS_MASK (0x4000u)
N#define CSL_PLL_PLLCNTL2_CLK_DIS_SHIFT (0x000Eu)
N#define CSL_PLL_PLLCNTL2_CLK_DIS_RESETVAL (0x0000u)
N
N#define CSL_PLL_PLLCNTL2_EN_VCO_DIV_MASK (0x2000u)
N#define CSL_PLL_PLLCNTL2_EN_VCO_DIV_SHIFT (0x000Du)
N#define CSL_PLL_PLLCNTL2_EN_VCO_DIV_RESETVAL (0x0000u)
N
N#define CSL_PLL_PLLCNTL2_ENTP_SIG_MASK (0x1000u)
N#define CSL_PLL_PLLCNTL2_ENTP_SIG_SHIFT (0x000Cu)
N#define CSL_PLL_PLLCNTL2_ENTP_SIG_RESETVAL (0x0000u)
N
N#define CSL_PLL_PLLCNTL2_EN_LW_JITTER_MASK (0x0800u)
N#define CSL_PLL_PLLCNTL2_EN_LW_JITTER_SHIFT (0x000Bu)
N#define CSL_PLL_PLLCNTL2_EN_LW_JITTER_RESETVAL (0x0001u)
N
N
N#define CSL_PLL_PLLCNTL2_PDSW_CNTL_MASK (0x0200u)
N#define CSL_PLL_PLLCNTL2_PDSW_CNTL_SHIFT (0x0009u)
N#define CSL_PLL_PLLCNTL2_PDSW_CNTL_RESETVAL (0x0000u)
N
N#define CSL_PLL_PLLCNTL2_PDSW_TOGDIS_MASK (0x0100u)
N#define CSL_PLL_PLLCNTL2_PDSW_TOGDIS_SHIFT (0x0008u)
N#define CSL_PLL_PLLCNTL2_PDSW_TOGDIS_RESETVAL (0x0000u)
N
N
N#define CSL_PLL_PLLCNTL2_LP_MASK (0x0020u)
N#define CSL_PLL_PLLCNTL2_LP_SHIFT (0x0005u)
N#define CSL_PLL_PLLCNTL2_LP_RESETVAL (0x0001u)
N
N#define CSL_PLL_PLLCNTL2_LW_BIAS_CURR_MASK (0x0010u)
N#define CSL_PLL_PLLCNTL2_LW_BIAS_CURR_SHIFT (0x0004u)
N#define CSL_PLL_PLLCNTL2_LW_BIAS_CURR_RESETVAL (0x0001u)
N
N#define CSL_PLL_PLLCNTL2_TST_LCK_MON_MASK (0x0008u)
N#define CSL_PLL_PLLCNTL2_TST_LCK_MON_SHIFT (0x0003u)
N#define CSL_PLL_PLLCNTL2_TST_LCK_MON_RESETVAL (0x0000u)
N
N#define CSL_PLL_PLLCNTL2_LNG_CLK_CNT_MASK (0x0004u)
N#define CSL_PLL_PLLCNTL2_LNG_CLK_CNT_SHIFT (0x0002u)
N#define CSL_PLL_PLLCNTL2_LNG_CLK_CNT_RESETVAL (0x0000u)
N
N#define CSL_PLL_PLLCNTL2_FSTRT_EN_MASK (0x0002u)
N#define CSL_PLL_PLLCNTL2_FSTRT_EN_SHIFT (0x0001u)
N#define CSL_PLL_PLLCNTL2_FSTRT_EN_RESETVAL (0x0001u)
N
N#define CSL_PLL_PLLCNTL2_NB_SEL_MASK (0x0001u)
N#define CSL_PLL_PLLCNTL2_NB_SEL_SHIFT (0x0000u)
N#define CSL_PLL_PLLCNTL2_NB_SEL_RESETVAL (0x0000u)
N
N#define CSL_PLL_PLLCNTL2_RESETVAL (0x0832u)
N
N/* PLLOUTCNTL */
N
N
N#define CSL_PLL_PLLOUTCNTL_OUT_DIV_EN_MASK (0x0200u)
N#define CSL_PLL_PLLOUTCNTL_OUT_DIV_EN_SHIFT (0x0009u)
N#define CSL_PLL_PLLOUTCNTL_OUT_DIV_EN_RESETVAL (0x0000u)
N
N#define CSL_PLL_PLLOUTCNTL_OUT_DIV2_BYP_MASK (0x0100u)
N#define CSL_PLL_PLLOUTCNTL_OUT_DIV2_BYP_SHIFT (0x0008u)
N#define CSL_PLL_PLLOUTCNTL_OUT_DIV2_BYP_RESETVAL (0x0000u)
N
N
N#define CSL_PLL_PLLOUTCNTL_OUT_DIV_MASK (0x003Fu)
N#define CSL_PLL_PLLOUTCNTL_OUT_DIV_SHIFT (0x0000u)
N#define CSL_PLL_PLLOUTCNTL_OUT_DIV_RESETVAL (0x0000u)
N
N#define CSL_PLL_PLLOUTCNTL_RESETVAL (0x0000u)
N
N/*CLK_OUT*/
N#define CSL_CLKOUT_CLKSRC_MASK (0x000fu)
N#define CSL_CLKOUT_CLKSRC_SHIFT (0x0000u)
N#define CSL_CLKOUT_CLKSRC_RESETVAL (0x0000u)
N
N#endif
L 69 "../common_inc/corazon.h" 2
N
N
N#define PLL_CNFIG(VP,VS) \
N	CSL_FINS(CSL_PLL_REGS->PLLCNTL2,CSL_PLL_PLLCNTL2_PLL_DIS,1);\
N	CSL_FINS(CSL_PLL_REGS->PLLCNTL1,CSL_PLL_PLLCNTL1_VP,VP);\
N	CSL_FINS(CSL_PLL_REGS->PLLINCNTL,CSL_PLL_PLLINCNTL_VS,VS);\
N	CSL_FINS(CSL_PLL_REGS->PLLOUTCNTL,CSL_PLL_PLLOUTCNTL_OUT_DIV_EN,0);\
N	CSL_FINS(CSL_PLL_REGS->PLLINCNTL,CSL_PLL_PLLINCNTL_RP_BYPASS,0);\
N	CSL_FINS(CSL_PLL_REGS->PLLINCNTL,CSL_PLL_PLLINCNTL_RD,120);\
N	CSL_FINS(CSL_PLL_REGS->PLLCNTL1,CSL_PLL_PLLCNTL1_PLL_STANDBY,0);\
N	CSL_FINS(CSL_PLL_REGS->PLLCNTL2,CSL_PLL_PLLCNTL2_PLL_DIS,0);\
N	asm("\trpt #1000");\
N	asm("\tnop");
X#define PLL_CNFIG(VP,VS) 	CSL_FINS(CSL_PLL_REGS->PLLCNTL2,CSL_PLL_PLLCNTL2_PLL_DIS,1);	CSL_FINS(CSL_PLL_REGS->PLLCNTL1,CSL_PLL_PLLCNTL1_VP,VP);	CSL_FINS(CSL_PLL_REGS->PLLINCNTL,CSL_PLL_PLLINCNTL_VS,VS);	CSL_FINS(CSL_PLL_REGS->PLLOUTCNTL,CSL_PLL_PLLOUTCNTL_OUT_DIV_EN,0);	CSL_FINS(CSL_PLL_REGS->PLLINCNTL,CSL_PLL_PLLINCNTL_RP_BYPASS,0);	CSL_FINS(CSL_PLL_REGS->PLLINCNTL,CSL_PLL_PLLINCNTL_RD,120);	CSL_FINS(CSL_PLL_REGS->PLLCNTL1,CSL_PLL_PLLCNTL1_PLL_STANDBY,0);	CSL_FINS(CSL_PLL_REGS->PLLCNTL2,CSL_PLL_PLLCNTL2_PLL_DIS,0);	asm("\trpt #1000");	asm("\tnop");
N
N/*
NRP_bypass is 0 since Input clock is 12MHz and has to be scaled down betweek 30kHz-170kHz for PD input
N*/
N
N/*****************************************************************************\
N* Peripheral Instance counts
X
N\*****************************************************************************/
N#define CSL_SAR_PER_CNT		1
N#define CSL_USB_PER_CNT		1
N#define CSL_I2C_PER_CNT		1                                              
N#define CSL_UART_PER_CNT	1                                       
N#define CSL_SPI_PER_CNT		1                                       
N#define CSL_MMCSD_PER_CNT	2                                       
N#define CSL_LCDC_PER_CNT		1    
N#define CSL_RTC_PER_CNT		1      
N#define CSL_DMA_PER_CNT		4                                
N
N#define CORAZON_SILICON		                                   	
N#define SDRAMTIMING0	0x5810
N#define SDRAMTIMING1	0x4221
N//#define	PLL_60MHz
N//#define	PLL_100MHz
N
N/*****************************************************************************\
N* Peripheral Overlay Structures
X
N\*****************************************************************************/
Ntypedef volatile ioport CSL_UsbRegs          * CSL_UsbRegsOvly;    
Ntypedef volatile ioport CSL_I2cRegs          * CSL_I2cRegsOvly;    
Ntypedef volatile ioport CSL_I2sRegs          * CSL_I2sRegsOvly;         
Ntypedef volatile ioport CSL_EmifRegs         * CSL_EmifRegsOvly;                          
Ntypedef volatile ioport CSL_UartRegs         * CSL_UartRegsOvly;                  
Ntypedef volatile ioport CSL_SpiRegs          * CSL_SpiRegsOvly;                   
Ntypedef volatile ioport CSL_MmcsdRegs        * CSL_MmcsdRegsOvly;                 
Ntypedef volatile ioport CSL_LcdcRegs         * CSL_LcdcRegsOvly;   
Ntypedef volatile ioport CSL_RtcRegs          * CSL_RtcRegsOvly;      
Ntypedef volatile ioport CSL_DmaRegs          * CSL_DmaRegsOvly;
Ntypedef volatile ioport CSL_SarRegs          * CSL_SarRegsOvly;
Ntypedef volatile ioport CSL_PllRegs          * CSL_PllRegsOvly;  
Ntypedef volatile ioport CSL_DmaEvtRegs       * CSL_DmaEvtIntRegsOvly;
N
N
N
N/*****************************************************************************\
N* Peripheral Base Address
X
N\*****************************************************************************/
N#define CSL_USB_REGS                    ((CSL_UsbRegsOvly)  0x8000) 
N#define CSL_SAR_REGS                    ((CSL_SarRegsOvly)  0x7000) 
N#define CSL_EMIF_REGS                   ((CSL_EmifRegsOvly) 0x1000) 
N#define CSL_I2C_0_REGS                  ((CSL_I2cRegsOvly)  0x1A00) 
N#define CSL_I2S0_REGS              		((CSL_I2sRegsOvly) 	0x2800)       
N#define CSL_I2S1_REGS              		((CSL_I2sRegsOvly) 	0x2900)       
N#define CSL_I2S2_REGS              		((CSL_I2sRegsOvly) 	0x2A00)       
N#define CSL_I2S3_REGS              		((CSL_I2sRegsOvly) 	0x2B00)   
N#define CSL_UART_REGS                   ((CSL_UartRegsOvly) 0x1B00)
N#define CSL_SPI_REGS                   	((CSL_SpiRegsOvly)  0x3000)          
N#define CSL_MMCSD0_REGS           		((CSL_MmcsdRegsOvly)0x3A00)        
N#define CSL_MMCSD1_REGS           		((CSL_MmcsdRegsOvly)0x3B00)        
N#define CSL_LCDC_REGS              		((CSL_LcdcRegsOvly) 0x2E00)  
N#define CSL_RTC_REGS              		((CSL_RtcRegsOvly)  0x1900) 
N#define CSL_PLL_REGS              		((CSL_PllRegsOvly)  0x1C20)  
N#define CSL_DMA0_REGS              		((CSL_DmaRegsOvly)  0x0C00)       
N#define CSL_DMA1_REGS              		((CSL_DmaRegsOvly)  0x0D00)       
N#define CSL_DMA2_REGS              		((CSL_DmaRegsOvly)  0x0E00)       
N#define CSL_DMA3_REGS              		((CSL_DmaRegsOvly)  0x0F00)  
N#define CSL_DMAEVTINT_REGS              ((CSL_DmaEvtIntRegsOvly)0x1C1A)            
N
N/** \brief I2C Module Instances
N* 
N*/
Ntypedef enum {
N  CSL_I2C_ANY    = -1, /**< <b>: Any instance of I2C module</b> */
N  CSL_I2C_0      =  0  /**< <b>: I2C Instance 0</b> */
N} CSL_I2cNum;
N
N
N/** \brief GPIO Module Instances
N* 
N*/
Ntypedef enum {
N  CSL_GPIO_ANY    = -1, /**< <b>: Any instance of GPIO module</b> */
N  CSL_GPIO        =  0,  /**< <b>: GPIO </b> */
N  CSL_MGPIO       =  1  /**< <b>: Multiplexed GPIO </b> */
N} CSL_GpioNum;
N
N
N/** \brief  UART Module Instances
N* 
N*/
Ntypedef enum {
N  CSL_UART_ANY    = -1, /**< <b>: Any instance of UART module</b> */
N  CSL_UART_0          =  0,  /**< <b>: UART Instance 0</b> */
N  CSL_UART_1          =  1  /**< <b>: UART Instance 1</b> */
N} CSL_UartNum;
N
N/** \brief  SPI Module Instances
N* 
N*/
Ntypedef enum {
N  CSL_SPI_ANY    = -1, /**< <b>: Any instance of SPI module</b> */
N  CSL_SPI_0      =  0  /**< <b>: SPI Instance 0</b> */
N} CSL_SpiNum;
N
N/** \brief  MMC/SD Module Instances
N* 
N*/
Ntypedef enum {
N  CSL_MMCSD_ANY    = -1, /**< <b>: Any instance of MMCSD module</b> */
N  CSL_MMCSD_0      =  0  /**< <b>: MMCSD Instance 0</b> */
N} CSL_MmcsdNum;
N
N
N/** \brief  DMA Module Instances
N* 
N*/
Ntypedef enum {
N  CSL_DMA_ANY    = -1, /**< <b>: Any instance of DMA module</b> */
N  CSL_DMA_0      =  0  /**< <b>: DMA Instance 0</b> */
N} CSL_DmaNum;
N
N/** @brief Enumerations for DMA channels */
Ntypedef enum {
N    CSL_DMA_CHA_ANY		   =		     -1,  /**< Any Channel */
N    CSL_DMA_CHA_0		   =		      0,  /**< Channel 0 */
N    CSL_DMA_CHA_1		   =		      1,  /**< Channel 1 */
N    CSL_DMA_CHA_2		   =		      2   /**< Channel 2 */
N} CSL_DmaChaNum;
N
N/** \brief  ATA Module Instances
N* 
N*/
Ntypedef enum {
N    CSL_ATA_PRIMARY = 0,
N    CSL_ATA_0 = 0,
N    CSL_ATA_SECONDARY = 1,
N    CSL_ATA_1 = 0,    
N    CSL_ATA_ANY = -1
N} CSL_AtaNum;
N
N/** \brief LCDC Module Instances
N*
N*/
N
Ntypedef enum {
N  CSL_LCDC_ANY    = -1, /**< <b>: Used to refer any instance of LCDC
N			      module</b> */
N  CSL_LCDC_0      =  0  /**< <b>: LCDC Instance 0</b> */
N} CSL_LcdcNum;
N
N/** \brief  NTSC/PAL Module Instances
N* 
N*/
Ntypedef enum {
N  CSL_NTSC_ANY    = -1, /**< <b>: Any instance of NTSC module</b> */
N  CSL_NTSC_0      =  0  /**< <b>: NTSC Instance 0</b> */
N} CSL_NtscNum;
N
N/** \brief EMIF Module Instances
N* 
N*/
Ntypedef enum {
N  CSL_EMIF_ANY    = -1, /**< <b>: Any instance of EMIF module</b> */
N  CSL_EMIF_0      =  0  /**< <b>: EMIF Instance 0</b> */
N} CSL_EmifNum;
N
Nextern void * _CSL_mmcsdlookup[];
N
N/*
Nextern void * _CSL_i2clookup[];
Nextern void * _CSL_uartlookup[];
Nextern void * _CSL_spilookup[];
Nextern void * _CSL_lcdclookup[];
N*/
N#endif
L 25 "../inc/ECGDemoNonBios.h" 2
N#include "cslr.h"
N#include "LCD_FontTable.h"
L 1 "..\inc\LCD_FontTable.h" 1
N/******************************************************************************
N**File Name			: lcdFontTable.h
N**File Description	:LCD Driver Font table defnition API header file
N**Created By		:
N**Creation Date		:
N**Last modified date:
N**Remarks			:
N*******************************************************************************/
N
N
N#ifndef _LCDFONTTABLE_H
N#define _LCDFONTTABLE_H
N
N/***********************************************************************
N *                       #define SECTION                               *
N ***********************************************************************/
N
N#define FONT_BITMAP_SIZE    8
N/**< hash define value for font bimap size */
N#define FONT_TABLE_SIZE      sizeof(lcdFontTable)
N/**< size of complete font table */
N
N#define LCD_CHAR_HEIGHT 				8
N/**< no of bytes to diplay one character  */
N#define LCD_CHAR_WIDTH                   8
N/**< total no of bits taken by one character */
N
N/***********************************************************************
N *                    ERROR & WARNING CODES                            *
N ***********************************************************************/
N
N/***********************************************************************
N *                    CONSTANTS DECLARATION SECTION                    *
N ***********************************************************************/
N
Nstatic unsigned char lcdFontTable[] = 
N	{
N    		/* 32 0x20 ' ' */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    
N	/* 33 0x21 '!' */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0x18, /* 00011000 */
N    0x3c, /* 00111100 */
N    0x3c, /* 00111100 */
N    0x3c, /* 00111100 */
N    0x18, /* 00011000 */
N    0x18, /* 00011000 */
N    
N	/* 34 0x22 '"' */
N    0x00, /* 00000000 */
N    0x66, /* 01100110 */
N    0x66, /* 01100110 */
N    0x66, /* 01100110 */
N    0x24, /* 00100100 */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    
N
N	/* 35 0x23 '#' */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0x6c, /* 01101100 */
N    0x6c, /* 01101100 */
N    0xfe, /* 11111110 */
N    0x6c, /* 01101100 */
N    0x6c, /* 01101100 */
N    
N
N	/* 36 0x24 '$' */
N    0x18, /* 00011000 */
N    0x18, /* 00011000 */
N    0x7c, /* 01111100 */
N    0xc6, /* 11000110 */
N    0xc2, /* 11000010 */
N    0xc0, /* 11000000 */
N    0x7c, /* 01111100 */
N    0x06, /* 00000110 */
N    
N
N	/* 37 0x25 '%' */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0xc2, /* 11000010 */
N    0xc6, /* 11000110 */
N    0x0c, /* 00001100 */
N    0x18, /* 00011000 */
N    
N	/* 38 0x26 '&' */
N   
N    0x00, /* 00000000 */
N    0x7c, /* 01111100 */
N    0xFE, /* 11111110 */
N    0xFE, /* 11111110 */
N    0xFE, /* 11111110 */
N    0xFE, /* 11111110 */
N    0xFE, /* 11111110 */
N    0x7c, /* 01111100 */
N    
N
N	/* 39 0x27 ''' */
N    0x00, /* 00000000 */
N    0x30, /* 00110000 */
N    0x30, /* 00110000 */
N    0x30, /* 00110000 */
N    0x60, /* 01100000 */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    
N
N	/* 40 0x28 '(' */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0x0c, /* 00001100 */
N    0x18, /* 00011000 */
N    0x30, /* 00110000 */
N    0x30, /* 00110000 */
N    0x30, /* 00110000 */
N    0x30, /* 00110000 */
N    
N
N	/* 41 0x29 ')' */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0x30, /* 00110000 */
N    0x18, /* 00011000 */
N    0x0c, /* 00001100 */
N    0x0c, /* 00001100 */
N    0x0c, /* 00001100 */
N    0x0c, /* 00001100 */
N    
N
N	/* 42 0x2a '*' */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0x66, /* 01100110 */
N    0x3c, /* 00111100 */
N    0xff, /* 11111111 */
N    
N
N	/* 43 0x2b '+' */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0x18, /* 00011000 */
N    0x18, /* 00011000 */
N    0x7e, /* 01111110 */
N    
N
N	/* 44 0x2c ',' */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    
N
N	/* 45 0x2d '-' */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0xfe, /* 11111110 */
N    
N	/* 46 0x2e '.' */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N
N	/* 47 0x2f '/' */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0x02, /* 00000010 */
N    0x06, /* 00000110 */
N    0x0c, /* 00001100 */
N    0x18, /* 00011000 */
N    
N
N
N	/* 48 0x30 '0' */
N    0x00, /* 00000000 */
N    0x7c, /* 01111100 */
N    0x82, /* 10000010 */
N    0x82, /* 10000010 */
N    0x82, /* 10000010 */
N    0x82, /* 10000010 */
N    0x82, /* 10000010 */
N    0x7c, /* 01111100 */
N    
N
N	/* 49 0x31 '1' */
N    0x00, /* 00000000 */
N    0x18, /* 00011000 */
N    0x38, /* 00111000 */
N    0x08, /* 00001000 */
N    0x08, /* 00001000 */
N    0x08, /* 00001000 */
N    0x08, /* 00001000 */
N    0x3e, /* 00111110 */
N    
N
N        /* 50 0x32 '2' */
N    0x00, /* 00000000 */
N    0xfe, /* 11111110 */
N    0x02, /* 00000010 */
N    0x04, /* 00000100 */
N    0x10, /* 00010000 */
N    0x40, /* 01000000 */
N    0x80, /* 10000000 */
N    0xfe, /* 11111110 */
N    
N
N	/* 51 0x33 '3' */
N    0x00, /* 00000000 */
N    0xfc, /* 11111100 */
N    0x02, /* 00000010 */
N    0x02, /* 00000010 */
N    0xfc, /* 11111100 */
N    0x02, /* 00000010 */
N    0x02, /* 00000010 */
N    0xfc, /* 11111100 */
N    
N
N	/* 52 0x34 '4' */
N    0x00, /* 00000000 */
N    0x0c, /* 00001100 */
N    0x14, /* 00010100 */
N    0x24, /* 00100100 */
N    0x44, /* 01000100 */
N    0xfe, /* 11111110 */
N    0x04, /* 00000100 */
N    0x04, /* 00000100 */
N    
N
N	/* 53 0x35 '5' */
N    0x00, /* 00000000 */
N    0xfe, /* 11111110 */
N    0x80, /* 10000000 */
N    0x80, /* 10000000 */
N    0xfc, /* 11111100 */
N    0x02, /* 00000010 */
N    0x02, /* 00000010 */
N    0xfe, /* 11111110 */
N    
N
N	/* 54 0x36 '6' */
N    0x00, /* 00000000 */
N    0x78, /* 01111000 */
N    0x80, /* 10000000 */
N    0x80, /* 10000000 */
N    0x80, /* 10000000 */
N    0xfe, /* 11111110 */
N    0x82, /* 10000010 */
N    0x7c, /* 01111100 */
N    
N
N	/* 55 0x37 '7' */
N    0x00, /* 00000000 */
N    0xfe, /* 11111110 */
N    0x02, /* 00000010 */
N    0x04, /* 00000100 */
N    0x08, /* 00001000 */
N    0x10, /* 00010000 */
N    0x10, /* 00010000 */
N    0x10, /* 00010000 */
N    
N
N	/* 56 0x38 '8' */
N    0x00, /* 00000000 */
N    0x7c, /* 01111100 */
N    0x82, /* 10000010 */
N    0x82, /* 10000010 */
N    0x7c, /* 01111100 */
N    0x82, /* 10000010 */
N    0x82, /* 10000010 */
N    0x7c, /* 01111100 */
N    
N
N	/* 57 0x39 '9' */
N    0x00, /* 00000000 */
N    0x7c, /* 01111100 */
N    0x82, /* 10000010 */
N    0x82, /* 10000010 */
N    0x7e, /* 01111110 */
N    0x02, /* 00000010 */
N    0x02, /* 00000010 */
N    0x7c, /* 01111100 */
N    
N
N	/* 58 0x3a ':' */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0x18, /* 00011000 */
N    0x00, /* 00000000 */
N    0x18, /* 00011000 */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    
N
N	/* 59 0x3b ';' */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0x18, /* 00011000 */
N    0x00, /* 00000000 */
N    0x18, /* 00011000 */
N    0x30, /* 00110000 */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    
N	/* 60 0x3c '<' */
N    0x00, /* 00000000 */
N    0x08, /* 00001000 */
N    0x10, /* 00010000 */
N    0x20, /* 00100000 */
N    0xc0, /* 11000000 */
N    0x20, /* 00100000 */
N    0x10, /* 00010000 */
N    0x08, /* 00001000 */
N    
N
N	/* 61 0x3d '=' */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0xfe, /* 11111110 */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0xfe, /* 11111110 */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    
N
N	/* 62 0x3e '>' */
N   
N    0x00, /* 00000000 */
N    0x10, /* 00010000 */
N    0x08, /* 00001000 */
N    0x04, /* 00000100 */
N    0x02, /* 00000010 */
N    0x04, /* 00000100 */
N    0x08, /* 00001000 */
N    0x10, /* 00010000 */
N   
N
N	/* 63 0x3f '?' */
N    0x00, /* 00000000 */
N    0x7c, /* 01111100 */
N    0x02, /* 00000010 */
N    0x02, /* 00000010 */
N    0x1c, /* 00011100 */
N    0x10, /* 00010000 */
N    0x00, /* 00000000 */
N    0x10, /* 00010000 */
N    
N
N	/* 64 0x40 '@' */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0x7c, /* 01111100 */
N    0xc6, /* 11000110 */
N    0xc6, /* 11000110 */
N    0xde, /* 11011110 */
N    0xde, /* 11011110 */
N    
N
N/* 65 0x41 'A' */
N    
N    0x00, /* 00000000 */
N    0x7c, /* 01111100 */
N    0x82, /* 10000010 */
N    0x82, /* 10000010 */
N    0x82, /* 10000010 */
N    0xfe, /* 11111110 */
N    0x82, /* 10000010 */
N    0x82, /* 10000010 */
N
N 	/* 66 0x42 B  */
N    0x00, /* 00000000 */
N    0xfc, /* 11111100 */
N    0x42, /* 01000010 */
N    0x42, /* 01000010 */
N    0x7c, /* 01111100 */
N    0x42, /* 01000010 */
N    0x42, /* 01000010 */
N    0xfc, /* 11111100 */
N    
N    
N /* 67 0x43 'C' */
N    
N    0x00, /* 00000000 */
N    0x3c, /* 00111100 */
N    0x42, /* 01000010 */
N    0x40, /* 01000000 */
N    0x40, /* 01000000 */
N    0x40, /* 01000000 */
N    0x42, /* 01000010 */
N    0x3c, /* 00111100 */
N     
N	/* 68 0x44 'D' */
N    0x00, /* 00000000 */
N    0xfc, /* 11111100 */
N    0x42, /* 01000010 */
N    0x42, /* 01000010 */
N    0x42, /* 01000010 */
N    0x42, /* 01000010 */
N    0x42, /* 01000010 */
N    0xfc, /* 11111100 */
N    
N
N	/* 69 0x45 'E' */
N    0x00, /* 00000000 */
N    0xfe, /* 11111110 */
N    0x80, /* 10000000 */
N    0x80, /* 10000000 */
N    0xf8, /* 11111000 */
N    0x80, /* 10000000 */
N    0x80, /* 10000000 */
N    0xfe, /* 11111110 */
N    
N
N	/* 70 0x46 'F' */
N    0x00, /* 00000000 */
N    0xfe, /* 11111110 */
N    0x42, /* 01000010 */
N    0x40, /* 01000000 */
N    0x7c, /* 01111100 */
N    0x40, /* 01000000 */
N    0x40, /* 01000000 */
N    0xe0, /* 11100000 */
N  
N
N	/* 71 0x47 'G' */
N    0x00, /* 00000000 */
N    0x7e, /* 01111110 */
N    0x80, /* 10000000 */
N    0x80, /* 10000000 */
N    0x9e, /* 10011110 */
N    0x82, /* 10000010 */
N    0x82, /* 10000010 */
N    0x7c, /* 01111100 */
N   
N 
N	/* 72 0x48 'H' */
N    0x00, /* 00000000 */
N    0x82, /* 10000010 */
N    0x82, /* 10000010 */
N    0x82, /* 10000010 */
N    0xfe, /* 11111110 */
N    0x82, /* 10000010 */
N    0x82, /* 10000010 */
N    0x82, /* 10000010 */
N    
N
N	/* 73 0x49 'I' */
N    0x00, /* 00000000 */
N    0x7c, /* 01111100 */
N    0x10, /* 00010000 */
N    0x10, /* 00010000 */
N    0x10, /* 00010000 */
N    0x10, /* 00010000 */
N    0x10, /* 00010000 */
N    0x7c, /* 01111100 */
N    
N
N	/* 74 0x4a 'J' */
N    0x00, /* 00000000 */
N    0x3e, /* 00111110 */
N    0x04, /* 00000100 */
N    0x04, /* 00000100 */
N    0x04, /* 00000100 */
N    0x84, /* 10000100 */
N    0x84, /* 10000100 */
N    0x78, /* 01111000 */
N
N	/* 75 0x4b 'K' */
N    0x00, /* 00000000 */
N    0x82, /* 10000010 */
N    0x82, /* 10000010 */
N    0x84, /* 10000100 */
N    0xf8, /* 11111000 */
N    0xf4, /* 10000100 */
N    0xf2, /* 10000010 */
N    0xf2, /* 10000010 */
N    
N
N	/*76 0x4c 'L' */
N    0x00, /* 00000000 */
N    0x80, /* 10000000 */
N    0x80, /* 10000000 */
N    0x80, /* 10000000 */
N    0x80, /* 10000000 */
N    0x80, /* 10000000 */
N    0x82, /* 10000010 */
N    0xfe, /* 11111110 */
N    
N
N	/* 77 0x4d 'M' */
N   
N    0x00, /* 00000000 */
N    0xc6, /* 11000110 */
N    0xaa, /* 10101010 */
N    0x92, /* 10010010 */
N    0x80, /* 10000000 */
N    0x82, /* 10000010 */
N    0x82, /* 10000010 */
N    0x82, /* 10000010 */
N    
N    /* 78  0x4e 'N' */
N    
N    0x00, /* 00000000 */
N    0xc2, /* 11000010 */
N    0xa2, /* 10100010 */
N    0x92, /* 10010010 */
N    0x8a, /* 10001010 */
N    0x86, /* 10000110 */
N    0x82, /* 10000010 */
N    0x82, /* 10000010 */
N   
N    
N
N	/* 79 0x4f 'O' */
N    0x00, /* 00000000 */
N    0x7c, /* 01111100 */
N    0x82, /* 10000010 */
N    0x82, /* 10000010 */
N    0x82, /* 10000010 */
N    0x82, /* 10000010 */
N    0x82, /* 10000010 */
N    0x7c, /* 01111100 */
N    
N
N	/* 80 0x50 'P' */
N    0x00, /* 00000000 */
N    0xfc, /* 11111100 */
N    0x42, /* 01000010 */
N    0x42, /* 01000010 */
N    0x7c, /* 01111100 */
N    0x40, /* 01000000 */
N    0x40, /* 01000000 */
N    0xe0, /* 11100000 */
N    
N
N	/* 81 0x51 'Q' */
N    0x00, /* 00000000 */
N    0x7c, /* 01111100 */
N    0x82, /* 10000010 */
N    0x82, /* 10000010 */
N    0x82, /* 10000010 */
N    0x8a, /* 10001010 */
N    0x7c, /* 01111100 */
N    0x07, /* 00000111 */
N    
N
N	/* 82 0x52 'R' */
N    0x00, /* 00000000 */
N    0xfc, /* 11111100 */
N    0x42, /* 01000010 */
N    0x42, /* 01000010 */
N    0x7c, /* 01111100 */
N    0x50, /* 01010000 */
N    0x48, /* 01001000 */
N    0x46, /* 01000110 */
N    
N
N	/* 83 0x53 'S' */
N    0x00, /* 00000000 */
N    0x7e, /* 01111110 */
N    0x80, /* 10000000 */
N    0x80, /* 10000000 */
N    0xfc, /* 11111100 */
N    0x02, /* 00000010 */
N    0x02, /* 00000010 */
N    0xfc, /* 11111100 */
N    
N
N	/* 84 0x54 'T' */
N    0x00, /* 00000000 */
N    0xfe, /* 11111110 */
N    0x10, /* 00010000 */
N    0x10, /* 00010000 */
N    0x10, /* 00010000 */
N    0x10, /* 00010000 */
N    0x10, /* 00010000 */
N    0x10, /* 00010000 */
N    
N
N	/* 85 0x55 'U' */
N    0x00, /* 00000000 */
N    0x82, /* 10000010 */
N    0x82, /* 10000010 */
N    0x82, /* 10000010 */
N    0x82, /* 10000010 */
N    0x82, /* 10000010 */
N    0x82, /* 10000010 */
N    0x7c, /* 01111100 */
N    
N
N	/* 86 0x56 'V' */
N    0x00, /* 00000000 */
N    0x82, /* 10000010 */
N    0x82, /* 10000010 */
N    0x82, /* 10000010 */
N    0x82, /* 10000010 */
N    0x44, /* 01000100 */
N    0x28, /* 00101000 */
N    0x10, /* 00010000 */
N    
N
N	/* 87 0x57 'W' */
N    0x00, /* 00000000 */
N    0x82, /* 10000010 */
N    0x82, /* 10000010 */
N    0x82, /* 10000010 */
N    0x82, /* 10000010 */
N    0x92, /* 10010010 */
N    0xaa, /* 10101010 */
N    0xc6, /* 11000110 */
N    
N
N	/* 88 0x58 'X' */
N    0x00, /* 00000000 */
N    0x82, /* 10000010 */
N    0x44, /* 01000100 */
N    0x28, /* 00101000 */
N    0x10, /* 00010000 */
N    0x28, /* 00101000 */
N    0x44, /* 01000100 */
N    0x82, /* 10000010 */
N    
N
N	/* 89 0x59 'Y' */
N    0x00, /* 00000000 */
N    0x82, /* 10000010 */
N    0x82, /* 10000010 */
N    0x44, /* 01000100 */
N    0x38, /* 00111000 */
N    0x10, /* 00010000 */
N    0x10, /* 00010000 */
N    0x10, /* 00010000 */
N    
N
N	/* 90 0x5a 'Z' */
N    0x00, /* 00000000 */
N    0xfe, /* 11111110 */
N    0x02, /* 00000010 */
N    0x04, /* 00000100 */
N    0x10, /* 00010000 */
N    0x40, /* 01000000 */
N    0x80, /* 10000000 */
N    0xfe, /* 11111110 */
N    
N	
N    /* 91 0x5b '[' */
N    0x00, /* 00000000 */
N    0xf8, /* 11111000 */
N    0x80, /* 10000000 */
N    0x80, /* 10000000 */
N    0x80, /* 10000000 */
N    0x80, /* 10000000 */
N    0x80, /* 10000000 */
N    0xf8, /* 11111000 */
N    
N	/* 92 0x5c '\' */
N    0x00, /* 00000000 */
N    0x80, /* 10000000 */
N    0x40, /* 01000000 */
N    0x20, /* 00100000 */
N    0x10, /* 00010000 */
N    0x08, /* 00001000 */
N    0x04, /* 00000100 */
N    0x02, /* 00000010 */
N    
N
N	/* 93 0x5d ']' */
N    0x00, /* 00000000 */
N    0x1e, /* 00011110 */
N    0x02, /* 00000010 */
N    0x02, /* 00000010 */
N    0x02, /* 00000010 */
N    0x02, /* 00000010 */
N    0x02, /* 00000010 */
N    0x1e, /* 00011110 */
N    
N
N	/* 94 0x5e '^' */
N    0x00, /* 00000000 */
N    0x10, /* 00010000 */
N    0x28, /* 00101000 */
N    0x44, /* 01000100 */
N    0x82, /* 10000010 */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    
N	/* 95 0x5f '_' */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0xfe, /* 11111110 */
N    
N	/* 96 0x60 '`' */
N    0x00, /* 00000000 */
N    0x30, /* 00110000 */
N    0x18, /* 00011000 */
N    0x0c, /* 00001100 */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    
N
N	/* 97 0x61 'a' */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0x78, /* 01111000 */
N    0x04, /* 00000100 */
N    0x3c, /* 00111100 */
N    0x44, /* 01000100 */
N    0x7e, /* 01111110 */
N    
N
N	/* 98 0x62 'b' */
N    0x00, /* 00000000 */
N    0x40, /* 01000000 */
N    0x40, /* 01000000 */
N    0x40, /* 01000000 */
N    0x7c, /* 01111100 */
N    0x42, /* 01000010 */
N    0x42, /* 01000010 */
N    0x7c, /* 01111100 */
N    
N
N	/* 99 0x63 'c' */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0x7e, /* 01111110 */
N    0x80, /* 10000000 */
N    0x80, /* 10000000 */
N    0x80, /* 10000000 */
N    0x7e, /* 01111110 */
N    
N
N	/* 100 0x64 'd' */
N    0x00, /* 00000000 */
N    0x02, /* 00000010 */
N    0x02, /* 00000010 */
N    0x02, /* 00000010 */
N    0x3e, /* 00111110 */
N    0x42, /* 01000010 */
N    0x42, /* 01000010 */
N    0x7e, /* 01111110 */
N    
N
N	/* 101 0x65 'e' */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0x7e, /* 01111110 */
N    0x82, /* 10000010 */
N    0xfe, /* 11111110 */
N    0x80, /* 10000000 */
N    0x7e, /* 01111110 */
N    
N
N	/* 102 0x66 'f' */
N    0x00, /* 00000000 */
N    0x1c, /* 00011100 */
N    0x12, /* 00010010 */
N    0x10, /* 00010000 */
N    0x7c, /* 01111100 */
N    0x10, /* 00010000 */
N    0x10, /* 00010000 */
N    0x10, /* 00010000 */
N    
N	/* 103 0x67 'g' */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0x7c, /* 01111100 */
N    0x82, /* 10000010 */
N    0xfe, /* 11111110 */
N    0x02, /* 00000010 */
N    0xfc, /* 11111100 */
N    
N	/* 104 0x68 'h' */
N    0x00, /* 00000000 */
N    0x80, /* 10000000 */
N    0x80, /* 10000000 */
N    0xf8, /* 11111000 */
N    0x84, /* 10000100 */
N    0x82, /* 10000010 */
N    0x82, /* 10000010 */
N    0x82, /* 10000010 */
N    
N
N	/* 105 0x69 'i' */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0x10, /* 00010000 */
N    0x00, /* 00000000 */
N    0x10, /* 00010000 */
N    0x10, /* 00010000 */
N    0x10, /* 00010000 */
N    0x10, /* 00010000 */
N    
N
N	/* 106 0x6a 'j' */
N    0x00, /* 00000000 */
N    0x02, /* 00000010 */
N    0x00, /* 00000000 */
N    0x02, /* 00000010 */
N    0x02, /* 00000010 */
N    0x42, /* 01000010 */
N    0x42, /* 01000010 */
N    0x3c, /* 00111100 */
N    
N	/* 107 0x6b 'k' */
N    0x00, /* 00000000 */
N    0x80, /* 10000000 */
N    0x80, /* 10000000 */
N    0x82, /* 10000010 */
N    0x84, /* 10000100 */
N    0xf8, /* 11111000 */
N    0x84, /* 10000100 */
N    0x82, /* 10000010 */
N    
N
N	/* 108 0x6c 'l' */
N    0x00, /* 00000000 */
N    0x70, /* 01110000 */
N    0x10, /* 00010000 */
N    0x10, /* 00010000 */
N    0x10, /* 00010000 */
N    0x10, /* 00010000 */
N    0x10, /* 00010000 */
N    0x38, /* 00111000 */
N    
N
N	/* 109 0x6d 'm' */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0x80, /* 10000000 */
N    0xee, /* 11101110 */
N    0x92, /* 10010010 */
N    0x92, /* 10010010 */
N    0x92, /* 10010010 */
N    
N
N	/* 110 0x6e 'n' */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0xb8, /* 10111000 */
N    0x44, /* 01000100 */
N    0x44, /* 01000100 */
N    0x44, /* 01000100 */
N    0x44, /* 01000100 */
N    
N	/* 111 0x6f 'o' */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0x3c, /* 00111100 */
N    0x42, /* 01000010 */
N    0x42, /* 01000010 */
N    0x42, /* 01000010 */
N    0x3c, /* 00111100 */
N    
N	/* 112 0x70 'p' */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0xfc, /* 11111100 */
N    0x82, /* 10000010 */
N    0xfc, /* 11111100 */
N    0x80, /* 10000000 */
N    0x80, /* 10000000 */
N    0x80, /* 10000000 */
N    
N	/* 113 0x71 'q' */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0x7c, /* 01111100 */
N    0x82, /* 10000010 */
N    0xfe, /* 11111110 */
N    0x02, /* 00000010 */
N    0x02, /* 00000010 */
N    0x02, /* 00000010 */
N    
N	/* 114 0x72 'r' */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0x2e, /* 00101110 */
N    0x10, /* 00010000 */
N    0x10, /* 00010000 */
N    0x10, /* 00010000 */
N    0x10, /* 00010000 */
N    
N	/* 115 0x73 's' */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0x7e, /* 01111110 */
N    0x80, /* 10000000 */
N    0x7c, /* 01111100 */
N    0x02, /* 00000010 */
N    0xfc, /* 11111100 */
N    
N
N	/* 116 0x74 't' */
N    0x00, /* 00000000 */
N    0x20, /* 00100000 */
N    0xf8, /* 11111000 */
N    0x20, /* 00100000 */
N    0x20, /* 00100000 */
N    0x24, /* 00100100 */
N    0x24, /* 00100100 */
N    0x18, /* 00011000 */
N    
N	/* 117 0x75 'u' */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0x84, /* 10000100 */
N    0x84, /* 10000100 */
N    0x84, /* 10000100 */
N    0x84, /* 10000100 */
N    0x7a, /* 01111010 */
N    
N	/* 118 0x76 'v' */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0x82, /* 10000010 */
N    0x82, /* 10000010 */
N    0x44, /* 01000100 */
N    0x28, /* 00101000 */
N    0x10, /* 00010000 */
N    
N
N	/* 119 0x77 'w' */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0x82, /* 10000010 */
N    0x82, /* 10000010 */
N    0x92, /* 10010010 */
N    0xaa, /* 10101010 */
N    0xc6, /* 11000110 */
N    
N	/* 120 0x78 'x' */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0x82, /* 10000010 */
N    0x28, /* 00101000 */
N    0x10, /* 00010000 */
N    0x28, /* 00101000 */
N    0xc6, /* 11000110 */
N    
N	/* 121 0x79 'y' */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0x22, /* 00100010 */
N    0x22, /* 00100010 */
N    0x1e, /* 00011110 */
N    0x02, /* 00000010 */
N    0x7c, /* 01111100 */
N    
N
N	/* 122 0x7a 'z' */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0x00, /* 00000000 */
N    0xfe, /* 11111110 */
N    0x02, /* 00000010 */
N    0x7c, /* 01111100 */
N    0x80, /* 10000000 */
N    0xfe, /* 11111110 */
N    
N
N    };	
N/**< lcd Font table declaration variable  */
N
N#endif  /* #ifndef LCDFONTTABLE_H */
N/*EOF*/
L 27 "../inc/ECGDemoNonBios.h" 2
N#include "ECGGlobals.h"
L 1 "..\inc\ECGGlobals.h" 1
N/******************************************************************************
N**File Name			: ECGGlobals.h
N**File Description	:Global definitions used for ECG System
N**Created By		:
N**Creation Date		:
N**Last modified date:
N**Remarks			:
N*******************************************************************************/
N
N
N#ifndef _ECGGLOBALS_H
N#define _ECGGLOBALS_H
N
N/* sampling rate for each channels*/
N#define SAMPLING_RATE (500)
N/* Aqsn Buff size*/
N#define AQSNBUFFSIZE ((2) * (MAXCHAN) )//(SAMPLING_RATE)//FILTERORDER
N/* Lead Info buff size*/
N#define LEADINFOSIZE ((SAMPLING_RATE)/2)
N/* Maximum No of Channels aquired from ADS1258 */
N#define MAXCHAN (8)
N/* Maximum Leads available*/
N#define MAXLEAD MAXCHAN
N/* Filter order*/
N#define FILTERORDER (351)
N/* DC Removal Numerator Coeff*/
N#define NRCOEFF (0.992)
N/* Decimation Factor*/
N#define DECIMATOR (1)
N/* Unit Voltage represented by ADS*/
N#define UNITVOLT (3165.512)
N/* Integer equivalent of max possible hex val*/
N#define MAXHEXVAL (16777261)
N/* ECG Success Status*/
N#define ECG_OK 0
N/* PCA9535 I2C Expander Address*/
N#define I2C_ECGFE_ADDR (0x20)
N/* SPI Success Status*/
N#define SPI_SUCCESS 0
N/* Amplification factor for signal - not used*/
N#define AMP_FACTOR (500)
N/*2sec of new ECG samples are stored in total of 6s*/
N/*#define REFRESH_HR_TIME 	2*/
N#define MAX_HEART_RATE		240
N#define MIN_HEART_RATE		30
N/*threshold = 0.6 * maxima*/
N#define QRS_THRESHOLD_FRACTION	0.7					
N/*70 samples are skipped without threshold detection after getting a maxima*/
N#define MINIMUM_SKIP_WINDOW		50	/*30*/
N/*search for maxima is done for 20 samples after crossing the threshold*/
N#define MAXIMA_SEARCH_WINDOW	40	/*7	*/
N/*Scaling Factor for the First Derivative*/
N#define FD_SCALE_FACTOR		1					
N/*Scaling Factor for the Second derivative*/
N#define SD_SCALE_FACTOR		1					
N/*length of the array which stores Sample index S1,S2........*/
N#define SI_ARRAY_LENGTH		((MAX_HEART_RATE / 60) * TOTAL_STORAGE_TIME ) 
N/*Zoom factor in max condition*/
N#define HIGH  160 /*	625	for old filtering method*/
N/*Default setting for medium Zoom set as default*/
N#define MEDIUM	325 /*1250 for old filtering method*/
N/*Default setting for least Zoom*/
N#define LOW		650/*2500	for old filtering method*/
N/*Total of 6s of data are stored for running HR calculation Algorithm*/
N/*#define TOTAL_STORAGE_TIME 	6	*/
N/*call QRS when 117 samples are filled in Lead_Info Buffer*/
N/*#define FIRST_QRS_CALL 		(LEADINFOSIZE/3)*/
N/*call QRS when 234 samples are filled in Lead_Info Buffer		*/
N/*#define SECOND_QRS_CALL 	(2*(LEADINFOSIZE/3))*/
N/*call QRS when 351 samples are filled in Lead_Info Buffer*/
N/*#define THIRD_QRS_CALL 		(3*(LEADINFOSIZE/3))*/
N/* QRS call factor*/
N#define QRSCALLFACT (LEADINFOSIZE/10)	
N/* Hex value corresponding to Lead-Off condition of Right Arm*/
N#define ELECTRODE_RA 		0x01FE
N/* Hex value corresponding to Lead-Off condition of Left Arm*/
N#define ELECTRODE_LA 		0x01FD
N/* Hex value corresponding to Lead-Off condition of Left Leg*/
N#define ELECTRODE_LL 		0x01FB
N/* Hex value corresponding to Lead-Off condition of V1*/
N#define ELECTRODE_V1 		0x01F7
N/* Hex value corresponding to Lead-Off condition of V2*/
N#define ELECTRODE_V2 		0x01EF
N/* Hex value corresponding to Lead-Off condition of V3*/
N#define ELECTRODE_V3 		0x01DF
N/* Hex value corresponding to Lead-Off condition of V4*/
N#define ELECTRODE_V4 		0x01BF
N/* Hex value corresponding to Lead-Off condition of V5*/
N#define ELECTRODE_V5 		0x017F
N/* Hex value corresponding to Lead-Off condition of V6*/
N#define ELECTRODE_V6 		0x00FF
N/* MASK for Leadoff*/
N#define LEADOFFMASK 0xFFFF
N
N/* LCD Display selection*/
N
N#define DISP_LCD 1
N
N
N/*  0 for Notch @ 50Hz  and 1 for Notch @ 60Hz*/
N
N#define NOTCHFILTERSEL 0
N   
N#define BANDPASS_05_45 0
N
N#define GPIO7SET 0x0080
N#define GPIO8SET 0x0100
N#define GPIO7CLR 0xFF7F
N#define GPIO8CLR 0xFEFF		
N
N
N
N#endif
N/*EOF*/
L 28 "../inc/ECGDemoNonBios.h" 2
N#include "ADS1298.h"
L 1 "..\inc\ADS1298.h" 1
N/******************************************************************************
N**File Name			: ADS1298.h
N**File Description	: Global definitions used for ADS configuration and accessing
N**Created By		:
N**Creation Date		:
N**Last modified date:
N**Remarks			:
N*******************************************************************************/
N
N
N#ifndef _ADS1298_H
N#define _ADS1298_H
N
N/* ADS1298 Commands*/
N
N#define CMD_START   0x08		/* Start conversion command*/
N#define CMD_SDATAC 	0x11		/* Stop read data continuously mode*/
N#define CMD_RESET  	0x06		/* Channel Data Read Command*/
N#define CMD_REGRD 	0x2116	  	/* Register Read Command for 23 registers
N                                   from address 01h */
N#define CMD_RREG1 	0x21		/* Register Write Command for 23 registers  from address 01h */
N#define CMD_RREG2   0x16
N                                  
N#define CMD_RDATAC 	0x10  		/* Read data continuous command*/
N#define CMD_RDATA   0x12        /* Read data in pulse mode */
N
N
N/* Status Bits  of read Data*/
N
N#define STAT_NEW 0x80
N#define STAT_OVF 0x40
N#define STAT_SUPPLY 0x20
N#define STAT_CHID 0x1F
N
N/* Channel ID */
N
N#define	DIFF0	0x0
N#define	DIFF1	0x1
N#define	DIFF2	0x02
N#define	DIFF3	0x03
N#define	DIFF4	0x04
N#define	DIFF5	0x05
N#define	DIFF6	0x06
N#define	DIFF7	0x07
N#define	AIN0	0x08
N#define	AIN1	0x09
N#define	AIN2	0x0A
N#define	AIN3	0x0B
N#define	AIN4	0x0C
N#define	AIN5	0x0D
N#define	AIN6	0x0E
N#define	AIN7	0x0F
N#define	AIN8	0x10
N#define	AIN9	0x11
N#define	AIN10	0x12
N#define	AIN11	0x13
N#define	AIN12	0x14
N#define	AIN13	0x15
N#define	AIN14	0x16
N#define	AIN15	0x17
N#define	OFFSET	0x18
N#define	VCC		0x1A
N#define	TEMP	0x1B
N#define	GAIN	0x1C
N#define	REF		0x1D
N
N#define ADS_GPIOC 0x07
N#define ADS_GPIOD 0x08
N#define REG_MUXSCH 0x02
N
N/* MASKS */
N
N#define CHID_MASK 0x001F
N
N#endif
N/*EOF*/
L 29 "../inc/ECGDemoNonBios.h" 2
N#include "typedef.h"
L 1 "..\inc\typedef.h" 1
N/******************************************************************************
N**File Name			: Typedef.h
N**File Description	:The file holds all the typedefinitions used in ECG system
N**Created By		:
N**Creation Date		:
N**Last modified date:
N**Remarks			:
N*******************************************************************************/
N#ifndef _TYPEDEF_H_
N#define _TYPEDEF_H_
Ntypedef int ECG_Status;						
N
Ntypedef struct 					/* Global structure for ECG system*/
N{
N	Uint16 	ECG_LeadOff;			/* Current Leadoff Value*/
N	Uint8 	FE_Status;				/* Current FE status*/
N	Uint8 	KEY_PressStatus;		/* Current KeyPress Status*/
N	Uint8 	ECG_HeartRate;			/* Current Heart Rate*/
N	Uint8 	ECG_FreezeStatus;
N	Uint8 	ECG_CurrentDisplayLead;		/* Current Display lead*/
N}ECG_System_Info;
N
Ntypedef struct 				
N{
N	/* Pointer to the current filling row of Leadinfo Buff*/
N    unsigned int main_index_ptr;	
N	/* LeadInfo Buffer*/
N    Int16 lead_info_data[LEADINFOSIZE][MAXLEAD];
X    Int16 lead_info_data[(((500))/2)][(8)];
N	
N}ECG_Lead_Info;
N
N#endif
N
N/*EOF*/
L 30 "../inc/ECGDemoNonBios.h" 2
N#include "timer.h"
L 1 "..\inc\timer.h" 1
N /******************************************************************************
N**File Name			: timer.h
N**File Description	:Timer register declaration
N**Created By		:
N**Creation Date		:
N**Last modified date:
N**Remarks			:
N*******************************************************************************/
N
N/*  Timer register declaration*/
N
N#define CPU_TIM0_CTRL ((ioport volatile unsigned*)0x1810)
N#define CPU_TIM0_PLWR ((ioport volatile unsigned*)0x1812)
N#define CPU_TIM0_PHWR ((ioport volatile unsigned*)0x1813)
N#define CPU_TIM0_CLWR ((ioport volatile unsigned*)0x1814)
N#define CPU_TIM0_CHWR ((ioport volatile unsigned*)0x1815)
N#define CPU_TIM0_IER ((ioport volatile unsigned*)0x1816)
N#define CPU_TIMINT_AGGR ((ioport volatile unsigned*)0x1c14)
N#define CPU_PSRCR ((ioport volatile unsigned*)0x1c04)
N#define CPU_PRCR ((ioport volatile unsigned*)0x1c05)
N
N
Nvoid ECG_Timer_Init(void);	   // function to Initialize the timer
N/*EOF*/
L 31 "../inc/ECGDemoNonBios.h" 2
N#include "psp_uart.h"
L 1 "..\inc\psp_uart.h" 1
N/******************************************************************************
N**File Name			:  psp_uart.h
N**File Description	:UART interface definition
N**Created By		:
N**Creation Date		:
N**Last modified date:
N**Remarks			:
N*******************************************************************************/
N
N#ifndef _PSP_UART_H_
N#define _PSP_UART_H_
N
N/* PSP Common header provides standard driver return codes, operation modes
N * and other generic defines required by all PSP drivers
N */
N#include "psp_common.h"
N#include "tistdtypes.h"
N
N
N/**
N * \defgroup PSPUartMain PSP UART Interface Definitions
N *
N * All interface definitions for PSP UART are contained in this file.
N *
N * @{
N */
N
N
N#define PSP_UART_MODULE_CLOCK 12000000u		/**< Input freq to UART module */
N
N/**
N *  \brief  default Init Config at Init time
N *
N *  Default value to be used by the driver if user does not give any specifications.
N */
N
N
N#define UARTMD_DEVPARAMS_DEFAULT 		\
N{      									\
N    PSP_OPMODE_INTERRUPT,				\
N    TRUE,								\
N	PSP_UART_MODULE_CLOCK,				\
N	NULL								\
N}
X#define UARTMD_DEVPARAMS_DEFAULT 		{      									    PSP_OPMODE_INTERRUPT,				    TRUE,									PSP_UART_MODULE_CLOCK,					NULL								}
N
N
N#define PSP_UART_EVENTQUEUE_USED   0
N/**< EDMA Event Queue used          */
N
N/* Mistral: Use Direct values for setting the baud rate instead of using enum values
N * Refer to UART documentation for supported baud rate values.
N */
N/**
N *  \brief  PSP Uart Baud Rate
N *
N *  Uart Baud rate
N */
Ntypedef enum
N{
N	PSP_UART_BAUD_RATE_0_3K     = 300,
N    PSP_UART_BAUD_RATE_1_2K     = 1200,
N    PSP_UART_BAUD_RATE_2_4K     = 2400,
N    PSP_UART_BAUD_RATE_4_8K     = 4800,
N    PSP_UART_BAUD_RATE_9_6K     = 9600,
N    PSP_UART_BAUD_RATE_19_2K    = 19200
N} PSP_UartBaudRate;
N
N/**
N *  \brief  PSP Uart Character Length
N *
N *  Uart Character length
N */
Ntypedef enum
N{
N    PSP_UART_CHARLEN_5  = 5,
N    PSP_UART_CHARLEN_6  = 6,
N    PSP_UART_CHARLEN_7  = 7,
N    PSP_UART_CHARLEN_8  = 8
N} PSP_UartCharLen;
N
N/**
N *  \brief  PSP Uart Parity
N *
N *  Uart Parity
N */
Ntypedef enum
N{
N    PSP_UART_PARITY_ODD     = 0,
N    PSP_UART_PARITY_EVEN    = 1,
N    PSP_UART_PARITY_NONE    = 2
N} PSP_UartParity;
N
N/**
N *  \brief  PSP Uart Stop Bits
N *
N *  Uart Stop Bits
N */
Ntypedef enum
N{
N    PSP_UART_NUM_STOP_BITS_1    = 0,
N    PSP_UART_NUM_STOP_BITS_1_5  = 1,
N    PSP_UART_NUM_STOP_BITS_2    = 2
N} PSP_UartNumStopBits;
N
N/**
N *  \brief  PSP Uart Flow Control
N *
N *  Uart Flow Control
N */
Ntypedef enum
N{
N    PSP_UART_FC_TYPE_NONE   = 0,    /**< No Flow Control */
N    PSP_UART_FC_TYPE_SW     = 1,    /**< Software Flow Control - NOT SUPPORTED IN THIS DRIVER */
N    PSP_UART_FC_TYPE_HW     = 2     /**< Hardware Flow Control */
N} PSP_UartFcType;
N
N/**
N *  \brief  PSP Uart Software Flow Control Param
N *
N *  Uart Software Flow Control Mode Param
N */
Ntypedef enum
N{
N    PSP_UART_SWFC_NONE          = 0,
N    PSP_UART_SWFC_XONXOFF_1     = 1,
N    PSP_UART_SWFC_XONXOFF_2     = 2,
N    PSP_UART_SWFC_XONXOFF_12    = 3
N} PSP_UartFcParam;
N
N/**
N *  \brief  PSP Uart Rx Trigger Level Param
N *
N *  Uart Receive Trigger Level Param
N */
Ntypedef enum
N{
N    PSP_UART_RX_TRIG1   = 1,    /**< Trigger Level 1*/
N    PSP_UART_RX_TRIG4   = 4,    /**< Trigger Level 4*/
N    PSP_UART_RX_TRIG8   = 8,    /**< Trigger Level 8*/
N    PSP_UART_RX_TRIG14  = 14    /**< Trigger Level 14*/
N}PSP_UartRxTrigLvl;
N/**
N *  \brief  PSP Uart Flow Control Configuration
N *
N *  Uart Software Flow Control Configuration
N */
Ntypedef struct
N{
N    PSP_UartFcType  fcType;         /**< Flow Control type */
N    PSP_UartFcParam fcParam;        /**< Flow Control param */
N} PSP_UartFlowControl;
N
N
N/**
N *  \brief  UART Configuration Parameters
N *
N *  UART Configuration Parameters that should be passed by the application
N *  during PSP_uartCreate
N *
N *  Note: Loopback is not supported in DMA mode. In case DMA mode is selected and
N *		   loopback is TRUE, driver will return an invalid parameter error code.
N */
Ntypedef struct
N{
N    PSP_OpMode          opmode;			/**< Mode of Operation Polled/Interrupt/DMA */
N    Uint32              baudRate;		/**< Baud Rate */
N    PSP_UartNumStopBits stopBits;		/**< Number of Stop Bits */
N    PSP_UartCharLen     charLen;		/**< Character Lenght */
N    PSP_UartParity      parity;			/**< Parity Settings */
N    PSP_UartFlowControl fc;				/**< Flow Control Enable/Disable */
N    PSP_UartRxTrigLvl   trigLvl;        /**< UART trigger level */
N    Bool                loopback;		/**< Enable or Disable the Loopback */
N    Bool                fifoEnable;		/**< FIFO Enable/Disable */
N} PSP_UartConfigParams;
N
N
N/**
N *  \brief  PSP Uart Ioctl commands
N *
N *  Uart Ioctl commands
N */
Ntypedef enum
N{
N    PSP_UART_IOCTL_GET_CONFIG_PARAMS = 0,	/**< Get UART config params, cmdArg = struct* UartConfigParams */
N    PSP_UART_IOCTL_SET_BAUD,				/**< Set baud rate, cmdArg= struct* UartBaudRate */
N    PSP_UART_IOCTL_SET_STOPBITS,			/**< Set number of stop bits, cmdArg = sturct* UartNumStopBits */
N    PSP_UART_IOCTL_SET_DATABITS,			/**< Set number of Data bits, cmdArg = sturct* UartCharLen*/
N    PSP_UART_IOCTL_SET_PARITY,				/**< Set parity type, cmdArg = sturct* UartParity */
N    PSP_UART_IOCTL_SET_FLOWCONTROL,			/**< Set flowcontrol, cmdArg = sturct* UartFlowControl*/
N	PSP_UART_IOCTL_SET_TRIGGER_LEVEL,		/**< Changing Trigger level  */
N	PSP_UART_IOCTL_SET_RX_FIFO_CLEAR,		/**< Clears the RX  FIFO    	*/
N	PSP_UART_IOCTL_SET_TX_FIFO_CLEAR,		/**< Clears the  TX FIFO    	*/
N	PSP_UART_IOCTL_CANCEL_IO,				/**< Cancel IO           */
N	PSP_UART_IOCTL_GET_STATS,				/**< Getting the Uart stats for DDC  */
N	PSP_UART_IOCTL_CLEAR_STATS,     		/**< Clearing the Stats of DDC */
N	PSP_UART_IOCTL_SET_FIFO_MODE,			/**< Enabling/Disabling the FIFO */
N    PSP_UART_IOCTL_SET_LOOPBACK,			/**< Set loopback, cmdArg=Bool */
N    PSP_UART_IOCTL_SET_OPMODE,				/**< Set operation mode, cmdArg= struct* IOM_OpMode */
N	PSP_UART_IOCTL_MAX_IOCTL				/**< Book-keep - Max ioctl's */
N} PSP_UartIoctlCmd;
N
N
N
N/**
N * \brief UART DDC Statistics Collection Object
N *
N * Statistics are collected on a per-controller basis for UART. Hence, an
N * object of this type is found embedded in the DDC_UartObj structure.
N */
Ntypedef struct
N{
N    Uint32  rxBytes;
N    /**< Number bytes received                                              */
N    Uint32  txBytes;
N    /**< Number bytes transmitted                                           */
N    Uint32  errors;
N    /**< Number of times an error was detected                              */
N    Uint32  nefc;
N    /**< Number of times a near-end flow control was initiated              */
N    Uint32  fefc;
N    /**< Number of times a far-end flow control was initiated               */
N	Uint32  overrun;
N    /**< Number of overrun errors                                           */
N	Uint32  rxTimeout;
N    /**< Number of Rx timeouts                                              */
N} PSP_UartStats;
N
N
N/**
N *  \brief  PSP Uart Config (at Init/Create time)
N *
N *  UART Configuration structure pointer passed to PSP_uartCreate function
N */
Ntypedef struct
N{
N    Uint32              inputFreq;          /**< Input Frequency in MHZ */
N    Uint32              heapSegId;          /**< *** NOT USE IN THIS DRIVER VERSION *** */
N    Bool                syncMode;           /**< Sync mode enabled? */
N    PSP_AppCallback     appCallback;        /**< Application Callback function when in Async mode - Can be called in interrupt context */
N    Ptr                 cbkContext;         /**< Application Callback context when in Async mode*/
N    PSP_OpMode          opMode;             /**< Operational mode of the UART instance - polled/interrupt/dma - DMA NOT SUPPORTED */
N    Ptr                 hEdma;              /**< Dma handle for UART transfer */
N} PSP_UartConfig;
N
N
N/**
N *  \brief  Init Config at Init time
N *
N *  UART Configuration structure pointer passed to mdBindDev function.
N */
Ntypedef struct
N{
N    PSP_OpMode	opMode;			/**< Opmode defined at init (eg. POLLED mode)*/
N    Bool        syncMode;   	/**< Sync mode enabled 						 */
N	Uint32 		inputFrequency; /**< Default input frequency of the uart	 */
N	Ptr         hEdma;			/**< EDMA Handle							 */
N} UARTMD_DevParams;
N
N
N
N/**
N *  \brief  IOM_PACKET element structure
N *
N *  Structure for the uart specific buffer address to be passed to the GIO.
N */
Ntypedef struct
N{
N	char * addr;               /**< Address of the data buffer */
N	Uint32 timeout;            /**< Timeout variable */
N} PSP_uartDataParam;
N
N
N/**
N *  \brief Create (and initialize) a given UART driver (instance)
N *
N *  After the instance is "created", DDA calls the initialization function
N *  on the DDC to initialize the instance of the device. Typically, software
N *  bookkeeping functions are performed in this call. Memory for device
N *  instance specific data structures may be allocated and initialized.
N *  Configuration information may be passed in the call and initialization
N *  based upon this information is done.
N *
N *  \param  instNum [IN]        Uart instance number
N *  \param  param [IN/OUT]      PSP_UartConfigParams structure pointer
N *  \return PSP_SOK or PSP Error code
N */
NPSP_Result  PSP_uartCreate(Uint32 instNum, Ptr param);
N
N/**
N *  \brief Delete a given UART driver (instance)
N *
N *  The instance of UART is deleted in softare. Memory allocated in Create
N *  funciton is released
N *
N *  \param  instNum [IN]        Uart instance number
N *  \param  param [IN/OUT]      Extra parameter (implementation specific)
N *  \return PSP_SOK or PSP Error code
N */
NPSP_Result  PSP_uartDelete(Uint32 instNum, Ptr param);
N
N/**
N *  \brief Open Instance of the UART device
N *
N *  This function prepares the device hardware for data transfers and
N *  usage by the upper layer driver software. The DDA layer is expected
N *  to install the ISR and enable it only after the completion of this
N *  call. The function prepares the driver instance for data transfers
N *  and returns an handle to the driver instance.
N *
N *  \param  instNum [IN]        Uart instance number
N *  \param  param [IN/OUT]      Extra parameter (implementation specific)
N *  \return PSP_Handle          Uart Driver Instance Handle
N */
NPSP_Handle  PSP_uartOpen(Uint32 instNum, Ptr param);
N
N/**
N *  \brief Close Instance of the UART device
N *
N *  This function closes the device for data transfers and usage by the
N *  upper layer driver software. The hardware is programmed to stop/abort data
N *  transfer (depending upon the type of device and its specifics) and the
N *  device ISR is "disabled" by the upper layer driver software after the
N *  completion of this call. After the successful completion of this call, the
N *  device cannot perform any data IO.
N *
N *  \param  hUart [IN]          UART Driver Instance Handle
N *  \param  param [IN/OUT]      Extra parameter (implementation specific)
N *  \return PSP_SOK or PSP Error code
N */
NPSP_Result  PSP_uartClose(PSP_Handle hUart, Ptr param);
N
N/**
N *  \brief Read data from UART Instance
N *
N *  This function reads data from the UART Instance.
N *
N *  \param  hUart [IN]          UART Driver Instance Handle
N *  \param  buf [IN/OUT]        Character Buffer pointer - where read data is to be stored
N *  \param  size [IN]           Size of character buffer - number of bytes to read
N *  \param  timeout [IN]        NOT SUPPORTED - Timeout (typically in milliSeconds)
N *  \param  param [IN/OUT]      Extra parameter (implementation specific)
N *  \return If successful, returns positive number - number of bytes read
N *          In Async mode, if number of bytes read is less than size, the request
N *          is pending and in progress.
N *          If failed, returns negative number - PSP Error code
N */
NInt  PSP_uartRead(PSP_Handle hUart, const Char *buf, Int size, Int timeout, Ptr param);
N
N/**
N *  \brief Write data to UART Instance
N *
N *  This function writes data to the UART Instance.
N *
N *  \param  hUart [IN]          UART Driver Instance Handle
N *  \param  buf [IN]            Character Buffer pointer - data to be written
N *  \param  size [IN]           Size of character buffer - number of bytes to write
N *  \param  timeout [IN]        NOT SUPPORTED - Timeout (typically in milliSeconds)
N *  \param  param [IN/OUT]      Extra parameter (implementation specific)
N *  \return If successful, returns positive number - number of bytes written
N *          In Async mode, if number of bytes written is less than size, the request
N *          is pending and in progress.
N *          If failed, returns negative number - PSP Error code
N */
NInt  PSP_uartWrite(PSP_Handle hUart, const Char *buf, Int size, Int timeout, Ptr param );
N
N
N/**
N *  \brief UART Ioctl
N *
N *  This function provides ioctl functionality for UART
N *
N *  \param  hUart [IN]          UART Driver Instance Handle
N *  \param  cmd [IN]            Operation to be performed, typically an enum gets passed
N *  \param  cmdArg [IN/OUT]     Provides additonal information related to the operation
N *  \param  param [IN/OUT]      Device/Cmd specific argument
N *  \return PSP_SOK or PSP Error code
N */
NPSP_Result  PSP_uartIoctl(PSP_Handle hUart, PSP_UartIoctlCmd cmd, Ptr cmdArg, Ptr param);
N
N/**
N *  \brief UART initialization function
N *
N *  This function creates the function pointer to the DDC Layer
N * and sets the parameters for the internal buffer
N * and initializes the DDC
N *
N *  \param  instNum [IN]      UART Driver Instance Number
N *  \param  param [IN/OUT]    Extra parameter(Implementation specific)
N *  \return Handle to the UART instance
N */
NPSP_Handle dda_uartInit(Uint32 instNum, Ptr param);
N
N/**
N *  \brief Write data to UART Instance
N *
N *  This function writes data to the UART Instance.
N *
N *  \param  hUart [IN]          UART Driver Instance Handle
N *  \param  buf [IN]            Character Buffer pointer - data to be written
N *  \param  size [IN]           Size of character buffer - number of bytes to write
N *  \param  timeout [IN]        NOT SUPPORTED - Timeout (typically in milliSeconds)
N *  \param  param [IN/OUT]      Extra parameter (implementation specific)
N *  \return If successful, returns positive number - number of bytes written
N *          If failed, returns negative number - PSP Error code
N */
NInt dda_uartWrite(PSP_Handle hUart, const Char *buf, Int size, Int timeout, Ptr param);
N
N
N/**
N *  \brief Read data from UART Instance
N *
N *  This function reads data from the UART Instance.
N *
N *  \param  hUart [IN]          UART Driver Instance Handle
N *  \param  buf [IN/OUT]        Character Buffer pointer - where read data is to be stored
N *  \param  size [IN]           Size of character buffer - number of bytes to read
N *  \param  timeout [IN]        NOT SUPPORTED - Timeout (typically in milliSeconds)
N *  \param  param [IN/OUT]      Extra parameter (implementation specific)
N *  \return If successful, returns positive number - number of bytes read
N *          If failed, returns negative number - PSP Error code
N */
NInt dda_uartRead(PSP_Handle hUart, const Char *buf, Int size, Int timeout, Ptr param);
N
N/*@} PSPUartMain */
N#endif  /* _PSP_UART_H_ */
N/*EOF*/
L 32 "../inc/ECGDemoNonBios.h" 2
N
N
N#define QRS_MAX_BUFF_SIZE (3 * 500)
N
N
N
N/* Function to toggle GPIO according to channel*/
Nvoid ECG_ToggleGPIO(Uint8);
N/* function to process samples for 1 sec*/	
NECG_Status ECG_SubSystem();	
N/* function to read channel data*/	 		
Nextern void ADS1258_Read_ChannelData(Uint8*,Uint8*);
N/* convert the 4 byte channel data to 24bit sample*/
Nextern Int32 ECG_GetCurrAqsnSample(Uint8*);
N/* process the current sample*/
Nextern void ECG_ProcessCurrSample(Int16 *CurrAqsSample, Int16 *FilteredOut);
N/* function which is called by the main thread at filling of every(N/3) */
N/* samples in Lead_Info_Buffer                                          */
Nextern void QRS_Algorithm_Interface(Int16);
N/* plot the current sample in LCD*/
Nextern void ECG_LCDPlotCurrSample(Int16 * , Uint16);
N/* Send the LeadInfo thru UART to plot in the PC Application*/
Nextern void ECG_UARTTxCurrSamples(Int16 *);
N/* Transmission of the curret set of sample thru UART*/
Nextern void ECG_UARTTxCurrSamples_debug(Int16 *);
N/*Function to display the HR Rate on LCD*/
Nvoid ECG_LCD_HRDisplay(void);
N/*Function to display the Lead Off status on LCD*/
Nvoid ECG_LCD_LeadOffDisplay(void);
N/* function to initialize peripherals*/
Nextern ECG_Status ECG_C5505Init();	
N/* function to display LCD startup screen*/
Nextern void ECG_DisplayLCDStartUpScreen();
N/* function to Initialize the timer*/
Nextern void ECG_TIMER_INIT();
N
N#endif
N/*EOF*/
N
L 11 "../src/ECGSystem_main.c" 2
N#include "lcd.h"
L 1 "../inc/lcd.h" 1
N/******************************************************************************
N**File Name			: lcd.h
N**File Description	:The file declares all the functions defined in the lcd.c
N**Created By		:
N**Creation Date		:
N**Last modified date:
N**Remarks			:
N*******************************************************************************/
N
N/*------------------------------------------------------------------------**
N** File Name : lcd.h                                                      **
N** Description : The file declares all the functions defined in the lcd.c **
N**------------------------------------------------------------------------*/
N
N#ifndef _LCD_H
N#define _LCD_H
N
N#include "psp_common.h"
N#include "csl_ioport.h"
N#include "corazon.h"
N#include "cslr.h"
N#include "LCD_FontTable.h"
N
N#define DISPLAY_WIDTH           128
N#define DISPLAY_HEIGHT          128
N
N//#define LCD_HEIGHT   128
N//#define LCD_WIDTH    128
N
N#define WHITE 0x3F3F3F
N#define BLACK 0
N#define RED 0x3F0000
N#define GREEN 0x003F00
N#define BLUE 0x00003F
N
N#define UP_KEY 				0x0001
N#define DOWN_KEY 			0x0002
N#define ENTER_KEY			0x0004
N#define PREVIOUS_KEY		0x0008
N
N#define ECG_FE				0x0003
N#define STETHO_FE			0x0002
N#define PULSE_OXY_FE		0x0001
N#define INCREMENT_LEAD		+1
N#define DECREMENT_LEAD		-1
N#define WAVEFORM_START_ROW	44
N#define WAVEFORM_END_ROW	128
N#define WAVEFORM_START_COL	0
N#define WAVEFORM_END_COL	128
N
Nvoid Init_LCD(void);
Nvoid draw_string (Uint16 , Uint16 , char* ,Uint32 );
Nvoid draw_font(Uint16, Uint16, Uint16,Uint32);
NInt32 drawLine( Uint16 , Uint16, Uint16, Uint16, Uint16, Uint16);
Nvoid  cmdWrite(Uint16);
Nvoid  dataWrite(Uint16);
N//void LCD_hr_status_update(void);
Nvoid LCD_clear_window(Uint16,Uint16,Uint16,Uint16);
Nvoid ECG_DisplayLCDStartUpScreen();
N/*	function give wait cycles*/
Nextern void wait(Uint32);
Nvoid draw_font_HR(Uint16, Uint16, Uint16,Uint32);
Nvoid draw_font_Lead(Uint16, Uint16, Uint16,Uint32);
N#endif
N/*EOF*/
L 12 "../src/ECGSystem_main.c" 2
N#include "SAR.h"/*remove with keypad*/
L 1 "../inc/SAR.h" 1
N/******************************************************************************
N**File Name			: sar.h
N**File Description	:Sar register and button
N**Created By		:
N**Creation Date		:
N**Last modified date:
N**Remarks			:
N*******************************************************************************/
N
N#include "psp_common.h"
N
N
Nvoid Init_SAR(void);		// function to Init SAR
NUint16 Get_Sar_Key(void);	// function to read the value of the current key
N
N
N/*  SAR Register Definitions*/
N#define SARCTRL   ((ioport volatile unsigned*)0x7012)
N#define SARDATA   ((ioport volatile unsigned*)0x7014)
N#define SARCLKCTRL   ((ioport volatile unsigned*)0x7016)
N#define SARPINCTRL   ((ioport volatile unsigned*)0x7018)
N#define SARGPOCTRL   ((ioport volatile unsigned*)0x701A)
N
N/*  Values corresponding to each keys*/
N#define SW6  0x236
N#define SW7  0
N#define SW8  0xd6
N#define SW9  0x166
N#define SW10 0x1d6
N#define SW11 0x215
N#define SW12 0x257
N#define SW13 0x2c7
N#define SW14 0x316
N#define SW15 0x274
N#define SW16 0x3FB
N/*EOF*/
L 13 "../src/ECGSystem_main.c" 2
N#include "string.h"
L 1 "C:/ti/ccsv6/tools/compiler/c5500_4.4.1/include/string.h" 1
N/*****************************************************************************/
N/* string.h   v4.4.1                                                         */
N/*                                                                           */
N/* Copyright (c) 1993-2012 Texas Instruments Incorporated                    */
N/* http://www.ti.com/                                                        */
N/*                                                                           */
N/*  Redistribution and  use in source  and binary forms, with  or without    */
N/*  modification,  are permitted provided  that the  following conditions    */
N/*  are met:                                                                 */
N/*                                                                           */
N/*     Redistributions  of source  code must  retain the  above copyright    */
N/*     notice, this list of conditions and the following disclaimer.         */
N/*                                                                           */
N/*     Redistributions in binary form  must reproduce the above copyright    */
N/*     notice, this  list of conditions  and the following  disclaimer in    */
N/*     the  documentation  and/or   other  materials  provided  with  the    */
N/*     distribution.                                                         */
N/*                                                                           */
N/*     Neither the  name of Texas Instruments Incorporated  nor the names    */
N/*     of its  contributors may  be used to  endorse or  promote products    */
N/*     derived  from   this  software  without   specific  prior  written    */
N/*     permission.                                                           */
N/*                                                                           */
N/*  THIS SOFTWARE  IS PROVIDED BY THE COPYRIGHT  HOLDERS AND CONTRIBUTORS    */
N/*  "AS IS"  AND ANY  EXPRESS OR IMPLIED  WARRANTIES, INCLUDING,  BUT NOT    */
N/*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR    */
N/*  A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT    */
N/*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,    */
N/*  SPECIAL,  EXEMPLARY,  OR CONSEQUENTIAL  DAMAGES  (INCLUDING, BUT  NOT    */
N/*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,    */
N/*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY    */
N/*  THEORY OF  LIABILITY, WHETHER IN CONTRACT, STRICT  LIABILITY, OR TORT    */
N/*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE    */
N/*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.     */
N/*                                                                           */
N/*****************************************************************************/
N
N#ifndef _STRING
N#define _STRING
N
N#ifdef __cplusplus
S//----------------------------------------------------------------------------
S// <cstring> IS RECOMMENDED OVER <string.h>.  <string.h> IS PROVIDED FOR
S// COMPATIBILITY WITH C AND THIS USAGE IS DEPRECATED IN C++
S//----------------------------------------------------------------------------
Sextern "C" namespace std
S{
N#endif /* __cplusplus */
N 
N#ifndef NULL
S#define NULL 0
N#endif
N
N#ifndef _SIZE_T
S#define _SIZE_T
Stypedef __SIZE_T_TYPE__ size_t;
N#endif
N
N#include <linkage.h>
N
N#define _OPT_IDECL	_IDECL
N
N_OPT_IDECL size_t  strlen(const char *_string);
X size_t  strlen(const char *_string);
N
N_OPT_IDECL char *strcpy(char *_dest, const char *_src);
X char *strcpy(char *_dest, const char *_src);
N_OPT_IDECL char *strncpy(char *_to, const char *_from, size_t _n);
X char *strncpy(char *_to, const char *_from, size_t _n);
N_OPT_IDECL char *strcat(char *_string1, const char *_string2);
X char *strcat(char *_string1, const char *_string2);
N_OPT_IDECL char *strncat(char *_to, const char *_from, size_t _n);
X char *strncat(char *_to, const char *_from, size_t _n);
N_OPT_IDECL char *strchr(const char *_string, int _c);
X char *strchr(const char *_string, int _c);
N_OPT_IDECL char *strrchr(const char *_string, int _c);
X char *strrchr(const char *_string, int _c);
N
N_OPT_IDECL int  strcmp(const char *_string1, const char *_string2);
X int  strcmp(const char *_string1, const char *_string2);
N_OPT_IDECL int  strncmp(const char *_string1, const char *_string2, size_t _n);
X int  strncmp(const char *_string1, const char *_string2, size_t _n);
N
N_CODE_ACCESS int     strcoll(const char *_string1, const char *_string2);
X int     strcoll(const char *_string1, const char *_string2);
N_CODE_ACCESS size_t  strxfrm(char *_to, const char *_from, size_t _n);
X size_t  strxfrm(char *_to, const char *_from, size_t _n);
N_CODE_ACCESS char   *strpbrk(const char *_string, const char *_chs);
X char   *strpbrk(const char *_string, const char *_chs);
N_CODE_ACCESS size_t  strspn(const char *_string, const char *_chs);
X size_t  strspn(const char *_string, const char *_chs);
N_CODE_ACCESS size_t  strcspn(const char *_string, const char *_chs);
X size_t  strcspn(const char *_string, const char *_chs);
N_CODE_ACCESS char   *strstr(const char *_string1, const char *_string2);
X char   *strstr(const char *_string1, const char *_string2);
N_CODE_ACCESS char   *strtok(char *_str1, const char *_str2);
X char   *strtok(char *_str1, const char *_str2);
N_CODE_ACCESS char   *strerror(int _errno);
X char   *strerror(int _errno);
N
N_CODE_ACCESS void   *memmove(void *_s1, const void *_s2, size_t _n);
X void   *memmove(void *_s1, const void *_s2, size_t _n);
N_CODE_ACCESS void   *memcpy(void *_s1, const void *_s2, size_t _n);
X void   *memcpy(void *_s1, const void *_s2, size_t _n);
N
N_OPT_IDECL int     memcmp(const void *_cs, const void *_ct, size_t _n);
X int     memcmp(const void *_cs, const void *_ct, size_t _n);
N_OPT_IDECL void   *memchr(const void *_cs, int _c, size_t _n);
X void   *memchr(const void *_cs, int _c, size_t _n);
N
N_OPT_IDECL   void   *memset(void *_mem, int _ch, size_t _n);
X   void   *memset(void *_mem, int _ch, size_t _n);
N
N
N#ifdef __cplusplus
S} /* extern "C" namespace std */
N#endif /* __cplusplus */
N
N#if defined(_INLINE) || defined(_STRING_IMPLEMENTATION)
X#if 0L || 0L
S
S
S#ifdef __cplusplus
Snamespace std {
S#endif
S
S#define _OPT_IDEFN	_IDEFN
S
S#if defined(_INLINE) || defined(_STRLEN)
S_OPT_IDEFN size_t strlen(const char *string)
S{
S   size_t      n = (size_t)-1;
S   const char *s = string;
S
S   do n++; while (*s++);
S   return n;
S}
S#endif /* _INLINE || _STRLEN */
S
S#if defined(_INLINE) || defined(_STRCPY)
S_OPT_IDEFN char *strcpy(register char *dest, register const char *src)
S{
S     register char       *d = dest;     
S     register const char *s = src;
S
S     while (*d++ = *s++);
S     return dest;
S}
S#endif /* _INLINE || _STRCPY */
S
S#if defined(_INLINE) || defined(_STRNCPY)
S_OPT_IDEFN char *strncpy(register char *dest,
S		     register const char *src,
S		     register size_t n)
S{
S     if (n) 
S     {
S	 register char       *d = dest;
S	 register const char *s = src;
S	 while ((*d++ = *s++) && --n);              /* COPY STRING         */
S	 if (n-- > 1) do *d++ = '\0'; while (--n);  /* TERMINATION PADDING */
S     }
S     return dest;
S}
S#endif /* _INLINE || _STRNCPY  */
S
S#if defined(_INLINE) || defined(_STRCAT)
S_OPT_IDEFN char *strcat(char *string1, const char *string2)
S{
S   char       *s1 = string1;
S   const char *s2 = string2;
S
S   while (*s1) s1++;		     /* FIND END OF STRING   */
S   while (*s1++ = *s2++);	     /* APPEND SECOND STRING */
S   return string1;
S}
S#endif /* _INLINE || _STRCAT */
S
S#if defined(_INLINE) || defined(_STRNCAT)
S_OPT_IDEFN char *strncat(char *dest, const char *src, register size_t n)
S{
S    if (n)
S    {
S	char       *d = dest;
S	const char *s = src;
S
S	while (*d) d++;                      /* FIND END OF STRING   */
S
S	while (n--)
S	  if (!(*d++ = *s++)) return dest; /* APPEND SECOND STRING */
S	*d = 0;
S    }
S    return dest;
S}
S#endif /* _INLINE || _STRNCAT */
S
S#if defined(_INLINE) || defined(_STRCHR)
S_OPT_IDEFN char *strchr(const char *string, int c)
S{
S   char        tch, ch  = c;
S   const char *s        = string;
S
S   for (;;)
S   {
S       if ((tch = *s) == ch) return (char *) s;
S       if (!tch)             return (char *) 0;
S       s++;
S   }
S}
S#endif /* _INLINE || _STRCHR */
S
S#if defined(_INLINE) || defined(_STRRCHR)
S_OPT_IDEFN char *strrchr(const char *string, int c)
S{
S   char        tch, ch = c;
S   char       *result  = 0;
S   const char *s       = string;
S
S   for (;;)
S   {
S      if ((tch = *s) == ch) result = (char *) s;
S      if (!tch) break;
S      s++;
S   }
S
S   return result;
S}
S#endif /* _INLINE || _STRRCHR */
S
S#if defined(_INLINE) || defined(_STRCMP)
S_OPT_IDEFN int strcmp(register const char *string1,
S		  register const char *string2)
S{
S   register int c1, res;
S
S   for (;;)
S   {
S       c1  = (unsigned char)*string1++;
S       res = c1 - (unsigned char)*string2++;
S
S       if (c1 == 0 || res != 0) break;
S   }
S
S   return res;
S}
S#endif /* _INLINE || _STRCMP */
S
S#if defined(_INLINE) || defined(_STRNCMP)
S_OPT_IDEFN int strncmp(const char *string1, const char *string2, size_t n)
S{
S     if (n) 
S     {
S	 const char *s1 = string1;
S	 const char *s2 = string2;
S	 unsigned char cp;
S	 int         result;
S
S	 do 
S	    if (result = (unsigned char)*s1++ - (cp = (unsigned char)*s2++))
S                return result;
S	 while (cp && --n);
S     }
S     return 0;
S}
S#endif /* _INLINE || _STRNCMP */
S
S#if defined(_INLINE) || defined(_MEMCMP)
S_OPT_IDEFN int memcmp(const void *cs, const void *ct, size_t n)
S{
S   if (n) 
S   {
S       const unsigned char *mem1 = (unsigned char *)cs;
S       const unsigned char *mem2 = (unsigned char *)ct;
S       int                 cp1, cp2;
S
S       while ((cp1 = *mem1++) == (cp2 = *mem2++) && --n);
S       return cp1 - cp2;
S   }
S   return 0;
S}
S#endif /* _INLINE || _MEMCMP */
S
S#if defined(_INLINE) || defined(_MEMCHR)
S_OPT_IDEFN void *memchr(const void *cs, int c, size_t n)
S{
S   if (n)
S   {
S      const unsigned char *mem = (unsigned char *)cs;   
S      unsigned char        ch  = c;
S
S      do 
S         if ( *mem == ch ) return (void *)mem;
S         else mem++;
S      while (--n);
S   }
S   return NULL;
S}
S#endif /* _INLINE || _MEMCHR */
S
S#if ((defined(_INLINE) || defined(_MEMSET)) && !defined(_TMS320C6X)) && !defined(__TMS470__) && !defined(__ARP32__)
S_OPT_IDEFN void *memset(void *mem, register int ch, register size_t length)
S{
S     register char *m = (char *)mem;
S
S     while (length--) *m++ = ch;
S     return mem;
S}
S#endif /* _INLINE || _MEMSET */
S
S#ifdef __cplusplus
S} /* namespace std */
S#endif
S
S
N#endif /* (_INLINE || _STRING_IMPLEMENTATION) */
N
N#endif /* ! _STRING */
N
N#if defined(__cplusplus) && !defined(_CPP_STYLE_HEADER)
X#if 0L && !0L
Susing std::size_t;
Susing std::strlen;
Susing std::strcpy;
Susing std::strncpy;
Susing std::strcat;
Susing std::strncat;
Susing std::strchr;
Susing std::strrchr;
Susing std::strcmp;
Susing std::strncmp;
Susing std::strcoll;
Susing std::strxfrm;
Susing std::strpbrk;
Susing std::strspn;
Susing std::strcspn;
Susing std::strstr;
Susing std::strtok;
Susing std::strerror;
Susing std::memmove;
Susing std::memcpy;
Susing std::memcmp;
Susing std::memchr;
Susing std::memset;
S
S
N#endif /* _CPP_STYLE_HEADER */
N
L 14 "../src/ECGSystem_main.c" 2
N#include "stdio.h"
L 1 "C:/ti/ccsv6/tools/compiler/c5500_4.4.1/include/stdio.h" 1
N/*****************************************************************************/
N/* STDIO.H v4.4.1                                                            */
N/*                                                                           */
N/* Copyright (c) 1993-2012 Texas Instruments Incorporated                    */
N/* http://www.ti.com/                                                        */
N/*                                                                           */
N/*  Redistribution and  use in source  and binary forms, with  or without    */
N/*  modification,  are permitted provided  that the  following conditions    */
N/*  are met:                                                                 */
N/*                                                                           */
N/*     Redistributions  of source  code must  retain the  above copyright    */
N/*     notice, this list of conditions and the following disclaimer.         */
N/*                                                                           */
N/*     Redistributions in binary form  must reproduce the above copyright    */
N/*     notice, this  list of conditions  and the following  disclaimer in    */
N/*     the  documentation  and/or   other  materials  provided  with  the    */
N/*     distribution.                                                         */
N/*                                                                           */
N/*     Neither the  name of Texas Instruments Incorporated  nor the names    */
N/*     of its  contributors may  be used to  endorse or  promote products    */
N/*     derived  from   this  software  without   specific  prior  written    */
N/*     permission.                                                           */
N/*                                                                           */
N/*  THIS SOFTWARE  IS PROVIDED BY THE COPYRIGHT  HOLDERS AND CONTRIBUTORS    */
N/*  "AS IS"  AND ANY  EXPRESS OR IMPLIED  WARRANTIES, INCLUDING,  BUT NOT    */
N/*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR    */
N/*  A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT    */
N/*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,    */
N/*  SPECIAL,  EXEMPLARY,  OR CONSEQUENTIAL  DAMAGES  (INCLUDING, BUT  NOT    */
N/*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,    */
N/*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY    */
N/*  THEORY OF  LIABILITY, WHETHER IN CONTRACT, STRICT  LIABILITY, OR TORT    */
N/*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE    */
N/*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.     */
N/*                                                                           */
N/*****************************************************************************/
N#ifndef _STDIO 
S#define _STDIO
S
S#include <linkage.h>
S#include <stdarg.h>
S
S/*---------------------------------------------------------------------------*/
S/* Attributes are only available in relaxed ANSI mode.                       */
S/*---------------------------------------------------------------------------*/
S#ifndef __ATTRIBUTE
S#if __TI_STRICT_ANSI_MODE__
S#define __ATTRIBUTE(attr)
S#else
S#define __ATTRIBUTE(attr) __attribute__(attr)
S#endif
S#endif
S
S
S#ifdef __cplusplus
S//----------------------------------------------------------------------------
S// <cstdio> IS RECOMMENDED OVER <stdio.h>.  <stdio.h> IS PROVIDED FOR
S// COMPATIBILITY WITH C AND THIS USAGE IS DEPRECATED IN C++
S//----------------------------------------------------------------------------
Sextern "C" namespace std {
S#endif
S
S/****************************************************************************/
S/* TYPES THAT ANSI REQUIRES TO BE DEFINED                                   */
S/****************************************************************************/
S#ifndef _SIZE_T
S#define _SIZE_T
Stypedef __SIZE_T_TYPE__ size_t;
S#endif
S
Stypedef struct {
S      int fd;                    /* File descriptor */
S      unsigned char* buf;        /* Pointer to start of buffer */
S      unsigned char* pos;        /* Position in buffer */
S      unsigned char* bufend;     /* Pointer to end of buffer */
S      unsigned char* buff_stop;  /* Pointer to last read char in buffer */
S      unsigned int   flags;      /* File status flags (see below) */
S} FILE;
S
S#ifndef _FPOS_T
S#define _FPOS_T
Stypedef long fpos_t;
S#endif /* _FPOS_T */
S
S/****************************************************************************/
S/* DEVICE AND STREAM RELATED MACROS                                         */
S/****************************************************************************/
S/****************************************************************************/
S/* MACROS THAT DEFINE AND USE FILE STATUS FLAGS                             */
S/****************************************************************************/
S
S#define _IOFBF       0x0001
S#define _IOLBF       0x0002
S#define _IONBF       0x0004
S#define _BUFFALOC    0x0008
S#define _MODER       0x0010
S#define _MODEW       0x0020
S#define _MODERW      0x0040
S#define _MODEA       0x0080
S#define _MODEBIN     0x0100
S#define _STATEOF     0x0200
S#define _STATERR     0x0400
S#define _UNGETC      0x0800
S#define _TMPFILE     0x1000
S
S#define _SET(_fp, _b)      (((_fp)->flags) |= (_b))
S#define _UNSET(_fp, _b)    (((_fp)->flags) &= ~(_b))
S#define _STCHK(_fp, _b)    (((_fp)->flags) & (_b))
S#define _BUFFMODE(_fp)     (((_fp)->flags) & (_IOFBF | _IOLBF | _IONBF))
S#define _ACCMODE(_fp)      (((_fp)->flags) & (_MODER | _MODEW))
S
S/****************************************************************************/
S/* MACROS THAT ANSI REQUIRES TO BE DEFINED                                  */
S/****************************************************************************/
S#define BUFSIZ          256 
S
S#define FOPEN_MAX       _NFILE
S#define FILENAME_MAX    256  
S#define TMP_MAX         65535
S
S#define stdin     (&_ftable[0])      
S#define stdout    (&_ftable[1])
S#define stderr    (&_ftable[2])
S
S#define L_tmpnam  _LTMPNAM
S
S
S#define SEEK_SET  (0x0000)
S#define SEEK_CUR  (0x0001)
S#define SEEK_END  (0x0002)
S
S#ifndef NULL
S#define NULL 0
S#endif
S
S#ifndef EOF
S#define EOF    (-1)
S#endif
S
S/******** END OF ANSI MACROS ************************************************/
S
S#define P_tmpdir        ""                   /* Path for temp files         */
S
S/****************************************************************************/
S/* DEVICE AND STREAM RELATED DATA STRUCTURES AND MACROS                     */
S/****************************************************************************/
S#define _NFILE           10                   /* Max number of files open   */
S#define _LTMPNAM         16                   /* Length of temp name        */
S
Sextern _DATA_ACCESS FILE _ftable[_NFILE];
Sextern _DATA_ACCESS char _tmpnams[_NFILE][_LTMPNAM];
S
S/****************************************************************************/
S/*   FUNCTION DEFINITIONS  - ANSI                                           */
S/****************************************************************************/
S/****************************************************************************/
S/* OPERATIONS ON FILES                                                      */
S/****************************************************************************/
Sextern _CODE_ACCESS int     remove(const char *_file);
Sextern _CODE_ACCESS int     rename(const char *_old, const char *_new);
Sextern _CODE_ACCESS FILE   *tmpfile(void);
Sextern _CODE_ACCESS char   *tmpnam(char *_s);
S
S/****************************************************************************/
S/* FILE ACCESS FUNCTIONS                                                    */
S/****************************************************************************/
Sextern _CODE_ACCESS int     fclose(FILE *_fp); 
Sextern _CODE_ACCESS FILE   *fopen(const char *_fname, const char *_mode);
Sextern _CODE_ACCESS FILE   *freopen(const char *_fname, const char *_mode,
S			            register FILE *_fp);
Sextern _CODE_ACCESS void    setbuf(register FILE *_fp, char *_buf);
Sextern _CODE_ACCESS int     setvbuf(register FILE *_fp, register char *_buf, 
S			            register int _type, register size_t _size);
Sextern _CODE_ACCESS int     fflush(register FILE *_fp); 
S
S/****************************************************************************/
S/* FORMATTED INPUT/OUTPUT FUNCTIONS                                         */
S/****************************************************************************/
Sextern _CODE_ACCESS int fprintf(FILE *_fp, const char *_format, ...)
S               __ATTRIBUTE ((__format__ (__printf__, 2, 3)));
Sextern _CODE_ACCESS int fscanf(FILE *_fp, const char *_fmt, ...)
S               __ATTRIBUTE ((__format__ (__scanf__, 2, 3)));
Sextern _CODE_ACCESS int printf(const char *_format, ...)
S               __ATTRIBUTE ((__format__ (__printf__, 1, 2)));
Sextern _CODE_ACCESS int scanf(const char *_fmt, ...)
S               __ATTRIBUTE ((__format__ (__scanf__, 1, 2)));
Sextern _CODE_ACCESS int sprintf(char *_string, const char *_format, ...)
S               __ATTRIBUTE ((__format__ (__printf__, 2, 3)));
Sextern _CODE_ACCESS int snprintf(char *_string, size_t _n, 
S				 const char *_format, ...)
S               __ATTRIBUTE ((__format__ (__printf__, 3, 4)));
Sextern _CODE_ACCESS int sscanf(const char *_str, const char *_fmt, ...)
S               __ATTRIBUTE ((__format__ (__scanf__, 2, 3)));
Sextern _CODE_ACCESS int vfprintf(FILE *_fp, const char *_format, va_list _ap)
S               __ATTRIBUTE ((__format__ (__printf__, 2, 0)));
Sextern _CODE_ACCESS int vprintf(const char *_format, va_list _ap)
S               __ATTRIBUTE ((__format__ (__printf__, 1, 0)));
Sextern _CODE_ACCESS int vsprintf(char *_string, const char *_format,
S				 va_list _ap)
S               __ATTRIBUTE ((__format__ (__printf__, 2, 0)));
Sextern _CODE_ACCESS int vsnprintf(char *_string, size_t _n, 
S				  const char *_format, va_list _ap)
S               __ATTRIBUTE ((__format__ (__printf__, 3, 0)));
S
S/****************************************************************************/
S/* CHARACTER INPUT/OUTPUT FUNCTIONS                                         */
S/****************************************************************************/
Sextern _CODE_ACCESS int     fgetc(register FILE *_fp);
Sextern _CODE_ACCESS char   *fgets(char *_ptr, register int _size,
S				  register FILE *_fp);
Sextern _CODE_ACCESS int     fputc(int _c, register FILE *_fp);
Sextern _CODE_ACCESS int     fputs(const char *_ptr, register FILE *_fp);
Sextern _CODE_ACCESS int     getc(FILE *_p);
Sextern _CODE_ACCESS int     getchar(void);
Sextern _CODE_ACCESS char   *gets(char *_ptr); 
Sextern _CODE_ACCESS int     putc(int _x, FILE *_fp);
Sextern _CODE_ACCESS int     putchar(int _x);
Sextern _CODE_ACCESS int     puts(const char *_ptr); 
Sextern _CODE_ACCESS int     ungetc(int _c, register FILE *_fp);
S
S/****************************************************************************/
S/* DIRECT INPUT/OUTPUT FUNCTIONS                                            */
S/****************************************************************************/
Sextern _CODE_ACCESS size_t  fread(void *_ptr, size_t _size, size_t _count,
S				  FILE *_fp);
Sextern _CODE_ACCESS size_t  fwrite(const void *_ptr, size_t _size,
S				   size_t _count, register FILE *_fp); 
S
S/****************************************************************************/
S/* FILE POSITIONING FUNCTIONS                                               */
S/****************************************************************************/
Sextern _CODE_ACCESS int     fgetpos(FILE *_fp, fpos_t *_pos);
Sextern _CODE_ACCESS int     fseek(register FILE *_fp, long _offset,
S				  int _ptrname);
Sextern _CODE_ACCESS int     fsetpos(FILE *_fp, const fpos_t *_pos);
Sextern _CODE_ACCESS long    ftell(FILE *_fp);
Sextern _CODE_ACCESS void    rewind(register FILE *_fp); 
S
S/****************************************************************************/
S/* ERROR-HANDLING FUNCTIONS                                                 */
S/****************************************************************************/
Sextern _CODE_ACCESS void    clearerr(FILE *_fp);
Sextern _CODE_ACCESS int     feof(FILE *_fp);
Sextern _CODE_ACCESS int     ferror(FILE *_fp);
Sextern _CODE_ACCESS void    perror(const char *_s);
S
S#define _getchar()      getc(stdin)
S#define _putchar(_x)    putc((_x), stdout)
S#define _clearerr(_fp)   ((void) ((_fp)->flags &= ~(_STATERR | _STATEOF)))
S
S#define _ferror(_x)     ((_x)->flags & _STATERR)
S
S#define _remove(_fl)    (unlink((_fl)))
S
S#ifdef __cplusplus
S} /* extern "C" namespace std */
S#endif  /* __cplusplus */
S
N#endif  /* #ifndef _STDIO */
N
N#if defined(__cplusplus) && !defined(_CPP_STYLE_HEADER)
X#if 0L && !0L
Susing std::size_t;
Susing std::FILE;
Susing std::fpos_t;
Susing std::_ftable;
Susing std::_tmpnams;
Susing std::remove;
Susing std::rename;
Susing std::tmpfile;
Susing std::tmpnam;
Susing std::fclose;
Susing std::fopen;
Susing std::freopen;
Susing std::setbuf;
Susing std::setvbuf;
Susing std::fflush;
Susing std::fprintf;
Susing std::fscanf;
Susing std::printf;
Susing std::scanf;
Susing std::sprintf;
Susing std::snprintf;
Susing std::sscanf;
Susing std::vfprintf;
Susing std::vprintf;
Susing std::vsprintf;
Susing std::vsnprintf;
Susing std::fgetc;
Susing std::fgets;
Susing std::fputc;
Susing std::fputs;
Susing std::getc;
Susing std::getchar;
Susing std::gets;
Susing std::putc;
Susing std::putchar;
Susing std::puts;
Susing std::ungetc;
Susing std::fread;
Susing std::fwrite;
Susing std::fgetpos;
Susing std::fseek;
Susing std::fsetpos;
Susing std::ftell;
Susing std::rewind;
Susing std::clearerr;
Susing std::feof;
Susing std::ferror;
Susing std::perror;
S
N#endif  /* _CPP_STYLE_HEADER */
N
N
L 15 "../src/ECGSystem_main.c" 2
N#include "evm5515.h"
L 1 "../inc/evm5515.h" 1
N /******************************************************************************
N**File Name			: ADS1298_Init.c
N**File Description	:
N**Created By		:
N**Creation Date		:
N**Last modified date:
N**Remarks			:
N*******************************************************************************/
N
N#ifndef STK5505_
N#define STK5505_
N
N/* ------------------------------------------------------------------------ *
N *                                                                          *
N *  Variable types                                                          *
N *                                                                          *
N * ------------------------------------------------------------------------ */
N
N#define Uint32  unsigned long
N#define Uint16  unsigned short
N#define Uint8   unsigned char
N#define Int32   int
N#define Int16   short
N#define Int8    char
N
N#define SW_BREAKPOINT      while(1);
N/* ------------------------------------------------------------------------ *
N *  System Module                                                           *
N * ------------------------------------------------------------------------ */
N#define SYS_EXBUSSEL       *(volatile ioport Uint16*)(0x1c00)
N#define SYS_PCGCR1         *(volatile ioport Uint16*)(0x1c02)
N#define SYS_PCGCR2         *(volatile ioport Uint16*)(0x1c03)
N#define SYS_PRCNTR         *(volatile ioport Uint16*)(0x1c04)
N#define SYS_PRCNTRLR       *(volatile ioport Uint16*)(0x1c05)
N#define SYS_GPIO_DIR0      *(volatile ioport Uint16*)(0x1c06)
N#define SYS_GPIO_DIR1      *(volatile ioport Uint16*)(0x1c07)
N#define SYS_GPIO_DATAIN0   *(volatile ioport Uint16*)(0x1c08)
N#define SYS_GPIO_DATAIN1   *(volatile ioport Uint16*)(0x1c09)
N#define SYS_GPIO_DATAOUT0  *(volatile ioport Uint16*)(0x1c0a)
N#define SYS_GPIO_DATAOUT1  *(volatile ioport Uint16*)(0x1c0b)
N#define SYS_OUTDRSTR       *(volatile ioport Uint16*)(0x1c16)
N#define SYS_SPPDIR         *(volatile ioport Uint16*)(0x1c17)
N
N/* ------------------------------------------------------------------------ *
N *  I2C Module                                                              *
N * ------------------------------------------------------------------------ */
N 
N#define I2C_IER    	       *(volatile ioport Uint16*)(0x1A04)
N#define I2C_STR    	       *(volatile ioport Uint16*)(0x1A08)
N#define I2C_CLKL           *(volatile ioport Uint16*)(0x1A0C)
N#define I2C_CLKH           *(volatile ioport Uint16*)(0x1A10)
N#define I2C_CNT    		   *(volatile ioport Uint16*)(0x1A14)
N#define I2C_DRR    		   *(volatile ioport Uint16*)(0x1A18)
N#define I2C_SAR    	       *(volatile ioport Uint16*)(0x1A1C)
N#define I2C_DXR    	       *(volatile ioport Uint16*)(0x1A20)
N#define I2C_MDR            *(volatile ioport Uint16*)(0x1A24)
N#define I2C_EDR    	       *(volatile ioport Uint16*)(0x1A2C)
N#define I2C_PSC    	       *(volatile ioport Uint16*)(0x1A30)
N/* ------------------------------------------------------------------------ *
N *  I2S Module                                                              *
N * ------------------------------------------------------------------------ */
N#define I2S0_CR            *(volatile ioport Uint16*)(0x2800)
N#define I2S0_SRGR          *(volatile ioport Uint16*)(0x2804)
N#define I2S0_W0_LSW_W      *(volatile ioport Uint16*)(0x2808)
N#define I2S0_W0_MSW_W      *(volatile ioport Uint16*)(0x2809)
N#define I2S0_W1_LSW_W      *(volatile ioport Uint16*)(0x280C)
N#define I2S0_W1_MSW_W      *(volatile ioport Uint16*)(0x280D)
N#define I2S0_IR            *(volatile ioport Uint16*)(0x2810)
N#define I2S0_ICMR          *(volatile ioport Uint16*)(0x2814)
N#define I2S0_W0_LSW_R      *(volatile ioport Uint16*)(0x2828)
N#define I2S0_W0_MSW_R      *(volatile ioport Uint16*)(0x2829)
N#define I2S0_W1_LSW_R      *(volatile ioport Uint16*)(0x282C)
N#define I2S0_W1_MSW_R      *(volatile ioport Uint16*)(0x282D)
N/* I2S2 */
N#define I2S2_CR            *(volatile ioport Uint16*)(0x2A00)
N#define I2S2_SRGR          *(volatile ioport Uint16*)(0x2A04)
N#define I2S2_W0_LSW_W      *(volatile ioport Uint16*)(0x2A08)
N#define I2S2_W0_MSW_W      *(volatile ioport Uint16*)(0x2A09)
N#define I2S2_W1_LSW_W      *(volatile ioport Uint16*)(0x2A0C)
N#define I2S2_W1_MSW_W      *(volatile ioport Uint16*)(0x2A0D)
N#define I2S2_IR            *(volatile ioport Uint16*)(0x2A10)
N#define I2S2_ICMR          *(volatile ioport Uint16*)(0x2A14)
N#define I2S2_W0_LSW_R      *(volatile ioport Uint16*)(0x2A28)
N#define I2S2_W0_MSW_R      *(volatile ioport Uint16*)(0x2A29)
N#define I2S2_W1_LSW_R      *(volatile ioport Uint16*)(0x2A2C)
N#define I2S2_W1_MSW_R      *(volatile ioport Uint16*)(0x2A2D)
N
N/* ------------------------------------------------------------------------ *
N *  UART Module                                                             *
N * ------------------------------------------------------------------------ */
N#define UART_RBR           *(volatile ioport Uint16*)(0x1B00)
N#define UART_THR           *(volatile ioport Uint16*)(0x1B00)
N#define UART_IER           *(volatile ioport Uint16*)(0x1B02)
N#define UART_IIR           *(volatile ioport Uint16*)(0x1B04)
N#define UART_FCR           *(volatile ioport Uint16*)(0x1B04)
N#define UART_LCR           *(volatile ioport Uint16*)(0x1B06)
N#define UART_MCR           *(volatile ioport Uint16*)(0x1B08)
N#define UART_LSR           *(volatile ioport Uint16*)(0x1B0A)
N#define UART_SCR           *(volatile ioport Uint16*)(0x1B0E)
N#define UART_DLL           *(volatile ioport Uint16*)(0x1B10)
N#define UART_DLH           *(volatile ioport Uint16*)(0x1B12)
N#define UART_PWREMU_MGMT   *(volatile ioport Uint16*)(0x1B18)
N
N/* ------------------------------------------------------------------------ *
N *  Prototypes                                                              *
N * ------------------------------------------------------------------------ */
N/* Board Initialization */
NInt16 EVM5515_init(void );
Xshort EVM5515_init(void );
N
N/* Wait Functions */
Nvoid EVM5515_wait( Uint32 delay );
Xvoid EVM5515_wait( unsigned long delay );
Nvoid EVM5515_waitusec( Uint32 usec );
Xvoid EVM5515_waitusec( unsigned long usec );
N
N#endif
N/*EOF*/
L 16 "../src/ECGSystem_main.c" 2
N#include "evm5515_uart.h"
L 1 "../inc/evm5515_uart.h" 1
N /******************************************************************************
N**File Name			: ADS1298_Init.c
N**File Description	:UART Header file
N**Created By		:
N**Creation Date		:
N**Last modified date:
N**Remarks			:
N*******************************************************************************/
N
N#include "evm5515.h"
N
N/* ------------------------------------------------------------------------ *
N *  Prototypes                                                              *
N * ------------------------------------------------------------------------ */
N
NInt16 EVM5515_UART_open( void);
Xshort EVM5515_UART_open( void);
NInt16 EVM5515_UART_close(void );
Xshort EVM5515_UART_close(void );
NInt16 EVM5515_UART_putChar( Uint8 data  );
Xshort EVM5515_UART_putChar( unsigned char data  );
NInt16 EVM5515_UART_getChar( Uint8* data );
Xshort EVM5515_UART_getChar( unsigned char* data );
N/*EOF*/
L 17 "../src/ECGSystem_main.c" 2
N#include <stdint.h>
L 1 "C:/ti/ccsv6/tools/compiler/c5500_4.4.1/include/stdint.h" 1
N/*****************************************************************************/
N/* STDINT.H v4.4.1                                                           */
N/*                                                                           */
N/* Copyright (c) 2002-2012 Texas Instruments Incorporated                    */
N/* http://www.ti.com/                                                        */
N/*                                                                           */
N/*  Redistribution and  use in source  and binary forms, with  or without    */
N/*  modification,  are permitted provided  that the  following conditions    */
N/*  are met:                                                                 */
N/*                                                                           */
N/*     Redistributions  of source  code must  retain the  above copyright    */
N/*     notice, this list of conditions and the following disclaimer.         */
N/*                                                                           */
N/*     Redistributions in binary form  must reproduce the above copyright    */
N/*     notice, this  list of conditions  and the following  disclaimer in    */
N/*     the  documentation  and/or   other  materials  provided  with  the    */
N/*     distribution.                                                         */
N/*                                                                           */
N/*     Neither the  name of Texas Instruments Incorporated  nor the names    */
N/*     of its  contributors may  be used to  endorse or  promote products    */
N/*     derived  from   this  software  without   specific  prior  written    */
N/*     permission.                                                           */
N/*                                                                           */
N/*  THIS SOFTWARE  IS PROVIDED BY THE COPYRIGHT  HOLDERS AND CONTRIBUTORS    */
N/*  "AS IS"  AND ANY  EXPRESS OR IMPLIED  WARRANTIES, INCLUDING,  BUT NOT    */
N/*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR    */
N/*  A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT    */
N/*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,    */
N/*  SPECIAL,  EXEMPLARY,  OR CONSEQUENTIAL  DAMAGES  (INCLUDING, BUT  NOT    */
N/*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,    */
N/*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY    */
N/*  THEORY OF  LIABILITY, WHETHER IN CONTRACT, STRICT  LIABILITY, OR TORT    */
N/*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE    */
N/*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.     */
N/*                                                                           */
N/*****************************************************************************/
N#ifndef _STDINT_H_
N#define _STDINT_H_
N
N/* 7.18.1.1 Exact-width integer types */
N
N#if defined(__MSP430__) || defined(__TMS320C55X_PLUS_BYTE__)
X#if 0L || 0L
S    typedef   signed char    int8_t;
S    typedef unsigned char   uint8_t;
S    typedef          int    int16_t;
S    typedef unsigned int   uint16_t;
S    typedef          long   int32_t;
S    typedef unsigned long  uint32_t;
N#else
N    typedef          int    int16_t;
N    typedef unsigned int   uint16_t;
N    typedef          long   int32_t;
N    typedef unsigned long  uint32_t;
N#endif
N
N    typedef          long long  int40_t;
N    typedef unsigned long long uint40_t;
N
N
N/* 7.18.1.2 Minimum-width integer types */
N
N#if defined(_TMS320C6X) || defined(__TMS470__) || defined(__MSP430__) || \
N    defined(__TMS320C55X_PLUS_BYTE__)  || defined(__ARP32__)
X#if 0L || 0L || 0L ||     0L  || 0L
S    typedef  int8_t   int_least8_t;
S    typedef uint8_t  uint_least8_t;
N#else
N    typedef  int16_t  int_least8_t;
N    typedef uint16_t uint_least8_t;
N#endif
N
N    typedef  int16_t  int_least16_t;
N    typedef uint16_t uint_least16_t;
N    typedef  int32_t  int_least32_t;
N    typedef uint32_t uint_least32_t;
N
N    typedef  int40_t  int_least40_t;
N    typedef uint40_t uint_least40_t;
N
N/* sorry, int_least64_t not implemented for C54x, C55x */
N
N/* 7.18.1.3 Fastest minimum-width integer types */
N
N    typedef  int16_t  int_fast8_t;
N    typedef uint16_t uint_fast8_t;
N    typedef  int16_t  int_fast16_t;
N    typedef uint16_t uint_fast16_t;
N
N    typedef  int32_t  int_fast32_t;
N    typedef uint32_t uint_fast32_t;
N
N    typedef  int40_t  int_fast40_t;
N    typedef uint40_t uint_fast40_t;
N
N
N/* 7.18.1.4 Integer types capable of holding object pointers */
N    typedef          long intptr_t;
N    typedef unsigned long uintptr_t;
N
N/* 7.18.1.5 Greatest-width integer types */
N    typedef          long long intmax_t;
N    typedef unsigned long long uintmax_t;
N
N/* 
N   According to footnotes in the 1999 C standard, "C++ implementations
N   should define these macros only when __STDC_LIMIT_MACROS is defined
N   before <stdint.h> is included." 
N*/
N#if !defined(__cplusplus) || defined(__STDC_LIMIT_MACROS)
X#if !0L || 0L
N
N/* 7.18.2 Limits of specified width integer types */
N
N#if defined(_TMS320C6X) || defined(__TMS470__) || defined(__MSP430__) || \
N    defined(__TMS320C55X_PLUS_BYTE__)  || defined(__ARP32__)
X#if 0L || 0L || 0L ||     0L  || 0L
S    #define  INT8_MAX   0x7f
S    #define  INT8_MIN   (-INT8_MAX-1)
S    #define UINT8_MAX   0xff
N#endif
N
N    #define  INT16_MAX  0x7fff
N    #define  INT16_MIN  (-INT16_MAX-1)
N    #define UINT16_MAX  0xffff
N
N    #define  INT32_MAX  0x7fffffff
N    #define  INT32_MIN  (-INT32_MAX-1)
N    #define UINT32_MAX  0xffffffff
N
N    #define  INT40_MAX  0x7fffffffff
N    #define  INT40_MIN  (-INT40_MAX-1)
N    #define UINT40_MAX  0xffffffffff
N
N
N#if defined(_TMS320C6X) || defined(__TMS470__) || defined(__MSP430__) || \
N    defined(__TMS320C55X_PLUS_BYTE__) || defined(__ARP32__)
X#if 0L || 0L || 0L ||     0L || 0L
S    #define  INT_LEAST8_MAX   INT8_MAX
S    #define  INT_LEAST8_MIN   INT8_MIN
S    #define UINT_LEAST8_MAX   UINT8_MAX
N#else
N    #define  INT_LEAST8_MAX   INT16_MAX
N    #define  INT_LEAST8_MIN   INT16_MIN
N    #define UINT_LEAST8_MAX   UINT16_MAX
N#endif
N
N    #define  INT_LEAST16_MAX  INT16_MAX
N    #define  INT_LEAST16_MIN  INT16_MIN
N    #define UINT_LEAST16_MAX  UINT16_MAX
N    #define  INT_LEAST32_MAX  INT32_MAX
N    #define  INT_LEAST32_MIN  INT32_MIN
N    #define UINT_LEAST32_MAX  UINT32_MAX
N
N    #define  INT_LEAST40_MAX  INT40_MAX
N    #define  INT_LEAST40_MIN  INT40_MIN
N    #define UINT_LEAST40_MAX  UINT40_MAX
N
N
N    #define  INT_FAST8_MAX   INT16_MAX
N    #define  INT_FAST8_MIN   INT16_MIN
N    #define UINT_FAST8_MAX   UINT16_MAX
N    #define  INT_FAST16_MAX  INT16_MAX
N    #define  INT_FAST16_MIN  INT16_MIN
N    #define UINT_FAST16_MAX  UINT16_MAX
N
N    #define  INT_FAST32_MAX  INT32_MAX
N    #define  INT_FAST32_MIN  INT32_MIN
N    #define UINT_FAST32_MAX  UINT32_MAX
N
N    #define  INT_FAST40_MAX  INT40_MAX
N    #define  INT_FAST40_MIN  INT40_MIN
N    #define UINT_FAST40_MAX  UINT40_MAX
N
N
N    #define INTPTR_MAX   INT32_MAX
N    #define INTPTR_MIN   INT32_MIN
N    #define UINTPTR_MAX  UINT32_MAX
N
N    #define INTMAX_MIN   INT40_MIN
N    #define INTMAX_MAX   INT40_MAX
N    #define UINTMAX_MAX  UINT40_MAX
N
N/* 7.18.3 Limits of other integer types */
N
N    #define PTRDIFF_MAX INT16_MAX
N    #define PTRDIFF_MIN INT16_MIN
N
N    #define SIG_ATOMIC_MIN INT16_MIN
N    #define SIG_ATOMIC_MAX INT16_MAX
N
N    #define SIZE_MAX INT16_MAX
N
N#ifndef WCHAR_MAX
N#if !defined(__TI_WCHAR_T_BITS__) || __TI_WCHAR_T_BITS__ == 16
X#if !1L || 16 == 16
N#define WCHAR_MAX 0xffffu
N#else 
S#define WCHAR_MAX 0xffffffffu
N#endif
N#endif
N
N#ifndef WCHAR_MIN
N#define WCHAR_MIN 0
N#endif
N
N    #define WINT_MIN INT16_MIN
N    #define WINT_MAX INT16_MAX
N
N/* 7.18.4.1 Macros for minimum-width integer constants */
N
N/*
N   There is a defect report filed against the C99 standard concerning how 
N   the (U)INTN_C macros should be implemented.  Please refer to --
N   http://wwwold.dkuug.dk/JTC1/SC22/WG14/www/docs/dr_209.htm 
N   for more information.  These macros are implemented according to the
N   suggestion given at this web site.
N*/
N
N    #define  INT8_C(value)  ((int_least8_t)(value))
N    #define UINT8_C(value)  ((uint_least8_t)(value))
N    #define  INT16_C(value) ((int_least16_t)(value))
N    #define UINT16_C(value) ((uint_least16_t)(value))
N    #define  INT32_C(value) ((int_least32_t)(value))
N    #define UINT32_C(value) ((uint_least32_t)(value))
N
N    #define  INT40_C(value) ((int_least40_t)(value))
N    #define UINT40_C(value) ((uint_least40_t)(value))
N
N
N/* 7.18.4.2 Macros for greatest-width integer constants */
N
N    #define  INTMAX_C(value) ((intmax_t)(value))
N    #define UINTMAX_C(value) ((uintmax_t)(value))
N
N#endif /* !defined(__cplusplus) || defined(__STDC_LIMIT_MACROS) */
N
N#endif /* _STDINT_H_ */
L 18 "../src/ECGSystem_main.c" 2
N#include "llc_spi.h"
N#include "corazon.h"/*spi*/
N#include "SPO2_Init_Functions.h"
L 1 "../inc/SPO2_Init_Functions.h" 1
N/******************************************************************************
N**File Name			: SPO2_Init_Functions.h
N**File Description	:
N**Created By		:
N**Creation Date		:
N**Last modified date:
N**Remarks			:
N*******************************************************************************/
N
N#ifndef INC_SPO2_INIT_FUNCTIONS_H_
N#define INC_SPO2_INIT_FUNCTIONS_H_
N
N#define SPO2_AFE_Chip_Select 0
N
N//void Init_AFE44xx_Resource(void);
N
Nvoid AFE44xx_Default_Reg_Init(void);
Nvoid AFE44xx_Reg_Write (unsigned char reg_address, unsigned long data);
Nunsigned long AFE44xx_Reg_Read(unsigned int Reg_address);
Nvoid moving_average_filter(void);
N//void Init_AFE44xx_DRDY_Interrupt (void);
N//void Enable_AFE44xx_DRDY_Interrupt (void);
N//void Disable_AFE44xx_DRDY_Interrupt (void);
N//void Set_GPIO(void);
N//void Set_UCB1_SPI(void);
N//void AFE44xx_Read_All_Regs(unsigned long AFE44xxeg_buf[]);
N                                    //to see reset function
N/*void AFE44xx_Parse_data_packet(void);
Nvoid ADS1292x_Parse_data_packet(void);
Nvoid Set_Device_out_bytes(void);
N*/
Nvoid AFE44xx_PowerOn_Init(void);
N
N#endif /* INC_SPO2_INIT_FUNCTIONS_H_ */
N/*EOF*/
L 21 "../src/ECGSystem_main.c" 2
N#include "SPO2_functions.h"
L 1 "../inc/SPO2_functions.h" 1
N/******************************************************************************
N**File Name			: SPO2_functions.h
N**File Description	:
N**Created By		:
N**Creation Date		:
N**Last modified date:
N**Remarks			:
N*******************************************************************************/
N
N#ifndef INC_SPO2_FUNCTIONS_H_
N#define INC_SPO2_FUNCTIONS_H_
N
N#define FILTER_LENGTH_SPO2 5
N
N
N
Nvoid moving_average_filter(void);
Nunsigned long calculate_ratio(unsigned long dc,unsigned long peak,unsigned long valley);
Nfloat u32_Window_Average(float *u16_Raw_Data);
Nfloat calulate_AC_to_DC_Ratio(unsigned long a,unsigned long b );
N#endif /* INC_SPO2_FUNCTIONS_H_ */
N/*EOF*/
L 22 "../src/ECGSystem_main.c" 2
N#include "I2C.h"
L 1 "../inc/I2C.h" 1
N/******************************************************************************
N**File Name			: I2C.h
N**File Description	:The file declares all the functions defined in I2C.c
N**Created By		:
N**Creation Date		:
N**Last modified date:
N**Remarks			:
N*******************************************************************************/
N 
N#ifndef _I2C_H
N#define _I2C_H
N
N
N#include "psp_common.h"
N
N/*   PCA9535 Comand values*/
N#define CMD_PCA9535_IP0 0x00
N#define CMD_PCA9535_IP1 0x01
N#define CMD_PCA9535_OP0 0x02
N#define CMD_PCA9535_OP1 0x03
N#define CMD_PCA9535_PIP0 0x04
N#define CMD_PCA9535_PIP1 0x05
N#define CMD_PCA9535_CP0 0x06
N#define CMD_PCA9535_CP1 0x07
N
N#define I2C_OWN_ADDR 		   		(0x2F)    /* Dummy address */
N#define I2C_BUS_FREQ 		   		(10000u) 
N
N/* function to write a register through i2c */
Nvoid i2c_write_reg(Uint8 cmd, Uint8* data);
Xvoid i2c_write_reg(unsigned char cmd, unsigned char* data);
N/* function to read register through i2c */
Nvoid i2c_read_reg(Uint8 cmd, Uint8* data);
Xvoid i2c_read_reg(unsigned char cmd, unsigned char* data);
NInt16 EVM5515_I2C_init( void);
Xshort EVM5515_I2C_init( void);
NInt16 EVM5515_I2C_close( void);
Xshort EVM5515_I2C_close( void);
NInt16 EVM5515_I2C_reset(void );
Xshort EVM5515_I2C_reset(void );
NInt16 EVM5515_I2C_write( Uint16 i2c_addr, Uint8* data, Uint16 len );
Xshort EVM5515_I2C_write( unsigned short i2c_addr, unsigned char* data, unsigned short len );
NInt16 EVM5515_I2C_read( Uint16 i2c_addr, Uint8* data, Uint16 len );
Xshort EVM5515_I2C_read( unsigned short i2c_addr, unsigned char* data, unsigned short len );
N#endif
N/*EOF*/
L 23 "../src/ECGSystem_main.c" 2
N#include "Communication_protocol.h"
L 1 "../inc/Communication_protocol.h" 1
N /******************************************************************************
N**File Name			: Communication_protocol.h
N**File Description	:
N**Author    		: Vuon1
N**Creation Date		: 15-Jul-2016
N**Last modified date:
N**Remarks			:
N*******************************************************************************/
N
N#ifndef INC_COMMUNICATION_PROTOCOL_H_
N#define INC_COMMUNICATION_PROTOCOL_H_
N
N#include "tistdtypes.h" /*Uint8 definition*/
N#include "llc_spi.h"	/*spi registers*/
N#include "corazon.h"    /*pointer to spi registers*/
N#include "evm5515_uart.h"/*uart registers*/
L 1 "..\inc\evm5515_uart.h" 1
N /******************************************************************************
N**File Name			: ADS1298_Init.c
N**File Description	:UART Header file
N**Created By		:
N**Creation Date		:
N**Last modified date:
N**Remarks			:
N*******************************************************************************/
N
N#include "evm5515.h"
N
N/* ------------------------------------------------------------------------ *
N *  Prototypes                                                              *
N * ------------------------------------------------------------------------ */
N
NInt16 EVM5515_UART_open( void);
Xshort EVM5515_UART_open( void);
NInt16 EVM5515_UART_close(void );
Xshort EVM5515_UART_close(void );
NInt16 EVM5515_UART_putChar( Uint8 data  );
Xshort EVM5515_UART_putChar( unsigned char data  );
NInt16 EVM5515_UART_getChar( Uint8* data );
Xshort EVM5515_UART_getChar( unsigned char* data );
N/*EOF*/
L 17 "../inc/Communication_protocol.h" 2
N#include "ECGSystemInit.h"
L 1 "..\inc\ECGSystemInit.h" 1
N/******************************************************************************
N**File Name			: ECGSystemInit.h
N**File Description	:Definitions used for ECGSystemInit, and function prototypes for all module Initialization
N**Created By		:
N**Creation Date		:
N**Last modified date:
N**Remarks			:
N*******************************************************************************/
N
N
N#ifndef _ECGSYSTEMINIT_H
N#define _ECGSYSTEMINIT_H
N
N#include<stdio.h>
L 1 "C:/ti/ccsv6/tools/compiler/c5500_4.4.1/include/stdio.h" 1
N/*****************************************************************************/
N/* STDIO.H v4.4.1                                                            */
N/*                                                                           */
N/* Copyright (c) 1993-2012 Texas Instruments Incorporated                    */
N/* http://www.ti.com/                                                        */
N/*                                                                           */
N/*  Redistribution and  use in source  and binary forms, with  or without    */
N/*  modification,  are permitted provided  that the  following conditions    */
N/*  are met:                                                                 */
N/*                                                                           */
N/*     Redistributions  of source  code must  retain the  above copyright    */
N/*     notice, this list of conditions and the following disclaimer.         */
N/*                                                                           */
N/*     Redistributions in binary form  must reproduce the above copyright    */
N/*     notice, this  list of conditions  and the following  disclaimer in    */
N/*     the  documentation  and/or   other  materials  provided  with  the    */
N/*     distribution.                                                         */
N/*                                                                           */
N/*     Neither the  name of Texas Instruments Incorporated  nor the names    */
N/*     of its  contributors may  be used to  endorse or  promote products    */
N/*     derived  from   this  software  without   specific  prior  written    */
N/*     permission.                                                           */
N/*                                                                           */
N/*  THIS SOFTWARE  IS PROVIDED BY THE COPYRIGHT  HOLDERS AND CONTRIBUTORS    */
N/*  "AS IS"  AND ANY  EXPRESS OR IMPLIED  WARRANTIES, INCLUDING,  BUT NOT    */
N/*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR    */
N/*  A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT    */
N/*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,    */
N/*  SPECIAL,  EXEMPLARY,  OR CONSEQUENTIAL  DAMAGES  (INCLUDING, BUT  NOT    */
N/*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,    */
N/*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY    */
N/*  THEORY OF  LIABILITY, WHETHER IN CONTRACT, STRICT  LIABILITY, OR TORT    */
N/*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE    */
N/*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.     */
N/*                                                                           */
N/*****************************************************************************/
N#ifndef _STDIO 
S#define _STDIO
S
S#include <linkage.h>
S#include <stdarg.h>
S
S/*---------------------------------------------------------------------------*/
S/* Attributes are only available in relaxed ANSI mode.                       */
S/*---------------------------------------------------------------------------*/
S#ifndef __ATTRIBUTE
S#if __TI_STRICT_ANSI_MODE__
S#define __ATTRIBUTE(attr)
S#else
S#define __ATTRIBUTE(attr) __attribute__(attr)
S#endif
S#endif
S
S
S#ifdef __cplusplus
S//----------------------------------------------------------------------------
S// <cstdio> IS RECOMMENDED OVER <stdio.h>.  <stdio.h> IS PROVIDED FOR
S// COMPATIBILITY WITH C AND THIS USAGE IS DEPRECATED IN C++
S//----------------------------------------------------------------------------
Sextern "C" namespace std {
S#endif
S
S/****************************************************************************/
S/* TYPES THAT ANSI REQUIRES TO BE DEFINED                                   */
S/****************************************************************************/
S#ifndef _SIZE_T
S#define _SIZE_T
Stypedef __SIZE_T_TYPE__ size_t;
S#endif
S
Stypedef struct {
S      int fd;                    /* File descriptor */
S      unsigned char* buf;        /* Pointer to start of buffer */
S      unsigned char* pos;        /* Position in buffer */
S      unsigned char* bufend;     /* Pointer to end of buffer */
S      unsigned char* buff_stop;  /* Pointer to last read char in buffer */
S      unsigned int   flags;      /* File status flags (see below) */
S} FILE;
S
S#ifndef _FPOS_T
S#define _FPOS_T
Stypedef long fpos_t;
S#endif /* _FPOS_T */
S
S/****************************************************************************/
S/* DEVICE AND STREAM RELATED MACROS                                         */
S/****************************************************************************/
S/****************************************************************************/
S/* MACROS THAT DEFINE AND USE FILE STATUS FLAGS                             */
S/****************************************************************************/
S
S#define _IOFBF       0x0001
S#define _IOLBF       0x0002
S#define _IONBF       0x0004
S#define _BUFFALOC    0x0008
S#define _MODER       0x0010
S#define _MODEW       0x0020
S#define _MODERW      0x0040
S#define _MODEA       0x0080
S#define _MODEBIN     0x0100
S#define _STATEOF     0x0200
S#define _STATERR     0x0400
S#define _UNGETC      0x0800
S#define _TMPFILE     0x1000
S
S#define _SET(_fp, _b)      (((_fp)->flags) |= (_b))
S#define _UNSET(_fp, _b)    (((_fp)->flags) &= ~(_b))
S#define _STCHK(_fp, _b)    (((_fp)->flags) & (_b))
S#define _BUFFMODE(_fp)     (((_fp)->flags) & (_IOFBF | _IOLBF | _IONBF))
S#define _ACCMODE(_fp)      (((_fp)->flags) & (_MODER | _MODEW))
S
S/****************************************************************************/
S/* MACROS THAT ANSI REQUIRES TO BE DEFINED                                  */
S/****************************************************************************/
S#define BUFSIZ          256 
S
S#define FOPEN_MAX       _NFILE
S#define FILENAME_MAX    256  
S#define TMP_MAX         65535
S
S#define stdin     (&_ftable[0])      
S#define stdout    (&_ftable[1])
S#define stderr    (&_ftable[2])
S
S#define L_tmpnam  _LTMPNAM
S
S
S#define SEEK_SET  (0x0000)
S#define SEEK_CUR  (0x0001)
S#define SEEK_END  (0x0002)
S
S#ifndef NULL
S#define NULL 0
S#endif
S
S#ifndef EOF
S#define EOF    (-1)
S#endif
S
S/******** END OF ANSI MACROS ************************************************/
S
S#define P_tmpdir        ""                   /* Path for temp files         */
S
S/****************************************************************************/
S/* DEVICE AND STREAM RELATED DATA STRUCTURES AND MACROS                     */
S/****************************************************************************/
S#define _NFILE           10                   /* Max number of files open   */
S#define _LTMPNAM         16                   /* Length of temp name        */
S
Sextern _DATA_ACCESS FILE _ftable[_NFILE];
Sextern _DATA_ACCESS char _tmpnams[_NFILE][_LTMPNAM];
S
S/****************************************************************************/
S/*   FUNCTION DEFINITIONS  - ANSI                                           */
S/****************************************************************************/
S/****************************************************************************/
S/* OPERATIONS ON FILES                                                      */
S/****************************************************************************/
Sextern _CODE_ACCESS int     remove(const char *_file);
Sextern _CODE_ACCESS int     rename(const char *_old, const char *_new);
Sextern _CODE_ACCESS FILE   *tmpfile(void);
Sextern _CODE_ACCESS char   *tmpnam(char *_s);
S
S/****************************************************************************/
S/* FILE ACCESS FUNCTIONS                                                    */
S/****************************************************************************/
Sextern _CODE_ACCESS int     fclose(FILE *_fp); 
Sextern _CODE_ACCESS FILE   *fopen(const char *_fname, const char *_mode);
Sextern _CODE_ACCESS FILE   *freopen(const char *_fname, const char *_mode,
S			            register FILE *_fp);
Sextern _CODE_ACCESS void    setbuf(register FILE *_fp, char *_buf);
Sextern _CODE_ACCESS int     setvbuf(register FILE *_fp, register char *_buf, 
S			            register int _type, register size_t _size);
Sextern _CODE_ACCESS int     fflush(register FILE *_fp); 
S
S/****************************************************************************/
S/* FORMATTED INPUT/OUTPUT FUNCTIONS                                         */
S/****************************************************************************/
Sextern _CODE_ACCESS int fprintf(FILE *_fp, const char *_format, ...)
S               __ATTRIBUTE ((__format__ (__printf__, 2, 3)));
Sextern _CODE_ACCESS int fscanf(FILE *_fp, const char *_fmt, ...)
S               __ATTRIBUTE ((__format__ (__scanf__, 2, 3)));
Sextern _CODE_ACCESS int printf(const char *_format, ...)
S               __ATTRIBUTE ((__format__ (__printf__, 1, 2)));
Sextern _CODE_ACCESS int scanf(const char *_fmt, ...)
S               __ATTRIBUTE ((__format__ (__scanf__, 1, 2)));
Sextern _CODE_ACCESS int sprintf(char *_string, const char *_format, ...)
S               __ATTRIBUTE ((__format__ (__printf__, 2, 3)));
Sextern _CODE_ACCESS int snprintf(char *_string, size_t _n, 
S				 const char *_format, ...)
S               __ATTRIBUTE ((__format__ (__printf__, 3, 4)));
Sextern _CODE_ACCESS int sscanf(const char *_str, const char *_fmt, ...)
S               __ATTRIBUTE ((__format__ (__scanf__, 2, 3)));
Sextern _CODE_ACCESS int vfprintf(FILE *_fp, const char *_format, va_list _ap)
S               __ATTRIBUTE ((__format__ (__printf__, 2, 0)));
Sextern _CODE_ACCESS int vprintf(const char *_format, va_list _ap)
S               __ATTRIBUTE ((__format__ (__printf__, 1, 0)));
Sextern _CODE_ACCESS int vsprintf(char *_string, const char *_format,
S				 va_list _ap)
S               __ATTRIBUTE ((__format__ (__printf__, 2, 0)));
Sextern _CODE_ACCESS int vsnprintf(char *_string, size_t _n, 
S				  const char *_format, va_list _ap)
S               __ATTRIBUTE ((__format__ (__printf__, 3, 0)));
S
S/****************************************************************************/
S/* CHARACTER INPUT/OUTPUT FUNCTIONS                                         */
S/****************************************************************************/
Sextern _CODE_ACCESS int     fgetc(register FILE *_fp);
Sextern _CODE_ACCESS char   *fgets(char *_ptr, register int _size,
S				  register FILE *_fp);
Sextern _CODE_ACCESS int     fputc(int _c, register FILE *_fp);
Sextern _CODE_ACCESS int     fputs(const char *_ptr, register FILE *_fp);
Sextern _CODE_ACCESS int     getc(FILE *_p);
Sextern _CODE_ACCESS int     getchar(void);
Sextern _CODE_ACCESS char   *gets(char *_ptr); 
Sextern _CODE_ACCESS int     putc(int _x, FILE *_fp);
Sextern _CODE_ACCESS int     putchar(int _x);
Sextern _CODE_ACCESS int     puts(const char *_ptr); 
Sextern _CODE_ACCESS int     ungetc(int _c, register FILE *_fp);
S
S/****************************************************************************/
S/* DIRECT INPUT/OUTPUT FUNCTIONS                                            */
S/****************************************************************************/
Sextern _CODE_ACCESS size_t  fread(void *_ptr, size_t _size, size_t _count,
S				  FILE *_fp);
Sextern _CODE_ACCESS size_t  fwrite(const void *_ptr, size_t _size,
S				   size_t _count, register FILE *_fp); 
S
S/****************************************************************************/
S/* FILE POSITIONING FUNCTIONS                                               */
S/****************************************************************************/
Sextern _CODE_ACCESS int     fgetpos(FILE *_fp, fpos_t *_pos);
Sextern _CODE_ACCESS int     fseek(register FILE *_fp, long _offset,
S				  int _ptrname);
Sextern _CODE_ACCESS int     fsetpos(FILE *_fp, const fpos_t *_pos);
Sextern _CODE_ACCESS long    ftell(FILE *_fp);
Sextern _CODE_ACCESS void    rewind(register FILE *_fp); 
S
S/****************************************************************************/
S/* ERROR-HANDLING FUNCTIONS                                                 */
S/****************************************************************************/
Sextern _CODE_ACCESS void    clearerr(FILE *_fp);
Sextern _CODE_ACCESS int     feof(FILE *_fp);
Sextern _CODE_ACCESS int     ferror(FILE *_fp);
Sextern _CODE_ACCESS void    perror(const char *_s);
S
S#define _getchar()      getc(stdin)
S#define _putchar(_x)    putc((_x), stdout)
S#define _clearerr(_fp)   ((void) ((_fp)->flags &= ~(_STATERR | _STATEOF)))
S
S#define _ferror(_x)     ((_x)->flags & _STATERR)
S
S#define _remove(_fl)    (unlink((_fl)))
S
S#ifdef __cplusplus
S} /* extern "C" namespace std */
S#endif  /* __cplusplus */
S
N#endif  /* #ifndef _STDIO */
N
N#if defined(__cplusplus) && !defined(_CPP_STYLE_HEADER)
X#if 0L && !0L
Susing std::size_t;
Susing std::FILE;
Susing std::fpos_t;
Susing std::_ftable;
Susing std::_tmpnams;
Susing std::remove;
Susing std::rename;
Susing std::tmpfile;
Susing std::tmpnam;
Susing std::fclose;
Susing std::fopen;
Susing std::freopen;
Susing std::setbuf;
Susing std::setvbuf;
Susing std::fflush;
Susing std::fprintf;
Susing std::fscanf;
Susing std::printf;
Susing std::scanf;
Susing std::sprintf;
Susing std::snprintf;
Susing std::sscanf;
Susing std::vfprintf;
Susing std::vprintf;
Susing std::vsprintf;
Susing std::vsnprintf;
Susing std::fgetc;
Susing std::fgets;
Susing std::fputc;
Susing std::fputs;
Susing std::getc;
Susing std::getchar;
Susing std::gets;
Susing std::putc;
Susing std::putchar;
Susing std::puts;
Susing std::ungetc;
Susing std::fread;
Susing std::fwrite;
Susing std::fgetpos;
Susing std::fseek;
Susing std::fsetpos;
Susing std::ftell;
Susing std::rewind;
Susing std::clearerr;
Susing std::feof;
Susing std::ferror;
Susing std::perror;
S
N#endif  /* _CPP_STYLE_HEADER */
N
N
L 15 "..\inc\ECGSystemInit.h" 2
N#include<math.h>
L 1 "C:/ti/ccsv6/tools/compiler/c5500_4.4.1/include/math.h" 1
N/****************************************************************************/
N/*  math.h           v4.4.1                                                 */
N/*                                                                          */
N/* Copyright (c) 1997-2012 Texas Instruments Incorporated                   */
N/* http://www.ti.com/                                                       */
N/*                                                                          */
N/*  Redistribution and  use in source  and binary forms, with  or without   */
N/*  modification,  are permitted provided  that the  following conditions   */
N/*  are met:                                                                */
N/*                                                                          */
N/*     Redistributions  of source  code must  retain the  above copyright   */
N/*     notice, this list of conditions and the following disclaimer.        */
N/*                                                                          */
N/*     Redistributions in binary form  must reproduce the above copyright   */
N/*     notice, this  list of conditions  and the following  disclaimer in   */
N/*     the  documentation  and/or   other  materials  provided  with  the   */
N/*     distribution.                                                        */
N/*                                                                          */
N/*     Neither the  name of Texas Instruments Incorporated  nor the names   */
N/*     of its  contributors may  be used to  endorse or  promote products   */
N/*     derived  from   this  software  without   specific  prior  written   */
N/*     permission.                                                          */
N/*                                                                          */
N/*  THIS SOFTWARE  IS PROVIDED BY THE COPYRIGHT  HOLDERS AND CONTRIBUTORS   */
N/*  "AS IS"  AND ANY  EXPRESS OR IMPLIED  WARRANTIES, INCLUDING,  BUT NOT   */
N/*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR   */
N/*  A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT   */
N/*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,   */
N/*  SPECIAL,  EXEMPLARY,  OR CONSEQUENTIAL  DAMAGES  (INCLUDING, BUT  NOT   */
N/*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,   */
N/*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY   */
N/*  THEORY OF  LIABILITY, WHETHER IN CONTRACT, STRICT  LIABILITY, OR TORT   */
N/*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE   */
N/*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.    */
N/*                                                                          */
N/****************************************************************************/
N
N#ifndef _TI_ENHANCED_MATH_H
S#define _TI_ENHANCED_MATH_H
N#endif
N
N#ifndef __math__
S#define __math__
S
S#ifndef EDOM
S#define EDOM   1
S#endif
S
S#ifndef ERANGE
S#define ERANGE 2
S#endif
S
S#include <float.h>
S#define HUGE_VAL DBL_MAX
S#define HUGE_VALL LDBL_MAX
S
S#include <access.h>
S#include <elfnames.h>
S
S#ifdef __cplusplus
S//----------------------------------------------------------------------------
S// <cmath> IS RECOMMENDED OVER <math.h>.  <math.h> IS PROVIDED FOR 
S// COMPATIBILITY WITH C AND THIS USAGE IS DEPRECATED IN C++
S//----------------------------------------------------------------------------
Sextern "C" namespace std {
S#endif
S
S__EXTERN double sqrt (double x);
S__EXTERN double exp  (double x);
S__EXTERN double log  (double x);
S__EXTERN double log10(double x);
S__EXTERN double pow  (double x, double y);
S__EXTERN double sin  (double x);
S__EXTERN double cos  (double x);
S__EXTERN double tan  (double x);
S__EXTERN double asin (double x);
S__EXTERN double acos (double x);
S__EXTERN double atan (double x);
S__EXTERN double atan2(double y, double x);
S__EXTERN double sinh (double x);
S__EXTERN double cosh (double x);
S__EXTERN double tanh (double x);
S
S__INLINE double ceil (double x);
S__INLINE double floor(double x);
S
S__EXTERN double fabs (double x);
S
S__EXTERN double ldexp(double x, int n);
S__EXTERN double frexp(double x, int *exp);
S__EXTERN double modf (double x, double *ip);
S__EXTERN double fmod (double x, double y);
S
S/* An inline version of fmod that works for limited domain only */
S/* See comments in implementation below */
S__INLINE double _FMOD(double x, double y);
S
S/* these present in many linked images, so we'll tell you about them. */
S__EXTERN double _nround(double x); /* round-to-nearest */
S__EXTERN double _trunc(double x); /* truncate towards 0 */
S
S#ifdef __cplusplus
S} /* extern "C" namespace std */
S#endif /* __cplusplus */
S
S/* the ANSI-optional *f and *l routines */
S#include <mathf.h>
S#include <mathl.h>
S
S#include <access.h>
S
S#ifdef __cplusplus
Sextern "C" namespace std {
S#endif
S
S#ifdef _TI_ENHANCED_MATH_H
S/* ------------------------------------------------- */
S/* Routines below are an addition to ANSI math.h     */
S/* Some (noted with "9x" in comment) will become ANSI*/
S/* once C9x is approved.                             */
S/* ------------------------------------------------- */
S
S__EXTERN double rsqrt(double x); /*   == 1/sqrt(x) but *MUCH* faster         */
S__EXTERN double exp2 (double x); /*9x mathematically equiv to pow(2.0 ,x)    */
S__EXTERN double exp10(double x); /*   mathematically equiv to pow(10.0,x)    */
S__EXTERN double log2 (double x); /*9x mathematically equiv to log(x)/log(2.0)*/
S
S__EXTERN double powi(double x, int i); /* equiv to pow(x,(double)i) */
S
S__EXTERN double cot  (double x);
S__EXTERN double acot (double x);
S__EXTERN double acot2(double x, double y);
S
S__EXTERN double coth (double x);
S
S__EXTERN double asinh(double x); /* 9x */
S__EXTERN double acosh(double x); /* 9x */
S__EXTERN double atanh(double x); /* 9x */
S__EXTERN double acoth(double x);
S
S#ifndef __INLINE_ISINF__
S#define __INLINE_ISINF__ 0
S#endif
S
S#if __INLINE_ISINF__
S__INLINE int __isinf(double x);
S#else
S__EXTERN int __isinf(double x);
S#endif
S
S__INLINE int __isnan(volatile double x);
S__INLINE int __isfinite(double x);
S__INLINE int __isnormal(double x);
S__EXTERN int __fpclassify(double x);
S
S#define isinf(x) (sizeof(x) == sizeof(double) ? __isinf(x) : \
S                  sizeof(x) == sizeof(float) ? __isinff(x) : __isinfl(x))
X#define isinf(x) (sizeof(x) == sizeof(double) ? __isinf(x) :                   sizeof(x) == sizeof(float) ? __isinff(x) : __isinfl(x))
S
S#define isnan(x) (sizeof(x) == sizeof(double) ? __isnan(x) : \
S                  sizeof(x) == sizeof(float) ? __isnanf(x) : __isnanl(x))
X#define isnan(x) (sizeof(x) == sizeof(double) ? __isnan(x) :                   sizeof(x) == sizeof(float) ? __isnanf(x) : __isnanl(x))
S
S#define isfinite(x) (sizeof(x) == sizeof(double) ? __isfinite(x) : \
S                     sizeof(x) == sizeof(float) ? __isfinitef(x) : \
S                     __isfinitel(x))
X#define isfinite(x) (sizeof(x) == sizeof(double) ? __isfinite(x) :                      sizeof(x) == sizeof(float) ? __isfinitef(x) :                      __isfinitel(x))
S
S#define isnormal(x) (sizeof(x) == sizeof(double) ? __isnormal(x) : \
S                     sizeof(x) == sizeof(float) ? __isnormalf(x) : \
S                     __isnormall(x))
X#define isnormal(x) (sizeof(x) == sizeof(double) ? __isnormal(x) :                      sizeof(x) == sizeof(float) ? __isnormalf(x) :                      __isnormall(x))
S
S#define fpclassify(x) (sizeof(x) == sizeof(double) ? __fpclassify(x) : \
S                       sizeof(x) == sizeof(float) ? __fpclassifyf(x) : \
S                       __fpclassifyl(x))
X#define fpclassify(x) (sizeof(x) == sizeof(double) ? __fpclassify(x) :                        sizeof(x) == sizeof(float) ? __fpclassifyf(x) :                        __fpclassifyl(x))
S
S#define round _nround /* 9x round-to-nearest   */
S#define trunc _trunc /* 9x truncate towards 0 */
S
S/*Definitions of classification macros used in fp_classify 
S  We do not support subnormal numbers yet, but the classification exists for
S  when they are supported */
S
S#define FP_INFINITE  1
S#define FP_NAN       2
S#define FP_NORMAL    3
S#define FP_ZERO      4
S#define FP_SUBNORMAL 5
S
S#endif /* defined(_TI_ENHANCED_MATH_H) */
S
S#ifdef __cplusplus
S} /* extern "C" namespace std */
S
S#endif /* __cplusplus */
S
S
S#ifdef _INLINE
S/****************************************************************************/
S/*  Inline versions of floor, ceil, fmod                                    */
S/****************************************************************************/
S
S#ifdef __cplusplus
Snamespace std {
S#endif
S
Sstatic __inline double floor(double x) 
S{
S   double y; 
S   return (modf(x, &y) < 0 ? y - 1 : y);
S}
S
Sstatic __inline double ceil(double x)
S{
S   double y; 
S   return (modf(x, &y) > 0 ? y + 1 : y);
S}
S
S/* 
S   The implementation below does not work correctly for all cases.
S   Consider the case of fmod(Big, 3), for any Big > 2**(MANT_DIG+2).
S   The correct result is one of 0,1, or 2.
S   But the implementation below will *always* return 0 
S   because the quotient is only an approximation.
S*/
Sstatic __inline double _FMOD(double x, double y)
S{
S   double d = fabs(x); 
S   if (d - fabs(y) == d) return (0);
S   modf(x/y, &d);  
S   return (x - d * y);
S}
S
S#ifdef _TI_ENHANCED_MATH_H
S
S#if __INLINE_ISINF__
S#ifndef REAL_TO_REALNUM
S#error isinf can only be inlined in the compilation of the rts
S#endif
S
Sstatic __inline int __isinf(double x)
S{
S  realnum _x;
S  REAL_TO_REALNUM(x, _x);
S  return _x.exp == (REAL_EMAX + 1) && (_x.mantissa << 1) == 0;
S}
S
S#endif /* __INLINE_ISINF___ */
S
S#pragma diag_suppress 681
Sstatic __inline int __isnan(volatile double x)
S{
S  return x != x;
S}
S#pragma diag_default 681
S
Sstatic __inline int __isfinite(double x)
S{
S  return (!__isinf(x) && !__isnan(x));
S}
S
Sstatic __inline int __isnormal(double x)
S{
S  return (__isfinite(x) && x != 0.0);
S}
S
S#endif /* defined(_TI_ENHANCED_MATH_H) */
S
S#ifdef __cplusplus
S} /* namespace std */
S#endif /* __cplusplus */
S
S#endif /* _INLINE */
S
S/*******************************************************************************/
S/* CQ35082 : Overloaded version of math functions for float and long double    */
S/*           removed from here, and include in cmath instead (see Section 26.5 */
S/*           of C++ standard for details). Thus cpp_inline_math.h is now       */
S/*           included in cmath .                                               */
S/*******************************************************************************/
S#include <unaccess.h>
S
N#endif /* __math__ */
N
N#if defined(__cplusplus) && !defined(_CPP_STYLE_HEADER)
X#if 0L && !0L
Susing std::sqrt; 
Susing std::exp; 
Susing std::log; 
Susing std::log10; 
Susing std::pow; 
Susing std::sin; 
Susing std::cos; 
Susing std::tan; 
Susing std::asin; 
Susing std::acos;
Susing std::atan;
Susing std::atan2;
Susing std::sinh;
Susing std::cosh;
Susing std::tanh;
Susing std::ceil;
Susing std::floor;
Susing std::fabs;
Susing std::ldexp;
Susing std::frexp;
Susing std::modf;
Susing std::fmod;
S
S#ifdef _TI_ENHANCED_MATH_H
Susing std::rsqrt;
Susing std::exp2;
Susing std::exp10;
Susing std::log2;
Susing std::powi;
Susing std::cot;
Susing std::acot;
Susing std::acot2;
Susing std::coth;
Susing std::asinh;
Susing std::acosh;
Susing std::atanh;
Susing std::acoth;
S#endif /* _TI_ENHANCED_MATH_H */
S
N#endif /* _CPP_STYLE_HEADER */
N
N#if defined(__cplusplus) && defined(_TI_ENHANCED_MATH_H)
X#if 0L && 1L
Susing std::__isnan;
Susing std::__isinf;
Susing std::__isfinite;
Susing std::__isnormal;
Susing std::__fpclassify;
Susing std::_nround; /* round-to-nearest */
Susing std::_trunc;
N#endif /* __cplusplus && _TI_ENHANCED_MATH_H */
L 16 "..\inc\ECGSystemInit.h" 2
N#include<file.h>
N#include <stdio.h>
L 1 "C:/ti/ccsv6/tools/compiler/c5500_4.4.1/include/stdio.h" 1
N/*****************************************************************************/
N/* STDIO.H v4.4.1                                                            */
N/*                                                                           */
N/* Copyright (c) 1993-2012 Texas Instruments Incorporated                    */
N/* http://www.ti.com/                                                        */
N/*                                                                           */
N/*  Redistribution and  use in source  and binary forms, with  or without    */
N/*  modification,  are permitted provided  that the  following conditions    */
N/*  are met:                                                                 */
N/*                                                                           */
N/*     Redistributions  of source  code must  retain the  above copyright    */
N/*     notice, this list of conditions and the following disclaimer.         */
N/*                                                                           */
N/*     Redistributions in binary form  must reproduce the above copyright    */
N/*     notice, this  list of conditions  and the following  disclaimer in    */
N/*     the  documentation  and/or   other  materials  provided  with  the    */
N/*     distribution.                                                         */
N/*                                                                           */
N/*     Neither the  name of Texas Instruments Incorporated  nor the names    */
N/*     of its  contributors may  be used to  endorse or  promote products    */
N/*     derived  from   this  software  without   specific  prior  written    */
N/*     permission.                                                           */
N/*                                                                           */
N/*  THIS SOFTWARE  IS PROVIDED BY THE COPYRIGHT  HOLDERS AND CONTRIBUTORS    */
N/*  "AS IS"  AND ANY  EXPRESS OR IMPLIED  WARRANTIES, INCLUDING,  BUT NOT    */
N/*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR    */
N/*  A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT    */
N/*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,    */
N/*  SPECIAL,  EXEMPLARY,  OR CONSEQUENTIAL  DAMAGES  (INCLUDING, BUT  NOT    */
N/*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,    */
N/*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY    */
N/*  THEORY OF  LIABILITY, WHETHER IN CONTRACT, STRICT  LIABILITY, OR TORT    */
N/*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE    */
N/*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.     */
N/*                                                                           */
N/*****************************************************************************/
N#ifndef _STDIO 
S#define _STDIO
S
S#include <linkage.h>
S#include <stdarg.h>
S
S/*---------------------------------------------------------------------------*/
S/* Attributes are only available in relaxed ANSI mode.                       */
S/*---------------------------------------------------------------------------*/
S#ifndef __ATTRIBUTE
S#if __TI_STRICT_ANSI_MODE__
S#define __ATTRIBUTE(attr)
S#else
S#define __ATTRIBUTE(attr) __attribute__(attr)
S#endif
S#endif
S
S
S#ifdef __cplusplus
S//----------------------------------------------------------------------------
S// <cstdio> IS RECOMMENDED OVER <stdio.h>.  <stdio.h> IS PROVIDED FOR
S// COMPATIBILITY WITH C AND THIS USAGE IS DEPRECATED IN C++
S//----------------------------------------------------------------------------
Sextern "C" namespace std {
S#endif
S
S/****************************************************************************/
S/* TYPES THAT ANSI REQUIRES TO BE DEFINED                                   */
S/****************************************************************************/
S#ifndef _SIZE_T
S#define _SIZE_T
Stypedef __SIZE_T_TYPE__ size_t;
S#endif
S
Stypedef struct {
S      int fd;                    /* File descriptor */
S      unsigned char* buf;        /* Pointer to start of buffer */
S      unsigned char* pos;        /* Position in buffer */
S      unsigned char* bufend;     /* Pointer to end of buffer */
S      unsigned char* buff_stop;  /* Pointer to last read char in buffer */
S      unsigned int   flags;      /* File status flags (see below) */
S} FILE;
S
S#ifndef _FPOS_T
S#define _FPOS_T
Stypedef long fpos_t;
S#endif /* _FPOS_T */
S
S/****************************************************************************/
S/* DEVICE AND STREAM RELATED MACROS                                         */
S/****************************************************************************/
S/****************************************************************************/
S/* MACROS THAT DEFINE AND USE FILE STATUS FLAGS                             */
S/****************************************************************************/
S
S#define _IOFBF       0x0001
S#define _IOLBF       0x0002
S#define _IONBF       0x0004
S#define _BUFFALOC    0x0008
S#define _MODER       0x0010
S#define _MODEW       0x0020
S#define _MODERW      0x0040
S#define _MODEA       0x0080
S#define _MODEBIN     0x0100
S#define _STATEOF     0x0200
S#define _STATERR     0x0400
S#define _UNGETC      0x0800
S#define _TMPFILE     0x1000
S
S#define _SET(_fp, _b)      (((_fp)->flags) |= (_b))
S#define _UNSET(_fp, _b)    (((_fp)->flags) &= ~(_b))
S#define _STCHK(_fp, _b)    (((_fp)->flags) & (_b))
S#define _BUFFMODE(_fp)     (((_fp)->flags) & (_IOFBF | _IOLBF | _IONBF))
S#define _ACCMODE(_fp)      (((_fp)->flags) & (_MODER | _MODEW))
S
S/****************************************************************************/
S/* MACROS THAT ANSI REQUIRES TO BE DEFINED                                  */
S/****************************************************************************/
S#define BUFSIZ          256 
S
S#define FOPEN_MAX       _NFILE
S#define FILENAME_MAX    256  
S#define TMP_MAX         65535
S
S#define stdin     (&_ftable[0])      
S#define stdout    (&_ftable[1])
S#define stderr    (&_ftable[2])
S
S#define L_tmpnam  _LTMPNAM
S
S
S#define SEEK_SET  (0x0000)
S#define SEEK_CUR  (0x0001)
S#define SEEK_END  (0x0002)
S
S#ifndef NULL
S#define NULL 0
S#endif
S
S#ifndef EOF
S#define EOF    (-1)
S#endif
S
S/******** END OF ANSI MACROS ************************************************/
S
S#define P_tmpdir        ""                   /* Path for temp files         */
S
S/****************************************************************************/
S/* DEVICE AND STREAM RELATED DATA STRUCTURES AND MACROS                     */
S/****************************************************************************/
S#define _NFILE           10                   /* Max number of files open   */
S#define _LTMPNAM         16                   /* Length of temp name        */
S
Sextern _DATA_ACCESS FILE _ftable[_NFILE];
Sextern _DATA_ACCESS char _tmpnams[_NFILE][_LTMPNAM];
S
S/****************************************************************************/
S/*   FUNCTION DEFINITIONS  - ANSI                                           */
S/****************************************************************************/
S/****************************************************************************/
S/* OPERATIONS ON FILES                                                      */
S/****************************************************************************/
Sextern _CODE_ACCESS int     remove(const char *_file);
Sextern _CODE_ACCESS int     rename(const char *_old, const char *_new);
Sextern _CODE_ACCESS FILE   *tmpfile(void);
Sextern _CODE_ACCESS char   *tmpnam(char *_s);
S
S/****************************************************************************/
S/* FILE ACCESS FUNCTIONS                                                    */
S/****************************************************************************/
Sextern _CODE_ACCESS int     fclose(FILE *_fp); 
Sextern _CODE_ACCESS FILE   *fopen(const char *_fname, const char *_mode);
Sextern _CODE_ACCESS FILE   *freopen(const char *_fname, const char *_mode,
S			            register FILE *_fp);
Sextern _CODE_ACCESS void    setbuf(register FILE *_fp, char *_buf);
Sextern _CODE_ACCESS int     setvbuf(register FILE *_fp, register char *_buf, 
S			            register int _type, register size_t _size);
Sextern _CODE_ACCESS int     fflush(register FILE *_fp); 
S
S/****************************************************************************/
S/* FORMATTED INPUT/OUTPUT FUNCTIONS                                         */
S/****************************************************************************/
Sextern _CODE_ACCESS int fprintf(FILE *_fp, const char *_format, ...)
S               __ATTRIBUTE ((__format__ (__printf__, 2, 3)));
Sextern _CODE_ACCESS int fscanf(FILE *_fp, const char *_fmt, ...)
S               __ATTRIBUTE ((__format__ (__scanf__, 2, 3)));
Sextern _CODE_ACCESS int printf(const char *_format, ...)
S               __ATTRIBUTE ((__format__ (__printf__, 1, 2)));
Sextern _CODE_ACCESS int scanf(const char *_fmt, ...)
S               __ATTRIBUTE ((__format__ (__scanf__, 1, 2)));
Sextern _CODE_ACCESS int sprintf(char *_string, const char *_format, ...)
S               __ATTRIBUTE ((__format__ (__printf__, 2, 3)));
Sextern _CODE_ACCESS int snprintf(char *_string, size_t _n, 
S				 const char *_format, ...)
S               __ATTRIBUTE ((__format__ (__printf__, 3, 4)));
Sextern _CODE_ACCESS int sscanf(const char *_str, const char *_fmt, ...)
S               __ATTRIBUTE ((__format__ (__scanf__, 2, 3)));
Sextern _CODE_ACCESS int vfprintf(FILE *_fp, const char *_format, va_list _ap)
S               __ATTRIBUTE ((__format__ (__printf__, 2, 0)));
Sextern _CODE_ACCESS int vprintf(const char *_format, va_list _ap)
S               __ATTRIBUTE ((__format__ (__printf__, 1, 0)));
Sextern _CODE_ACCESS int vsprintf(char *_string, const char *_format,
S				 va_list _ap)
S               __ATTRIBUTE ((__format__ (__printf__, 2, 0)));
Sextern _CODE_ACCESS int vsnprintf(char *_string, size_t _n, 
S				  const char *_format, va_list _ap)
S               __ATTRIBUTE ((__format__ (__printf__, 3, 0)));
S
S/****************************************************************************/
S/* CHARACTER INPUT/OUTPUT FUNCTIONS                                         */
S/****************************************************************************/
Sextern _CODE_ACCESS int     fgetc(register FILE *_fp);
Sextern _CODE_ACCESS char   *fgets(char *_ptr, register int _size,
S				  register FILE *_fp);
Sextern _CODE_ACCESS int     fputc(int _c, register FILE *_fp);
Sextern _CODE_ACCESS int     fputs(const char *_ptr, register FILE *_fp);
Sextern _CODE_ACCESS int     getc(FILE *_p);
Sextern _CODE_ACCESS int     getchar(void);
Sextern _CODE_ACCESS char   *gets(char *_ptr); 
Sextern _CODE_ACCESS int     putc(int _x, FILE *_fp);
Sextern _CODE_ACCESS int     putchar(int _x);
Sextern _CODE_ACCESS int     puts(const char *_ptr); 
Sextern _CODE_ACCESS int     ungetc(int _c, register FILE *_fp);
S
S/****************************************************************************/
S/* DIRECT INPUT/OUTPUT FUNCTIONS                                            */
S/****************************************************************************/
Sextern _CODE_ACCESS size_t  fread(void *_ptr, size_t _size, size_t _count,
S				  FILE *_fp);
Sextern _CODE_ACCESS size_t  fwrite(const void *_ptr, size_t _size,
S				   size_t _count, register FILE *_fp); 
S
S/****************************************************************************/
S/* FILE POSITIONING FUNCTIONS                                               */
S/****************************************************************************/
Sextern _CODE_ACCESS int     fgetpos(FILE *_fp, fpos_t *_pos);
Sextern _CODE_ACCESS int     fseek(register FILE *_fp, long _offset,
S				  int _ptrname);
Sextern _CODE_ACCESS int     fsetpos(FILE *_fp, const fpos_t *_pos);
Sextern _CODE_ACCESS long    ftell(FILE *_fp);
Sextern _CODE_ACCESS void    rewind(register FILE *_fp); 
S
S/****************************************************************************/
S/* ERROR-HANDLING FUNCTIONS                                                 */
S/****************************************************************************/
Sextern _CODE_ACCESS void    clearerr(FILE *_fp);
Sextern _CODE_ACCESS int     feof(FILE *_fp);
Sextern _CODE_ACCESS int     ferror(FILE *_fp);
Sextern _CODE_ACCESS void    perror(const char *_s);
S
S#define _getchar()      getc(stdin)
S#define _putchar(_x)    putc((_x), stdout)
S#define _clearerr(_fp)   ((void) ((_fp)->flags &= ~(_STATERR | _STATEOF)))
S
S#define _ferror(_x)     ((_x)->flags & _STATERR)
S
S#define _remove(_fl)    (unlink((_fl)))
S
S#ifdef __cplusplus
S} /* extern "C" namespace std */
S#endif  /* __cplusplus */
S
N#endif  /* #ifndef _STDIO */
N
N#if defined(__cplusplus) && !defined(_CPP_STYLE_HEADER)
X#if 0L && !0L
Susing std::size_t;
Susing std::FILE;
Susing std::fpos_t;
Susing std::_ftable;
Susing std::_tmpnams;
Susing std::remove;
Susing std::rename;
Susing std::tmpfile;
Susing std::tmpnam;
Susing std::fclose;
Susing std::fopen;
Susing std::freopen;
Susing std::setbuf;
Susing std::setvbuf;
Susing std::fflush;
Susing std::fprintf;
Susing std::fscanf;
Susing std::printf;
Susing std::scanf;
Susing std::sprintf;
Susing std::snprintf;
Susing std::sscanf;
Susing std::vfprintf;
Susing std::vprintf;
Susing std::vsprintf;
Susing std::vsnprintf;
Susing std::fgetc;
Susing std::fgets;
Susing std::fputc;
Susing std::fputs;
Susing std::getc;
Susing std::getchar;
Susing std::gets;
Susing std::putc;
Susing std::putchar;
Susing std::puts;
Susing std::ungetc;
Susing std::fread;
Susing std::fwrite;
Susing std::fgetpos;
Susing std::fseek;
Susing std::fsetpos;
Susing std::ftell;
Susing std::rewind;
Susing std::clearerr;
Susing std::feof;
Susing std::ferror;
Susing std::perror;
S
N#endif  /* _CPP_STYLE_HEADER */
N
N
L 18 "..\inc\ECGSystemInit.h" 2
N#include <std.h>
N#include "psp_common.h"
N#include "dda_spi.h"
N#include "timer.h"
L 1 "..\inc\timer.h" 1
N /******************************************************************************
N**File Name			: timer.h
N**File Description	:Timer register declaration
N**Created By		:
N**Creation Date		:
N**Last modified date:
N**Remarks			:
N*******************************************************************************/
N
N/*  Timer register declaration*/
N
N#define CPU_TIM0_CTRL ((ioport volatile unsigned*)0x1810)
N#define CPU_TIM0_PLWR ((ioport volatile unsigned*)0x1812)
N#define CPU_TIM0_PHWR ((ioport volatile unsigned*)0x1813)
N#define CPU_TIM0_CLWR ((ioport volatile unsigned*)0x1814)
N#define CPU_TIM0_CHWR ((ioport volatile unsigned*)0x1815)
N#define CPU_TIM0_IER ((ioport volatile unsigned*)0x1816)
N#define CPU_TIMINT_AGGR ((ioport volatile unsigned*)0x1c14)
N#define CPU_PSRCR ((ioport volatile unsigned*)0x1c04)
N#define CPU_PRCR ((ioport volatile unsigned*)0x1c05)
N
N
Nvoid ECG_Timer_Init(void);	   // function to Initialize the timer
N/*EOF*/
L 22 "..\inc\ECGSystemInit.h" 2
N#include "llc_spi.h"
N#include "ADS1298.h"
N/*#include "cpu_power.h"*/
N#include "csl_ioport.h"
N#include "corazon.h"
N#include "cslr.h"
N#include "lcd.h"
N#include "I2C.h"
N#include "LCD_FontTable.h"
N#include "sar.h"
L 1 "..\inc\sar.h" 1
N/******************************************************************************
N**File Name			: sar.h
N**File Description	:Sar register and button
N**Created By		:
N**Creation Date		:
N**Last modified date:
N**Remarks			:
N*******************************************************************************/
N
N#include "psp_common.h"
N
N
Nvoid Init_SAR(void);		// function to Init SAR
NUint16 Get_Sar_Key(void);	// function to read the value of the current key
Xunsigned short Get_Sar_Key(void);	
N
N
N/*  SAR Register Definitions*/
N#define SARCTRL   ((ioport volatile unsigned*)0x7012)
N#define SARDATA   ((ioport volatile unsigned*)0x7014)
N#define SARCLKCTRL   ((ioport volatile unsigned*)0x7016)
N#define SARPINCTRL   ((ioport volatile unsigned*)0x7018)
N#define SARGPOCTRL   ((ioport volatile unsigned*)0x701A)
N
N/*  Values corresponding to each keys*/
N#define SW6  0x236
N#define SW7  0
N#define SW8  0xd6
N#define SW9  0x166
N#define SW10 0x1d6
N#define SW11 0x215
N#define SW12 0x257
N#define SW13 0x2c7
N#define SW14 0x316
N#define SW15 0x274
N#define SW16 0x3FB
N/*EOF*/
L 32 "..\inc\ECGSystemInit.h" 2
N#include "ECGGlobals.h"
N#include "typedef.h"
N
N#define GPIO10 10
N#define GPIO11 11
N
N/* function to initialize peripherals*/
NECG_Status ECG_C5505Init();	
N/* function to initialize ADS1298*/	 
NECG_Status ECG_ADS1298_INIT();
N/*function to initialize SPI*/
NECG_Status ECG_SPI_INIT(Uint32 spiClkRate, Uint16 wordLen, Uint16 frameLen );
XECG_Status ECG_SPI_INIT(unsigned long spiClkRate, unsigned short wordLen, unsigned short frameLen );
N
N
N/* function to Initialize LCD*/
Nvoid ECG_LCD_INIT(void);
N/* function to INIT GPIO*/			 
Nvoid ECG_GPIO_INIT();
N/* function to Initialize UART*/
Nvoid ECG_UART_INIT(void);
N/* function to Init FE*/				
NECG_Status ECG_FE_INIT();
N/* function to detect FE*/
NECG_Status ECG_SAR_INIT();
N
NUint8 FE_DETECT();					
Xunsigned char FE_DETECT();					
N/* Function to Initialize timer*/
Nvoid ECG_TIMER_INIT();
N/*	function give wait cycles*/
Nextern void wait(Uint32);
Xextern void wait(unsigned long);
N
N#endif
N/*EOF*/
L 18 "../inc/Communication_protocol.h" 2
N#define START 				0x24
N#define STOP 				0x2A
N
N
N#define ECG_DATA_LEAD1 				0x01
N#define ECG_DATA_LEAD2 				0x02
N#define ECG_DATA_LEAD3 				0x03
N#define RESPIRATION_DATA 			0x04
N#define SPO2_DATA 					0x05
N#define RESPIRATION_RATE 			0x06
N#define ECG_HEART_RATE 				0x07
N#define SPO2_HEART_RATE 			0x08
N#define NIBP_HEART_RATE 			0x09
N#define SPO2_VALUE 					0x0A
N#define BODY_TEMPERATURE		    0x0B
N#define BLOOD_PRESSURE 				0x0C
N
N#define MB_chip_select 1
N
N
N//commands sent to MSP from MB via UART-code to be tested in MB
N#define Power_set_cmd 											0x01
N#define Mode_NIBPMode_ECGConfig_NotchFilterFrequency_set_cmd	0x02
N#define NIBP_Interval_set_cmd									0x03
N#define Diagnostics_status_request 								0x04
N#define Flags_Status_request 									0x05
N#define Respiration_rate_request 								0x06
N#define Blood_oxygen_percentage_request 						0x07
N#define Heart_rate_request 										0x08
N#define NIBP_Sys_Dia_Map_HR_request 							0x09
N#define Temperature_request 									0x0A
N#define Heart_rate_SpO2_request 								0x0B
N
N/*Power_set_cmd*/
N#define ECG_Power 							0x01
N#define SpO2_Power 							0x02
N#define NIBP_Power							0x04
N#define Temperature_Read 					0x08
N#define Reset_Packet_Identifier 			0x10
N/*EOF*/
N
N/*Mode_NIBPMode_ECGConfig_NotchFilterFrequency_set_cmd*/
N#define Normal_0_Calibration_1_Mode 		0x01
N#define Patient_change_Yes_1_No_0 			0x02
N#define NIBP_Pediatric_1_Adult_0_Mode       0x04
N#define ECG_Gain_x1
N#define ECG_Gain_x4
N#define ECG_Gain_x8
N#define Notch_Filter_50hz_0_60hz_1          0x20
N
N
N/*definitions for Diagnostics_StsRegister*/
N#define ECG_AFE_Failure 					0x01
N#define SpO2_AFE_Failure 					0x02
N#define SpO2_LED_PD_sensor_cable_fault		0x04
N#define ECG_Data_Stability					0x08
N#define SpO2_Data_Stability 				0x10
N#define Respiration_Data_Stability 			0x20
N#define Heart_Rate_Stability 				0x40
N#define Temperature_Data_Stability 			0x80
N
N
N/*definitions for Flag_StsRegister*/
N#define ECG_Lead_I_OFF
N#define ECG_Lead_II_OFF
N#define ECG_Lead_III_OFF
N#define SpO2_Probe_power_ON					0x04
N#define Pace_Detected						0x08
N#define Arrhythmia_Detected				0x10
N#define Air_Leakage_Detected				0x20
N#define Cuff_Inflation_Failure_Detected				0x40
N#define Motion_detected					0x80
N
N
N
N
N
N
N
N
N
N
N
N
N
N
N
N
N
N
N
N
N#define IDLE 						0
N#define START_BYTE_RECEIVED 		1
N#define STOP_BYTE_RECEIVED 			2
N
N
N
N
N
N
N
N
N/*eg
N Diagnostics_StatusRegister&=~(ECG_Data_Stability) will set ecg data as unstable
N Diagnostics_StatusRegister|=(ECG_Data_Stability) will set ecg data as stable
N*/
N
N
N
N
N
N
N
N
N
N
N
NUint8 pack(void* p,Uint8 data_index,Uint8 no_of_data_points,Uint8* pack);
Xunsigned char pack(void* p,unsigned char data_index,unsigned char no_of_data_points,unsigned char* pack);
Nvoid receive_uartdata(void);
Nvoid receive_I2Cdata(void);
Nvoid unpack_I2Cdata(void);
Nvoid unpack_uartdata(Uint8* datapack);
Xvoid unpack_uartdata(unsigned char* datapack);
Nvoid send_via_spi(Uint8 no_of_datapoints,Uint8* datapack);
Xvoid send_via_spi(unsigned char no_of_datapoints,unsigned char* datapack);
Nvoid Mode_NIBPModeECGgain_NotchFreq(Uint8 Mode_NIBPmodeECGgainNotch);
Xvoid Mode_NIBPModeECGgain_NotchFreq(unsigned char Mode_NIBPmodeECGgainNotch);
Nvoid send_via_uart(Uint8 no_of_datapoints, Uint8* datapack);
Xvoid send_via_uart(unsigned char no_of_datapoints, unsigned char* datapack);
NUint8 pack_to_MSP(Uint8 data,Uint8 data_index,Uint8* pack);
Xunsigned char pack_to_MSP(unsigned char data,unsigned char data_index,unsigned char* pack);
Nvoid Power_peripherals(Uint8 power_command);
Xvoid Power_peripherals(unsigned char power_command);
NUint8 Set_ECG_gain(Uint16 ecg_setting);
Xunsigned char Set_ECG_gain(unsigned short ecg_setting);
N#endif /* INC_COMMUNICATION_PROTOCOL_H_ */
N/*EOF*/
L 24 "../src/ECGSystem_main.c" 2
N#include "Arrythmia.h"
L 1 "../inc/Arrythmia.h" 1
N /******************************************************************************
N**File Name			: Arrythmia.h
N**File Description	:
N**Author    		: Vuon1
N**Creation Date		: 03-Aug-2016
N**Last modified date:
N**Remarks			:
N*******************************************************************************/
N
N#ifndef INC_ARRYTHMIA_H_
N#define INC_ARRYTHMIA_H_
N
N
N#include "tistdtypes.h" /*Uint8 definition*/
N#include "csl_ioport.h"
N
N
N
N
N
N
N
N
N
N
N
N
N
N
N
N
N
N
N
Nvoid arrythmia_detection(int *buffer,int buff_idf);
N
N
Nint down_slope_right(short int s,float* x,short int fs);
Nfloat* absolute(float* x,float m);
Nfloat* normalize(float* x,float *y,float m);
Nfloat find_big(short int s1,short int s2,float* x);
Nfloat find_small(int s1,int s2,float* x);
Nint search(short int s1,float *x,float search_value);
Nint up_slope_right(short int s,float* x,short int fs);
Nshort int up_slope_left(short int s,float* x,short int fs);
N
Nint down_slope_left(short int s,float* x,short int fs);
N
Nshort int down_left(short int s,float* x);
Nshort int down_right(short int s,float* x);
N
Nshort int up_left(short int s,float* x);
Nint down_slope_right(short int s,float* x,short int fs);
N
N#endif /* INC_ARRYTHMIA_H_ */
L 25 "../src/ECGSystem_main.c" 2
N#include "Filtering.h"
L 1 "../inc/Filtering.h" 1
N /******************************************************************************
N**File Name			: Filtering.h
N**File Description	:
N**Author    		: Vuon1
N**Creation Date		: 05-Aug-2016
N**Last modified date:
N**Remarks			:
N*******************************************************************************/
N
N#ifndef INC_FILTERING_H_
N#define INC_FILTERING_H_
N
N
Nlong Low_Pass_Filter(int ecgdata);
Nlong DC_removal(long LPF_2_hz_out1);
Nlong Notch_Filter(long LPF_2_hz_out);
N#endif /* INC_FILTERING_H_ */
L 26 "../src/ECGSystem_main.c" 2
N#include "System_states.h"
L 1 "../inc/System_states.h" 1
N /******************************************************************************
N**File Name			: System_states.h
N**File Description	:
N**Author    		: Vuon1
N**Creation Date		: 08-Sep-2016
N**Last modified date:
N**Remarks			:
N*******************************************************************************/
N
N#ifndef INC_SYSTEM_STATES_H_
N#define INC_SYSTEM_STATES_H_
N/*system main states*/
N
Nenum SystemStates
N{	SystemState_Idle,
N	SystemState_ProcessingECGdata,
N	SystemState_ProcessingSPO2data,
N	SystemState_DetectingArrythmiaInBuffer1,
N	SystemState_DetectingArrythmiaInBuffer2,
N	SystemState_ProcessingUARTdata,
N	SystemState_ProcessingGPIOinterrupt,
N	SystemState_ProcessingI2Cdata
N};
Nenum SystemState_ProcessingECGdataStates
N{	SystemState_ProcessingECGdata_ServicingInterruptReadingECGdata,
N	SystemState_ProcessingECGdata_ServicingInterruptRemovingDC,
N	SystemState_ProcessingECGdata_ServicingInterruptFillingArrythmiaBuffer,
N	SystemState_ProcessingECGdata_DetectingLeadoff,
N	SystemState_ProcessingECGdata_NotchFiltering,
N	SystemState_ProcessingECGdata_SendingECGDATAtoMB
N};
N
N
Nenum SystemState_DetectingArrythmiaInBufferStates
N{
N	SystemState_DetectingArrythmia_
N};
N/*enum Subsystem_ProcessingI2CdataStates
N{
N
N};*/
N
Nenum SystemState_ProcessingSPO2dataStates
N{
N	SystemState_ProcessingSPO2dataStates_peakdetected,
N	SystemState_ProcessingSPO2dataStates_valleydetected
N};
N
N#endif /* INC_SYSTEM_STATES_H_ */
L 27 "../src/ECGSystem_main.c" 2
N#include "timer.h"
L 1 "../inc/timer.h" 1
N /******************************************************************************
N**File Name			: timer.h
N**File Description	:Timer register declaration
N**Created By		:
N**Creation Date		:
N**Last modified date:
N**Remarks			:
N*******************************************************************************/
N
N/*  Timer register declaration*/
N
N#define CPU_TIM0_CTRL ((ioport volatile unsigned*)0x1810)
N#define CPU_TIM0_PLWR ((ioport volatile unsigned*)0x1812)
N#define CPU_TIM0_PHWR ((ioport volatile unsigned*)0x1813)
N#define CPU_TIM0_CLWR ((ioport volatile unsigned*)0x1814)
N#define CPU_TIM0_CHWR ((ioport volatile unsigned*)0x1815)
N#define CPU_TIM0_IER ((ioport volatile unsigned*)0x1816)
N#define CPU_TIMINT_AGGR ((ioport volatile unsigned*)0x1c14)
N#define CPU_PSRCR ((ioport volatile unsigned*)0x1c04)
N#define CPU_PRCR ((ioport volatile unsigned*)0x1c05)
N
N
Nvoid ECG_Timer_Init(void);	   // function to Initialize the timer
N/*EOF*/
L 28 "../src/ECGSystem_main.c" 2
N#define ENABLE_NOTCH_FILTER 1
N
N/*system states*/
NUint8 SystemState_DetectingArrythmiaInBufferStatus;
Xunsigned char SystemState_DetectingArrythmiaInBufferStatus;
NUint8 SystemStatus;
Xunsigned char SystemStatus;
NUint8 SystemState_ProcessingECGdataStatus;
Xunsigned char SystemState_ProcessingECGdataStatus;
NUint8 SystemState_ProcessingSPO2dataStatus;
Xunsigned char SystemState_ProcessingSPO2dataStatus;
N
N
N/* Global variables used in this file */
NUint8 Packet[50] = {0};
Xunsigned char Packet[50] = {0};
N
N/* System Information structure*/								
NECG_System_Info ECG_Info;			
N/* Leadoffstatus variable*/
NUint16 LeadOffStatus = 0x01FF;	
Xunsigned short LeadOffStatus = 0x01FF;	
N
Nextern Uint8 FE_Active;
Xextern unsigned char FE_Active;
N
N/* Handle for UART driver*/
Nextern PSP_Handle hUart;
N/* structure which stores the UART configuration values*/
Nextern PSP_UartConfig uartCfg;	
NUint8 Initial_HR_Flag = 0;
Xunsigned char Initial_HR_Flag = 0;
NUint8 HR_flag = 0;
Xunsigned char HR_flag = 0;
NUint8 LeadOff_Flag = 0;
Xunsigned char LeadOff_Flag = 0;
NUint8 HR_display_count = 1;	
Xunsigned char HR_display_count = 1;	
NUint8 lead_digit_count = 0;
Xunsigned char lead_digit_count = 0;
NUint8 ReadKey = 0;
Xunsigned char ReadKey = 0;
N
N
N/*from SPO2 */
Nunsigned long AFE44xx_SPO2_Data_buf[7]={0};
Nextern unsigned int k;
Nextern unsigned long adc_value,adc_value1;
Nextern unsigned long raw_RED_buffer[FILTER_LENGTH_SPO2];
Xextern unsigned long raw_RED_buffer[5];
Nextern float SPO2;
Nextern unsigned long raw_IR_buffer[FILTER_LENGTH_SPO2];
Xextern unsigned long raw_IR_buffer[5];
Nstatic Uint8 digit1,digit2,digit3;
Xstatic unsigned char digit1,digit2,digit3;
N		
N
N#define NEW_PG_BOARD    1
N
N#define MB_chip_select 1
N#define SPO2_AFE_Chip_Select 0
N
N#if (NEW_PG_BOARD ==1)
X#if (1 ==1)
N#define ADS1298_V6      0x001000
N#define ADS1298_V1      0x080000
N#define ADS1298_V5432   0x078000
N#define ADS1298_LL_LA   0x006000
N#define ADS1298_RA      0x000020
N#else
S#define ADS1298_V6 0x100000
S#define ADS1298_V1 0x80000
S#define ADS1298_V5432 0x78000
S#define ADS1298_LL_LA 0x6000
S#define ADS1298_RA 0x20
N#endif
N
N
N
N/*from ECGSystemFunctions*/
Nextern Uint8 UARTPacket[256];
Xextern unsigned char UARTPacket[256];
Nextern unsigned char UARTstart;
Nextern unsigned char UARTend;
Nextern unsigned UARTdataRdy;
Nextern Uint8 col;
Xextern unsigned char col;
Nextern Uint16 QRS_Heart_Rate;//used in updating global heart rate register
Xextern unsigned short QRS_Heart_Rate;
Nextern Uint8 Zoom_Flag;
Xextern unsigned char Zoom_Flag;
N
N
N
N/*from interrupt handlers*/
Nextern Uint8 LCDDataReady;
Xextern unsigned char LCDDataReady;
Nextern Uint8 UARTDataReady;
Xextern unsigned char UARTDataReady;
Nextern Uint8 QRSDataReady;
Xextern unsigned char QRSDataReady;
Nextern Int16 ECGData[];
Xextern short ECGData[];
Nextern Uint32 LeadStatus;
Xextern unsigned long LeadStatus;
Nextern Uint8 SPO2_dataready_flag;
Xextern unsigned char SPO2_dataready_flag;
Nextern Uint8 UART_dataready_flag;
Xextern unsigned char UART_dataready_flag;
Nextern Uint8 I2C_dataready_flag;
Xextern unsigned char I2C_dataready_flag;
Nextern Uint16 LeadSelect;
Xextern unsigned short LeadSelect;
Nextern Int8 intCount;
Xextern char intCount;
Nextern Uint16 KeyPressed;
Xextern unsigned short KeyPressed;
Nextern Uint16 ChannelNo;
Xextern unsigned short ChannelNo;
Nextern Uint8 GPIO_interrupt_flag;
Xextern unsigned char GPIO_interrupt_flag;
N
N/*referred in Communication_protocol */
NUint8 Heart_Rate;
Xunsigned char Heart_Rate;
NUint8 Respiration_Rate;
Xunsigned char Respiration_Rate;
NUint8 SPO2_Percentage;// referred in SPO2_functions.c
Xunsigned char SPO2_Percentage;
NUint8 Notch_Filter_60_Hz=0;//referred in Filtering.c
Xunsigned char Notch_Filter_60_Hz=0;
NUint8 Normal_Mode=1;
Xunsigned char Normal_Mode=1;
NUint8 Patient_change=0;
Xunsigned char Patient_change=0;
NUint8 NIBP_Pediatric_Mode=0;//default mode is adult mode
Xunsigned char NIBP_Pediatric_Mode=0;
N
N/*for Communication_protocol*/
NUint8 Diagnostics_StsRegister=(Uint8)0;
Xunsigned char Diagnostics_StsRegister=(unsigned char)0;
NUint8 Flag_StsRegister=(Uint8)0;
Xunsigned char Flag_StsRegister=(unsigned char)0;
N
N
NUint8 ECG_gain=(Uint8)1;
Xunsigned char ECG_gain=(unsigned char)1;
NUint8 ECG_gain_bit0=(Uint8)0;
Xunsigned char ECG_gain_bit0=(unsigned char)0;
NUint8 ECG_gain_bit1=(Uint8)0;
Xunsigned char ECG_gain_bit1=(unsigned char)0;
N
NUint8 uart_msp_data=(Uint8)0;
Xunsigned char uart_msp_data=(unsigned char)0;
N
NUint8 ModeNIBPmode_ECGConfig_Notchfreq_Config_CmdRegister=(Uint8)0;
Xunsigned char ModeNIBPmode_ECGConfig_Notchfreq_Config_CmdRegister=(unsigned char)0;
NUint8 ModeNIBPmode_ECGConfig_Notchfreq_Config_StsRegister=(Uint8)0;
Xunsigned char ModeNIBPmode_ECGConfig_Notchfreq_Config_StsRegister=(unsigned char)0;
NUint8 Power_Config_CmdRegister=(Uint8)0;
Xunsigned char Power_Config_CmdRegister=(unsigned char)0;
NUint8 Power_Config_StsRegister=(Uint8)0;
Xunsigned char Power_Config_StsRegister=(unsigned char)0;
NUint8 NIBP_IntervalSet_CmdRegister=(Uint8)0;
Xunsigned char NIBP_IntervalSet_CmdRegister=(unsigned char)0;
NUint8 NIBP_IntervalSet_StsRegister=(Uint8)0;
Xunsigned char NIBP_IntervalSet_StsRegister=(unsigned char)0;
N
N
N
N/*for arrythmia.c*/
NUint8 buf_ready_to_process=0;
Xunsigned char buf_ready_to_process=0;
N Uint8 buf1_ready_to_process=0;
X unsigned char buf1_ready_to_process=0;
N Uint8 buf2_ready_to_process=0;
X unsigned char buf2_ready_to_process=0;
N int ECG_arrythmia_buf2[2500]={0};
N int ECG_arrythmia_buf1[2500]={0};
N
N//keypad will not be used
Nvoid ProcessKey(Uint16 AdCVal)
Xvoid ProcessKey(unsigned short AdCVal)
N{
N	static Uint16 Key =0, PrevKey=0, FirstTime=1, PrevADCval ;
X	static unsigned short Key =0, PrevKey=0, FirstTime=1, PrevADCval ;
N
N	if ( FirstTime == 1 )
N	{
N		PrevADCval = AdCVal;
N
N		FirstTime = 0;
N	}
N	if ( PrevADCval != AdCVal)
N	{
N		if ( AdCVal < SW8 -10)
X		if ( AdCVal < 0xd6 -10)
N		{
N			Key = 7;
N			if ( PrevKey != Key)
N			{
N	          	LeadSelect ++;
N				ChannelNo++;
N
N				if(ChannelNo >= 8) ChannelNo =0;
N				
N				if(LeadSelect >= 12)
N				{
N					LeadSelect = 0;
N				}
N
N				KeyPressed = 1;
N				PrevKey = Key;
N			}
N		}
N	    else if((AdCVal >= SW8) && (AdCVal <= SW9))
X	    else if((AdCVal >= 0xd6) && (AdCVal <= 0x166))
N	    {printf("_not_first_time\n");
N			Key = 8;
N			if ( PrevKey != Key)
N			{
N				if(Zoom_Flag==0)
N				{
N					Zoom_Flag = 1;
N				}
N				else
N				if(Zoom_Flag==1)
N				{
N					Zoom_Flag = 2;
N				}
N				else
N				if(Zoom_Flag==2)
N				{
N					Zoom_Flag = 0;
N				}
N				PrevKey = Key;
N
N			}
N	    }
N	    else
N	    {
N			Key = 0;
N
N	    }
N
N		PrevKey = Key;
N		PrevADCval = AdCVal;
N	}
N}
N
N//keypad will not be used
Nvoid KeyDetect(void)
N{
N
N	static Uint16 Count=0,AdCVal, debounce = 0;
X	static unsigned short Count=0,AdCVal, debounce = 0;
N
N		Count++;
N		if ( debounce > 0) debounce--;
N		if (Count > 5 && debounce==0)
N		{
N			AdCVal = Get_Sar_Key();
N			Count =0;
N		} 
N		else if ( Count > 4)
N		{
N			ProcessKey(AdCVal);
N			if (KeyPressed)
N			{
N				debounce = 25;
N			}
N		}
N
N}
N
N//lcd will not be used
Nvoid ECG_LCD_LeadDisplay(LeadSelect)	
N{
N
N	static Uint16 digit_count = 0;
X	static unsigned short digit_count = 0;
N	char fontArr[4] ={" II"};
N	switch(LeadSelect )
N	{
N		case 0 :
N    	strcpy((char*)fontArr,"  I");
N		break;
N
N		case 1 :
N    	strcpy((char*)fontArr," II");
N
N		break;
N		case 2 :
N    	strcpy((char*)fontArr,"III");
N
N		break;
N		case 3 :
N    	strcpy((char*)fontArr,"aVR");
N
N		break;
N		case 4 :
N    	strcpy((char*)fontArr,"aVL");
N
N		break;
N		case 5 :
N    	strcpy((char*)fontArr,"aVF");
N
N		break;
N		case 6 :
N    	strcpy((char*)fontArr," V1");
N
N		break;
N		case 7 :
N    	strcpy((char*)fontArr," V2");
N
N		break;
N		case 8 :
N    	strcpy((char*)fontArr," V3");
N
N		break;
N		case 9 :
N    	strcpy((char*)fontArr," V4");
N
N		break;
N		case 10 :
N    	strcpy((char*)fontArr," V5");
N
N		break;
N		case 11 :
N    	strcpy((char*)fontArr," V6");
N
N		break;
N    }
N
N    // diplay lead name  Peter
N#if 0    
S    /*Displaying 1st Digit of Lead value */	
S	draw_font_Lead(4,13,fontArr[0],BLUE);
S	/*Displaying 2nd Digit of Lead value */	
S	draw_font_Lead(12,13,fontArr[1],BLUE);
S	/*Displaying 3rd Digit of Lead value */	
S	draw_font_Lead(20,13,fontArr[2],BLUE);
S
S	digit_count = 0;
S	KeyPressed = 0;
S
N#else
N	if(digit_count<8)
N	{	
N			/*Displaying 1st Digit of Lead value */	
N			draw_font_Lead(4,13,fontArr[0],BLUE);
X			draw_font_Lead(4,13,fontArr[0],0x00003F);
N	}
N
N		if((digit_count >= 8)&&(digit_count <= 15))
N	{	
N			/*Displaying 2nd Digit of Lead value */	
N			draw_font_Lead(12,13,fontArr[1],BLUE);
X			draw_font_Lead(12,13,fontArr[1],0x00003F);
N	}
N	if((digit_count >15)&&(digit_count <= 23))
N	{							
N			/*Displaying 3rd Digit of Lead value */	
N		draw_font_Lead(20,13,fontArr[2],BLUE);
X		draw_font_Lead(20,13,fontArr[2],0x00003F);
N	}
N
N		
N	digit_count++;	
N	if(digit_count >= 24)
N	{
N		digit_count = 0;
N		KeyPressed = 0;
N	}	
N#endif
N
N}
N
N
N
N
N
NUint8 Detect_GPIO_interrupt(void)
Xunsigned char Detect_GPIO_interrupt(void)
N{
N	Uint8 size_of_packet;
X	unsigned char size_of_packet;
N	if(*GPIO_IFR0_ADDR)
X	if(*((ioport volatile unsigned*)0x1C10))
N		{
N			switch(*GPIO_IFR0_ADDR)
X			switch(*((ioport volatile unsigned*)0x1C10))
N			{/*reading the diagnostics register when PD or LED alarm occurs is not required since those alarms arrive only in diagnostics mode
N			.In diagnostics mode when diagnostics end interrupt occurs the diagnostics register is read*/
N			case 0x1000:/*when PD alarm interrupt occurs set SPO2 AFE failure flag and notify MB*/
N	//			Diagnostics_StsRegister|=(Uint8)ECG_AFE_Failure;
N			break;
N			case 0x2000:/*when LED alarm interrupt occurs set SPO2 AFE failure flag and notify MB*/
N	//			Diagnostics_StsRegister|=(Uint8)ECG_AFE_Failure;
N			break;
N			case 0x4000:/*on Diagnostics end read diagnostics register of AFE*/
N	//			AFE44xx_SPO2_Data_buf[6] = AFE44xx_Reg_Read(48);  //read DIAG
N			break;
N			default:
N			break;
N
N			}
N
N
N		}
N	//	else if(*GPIO_IFR1_ADDR)
N	//	{
N	//		switch(*GPIO_IFR1_ADDR)
N	//		{
N	//		case 0x0001:/*PACEOUT signal*/
N	//
N	//		break;
N	//		}
N	//	}
N		else if(*GPIO_IFR1_ADDR==0x0001)
X		else if(*((ioport volatile unsigned*)0x1C11)==0x0001)
N		{
N			/*code for reception of paceout signal*/
N
N			/*enable in the final board*/
N
N			Flag_StsRegister|=(Uint8)Pace_Detected;
X			Flag_StsRegister|=(unsigned char)0x08;
N			/*sent pace reset signal to reset latch*/
N			*GPIO_DOUT0_ADDR&=~0x0080;/*make it 0 to reset */
X			*((ioport volatile unsigned*)0x1C0A)&=~0x0080; 
N			wait(10);
N			//notify the pace detection to the motherboard
N			size_of_packet=pack(ECGData,ECG_DATA_LEAD1,1,Packet);
X			size_of_packet=pack(ECGData,0x01,1,Packet);
N			send_via_spi(size_of_packet,Packet);
N
N
N			*GPIO_DOUT0_ADDR|=0x0080;/*make it 1 after it gets reset*/
X			*((ioport volatile unsigned*)0x1C0A)|=0x0080; 
N
N
N		}
N	return 0;
N}
N
N
N
N/*-------------------------------------------------------------------------**
N**    Function name : ECG_SubSystem()                                      **
N**    Description : The function does the following:-                      **
N**                                                                         **
N**                                                                         **
N**                                                                         **
N**                     - Call the Process function                         **
N**                                                                         **
N**                     - Call the QRS Algorithm                            **
N**                                                                         **
N**                     - Call the LCD plot function for the current sample **
N**                                                                         **
N**                                                                         **
N**     Parameters  : None                                                  **
N** 	  Return	   : ECG system status                                     **
N**-------------------------------------------------------------------------*/
N
NECG_Status ECG_SubSystem()
N{
N	*GPIO_DOUT0_ADDR|=0x0008;
X	*((ioport volatile unsigned*)0x1C0A)|=0x0008;
N
N      /* Declare all the variables used in the function*/
N	ECG_Status Status = ECG_OK; /* Status Variable */
X	ECG_Status Status = 0;  
N
N	/* Count used to measure number of Frames*/  
N	static Uint16 frameCnt =0, LeadSts = 0;
X	static unsigned short frameCnt =0, LeadSts = 0;
N	static Uint16 Debaunce = 0;
X	static unsigned short Debaunce = 0;
N
N	Uint32 LeadFail;
X	unsigned long LeadFail;
N	Int16 Filterd_Out[8], PrevLeadSts;
X	short Filterd_Out[8], PrevLeadSts;
N	long LPF_2_hz_out=0;
N	long LPF_2_hz_out1;
N
N
N
N	frameCnt++;
N	if(frameCnt % 4 ==0)
N	{
N		frameCnt = 0;
N		LCDDataReady = 1;
N	}
N
N
N
N
N	SystemState_ProcessingECGdataStatus=SystemState_ProcessingECGdata_DetectingLeadoff;
N#if (NEW_PG_BOARD ==1)
X#if (1 ==1)
N    if ( LeadStatus & 0x0FF060)
N#else	
S	if ( LeadStatus & 0x1FE060)
N#endif
N
N	{
N		LeadOff_Flag = 1;
N		PrevLeadSts = ~LeadOffStatus;
N		PrevLeadSts &= 0x1FF;
N#if (NEW_PG_BOARD ==1)		
X#if (1 ==1)		
N	    LeadFail = (LeadStatus & ADS1298_V6) >> 4;
X	    LeadFail = (LeadStatus & 0x001000) >> 4;
N		LeadFail |= (LeadStatus & ADS1298_V1) >> 16;
X		LeadFail |= (LeadStatus & 0x080000) >> 16;
N		LeadFail |= (LeadStatus & ADS1298_V5432) >> 11;
X		LeadFail |= (LeadStatus & 0x078000) >> 11;
N		LeadFail |= (LeadStatus & ADS1298_LL_LA) >> 12;
X		LeadFail |= (LeadStatus & 0x006000) >> 12;
N		LeadFail |= (LeadStatus & ADS1298_RA) >> 5;
X		LeadFail |= (LeadStatus & 0x000020) >> 5;
N#else				
S		LeadFail = (LeadStatus & ADS1298_V6) >> 12;
S		LeadFail |= (LeadStatus & ADS1298_V1) >> 16;
S		LeadFail |= (LeadStatus & ADS1298_V5432) >> 11;
S		LeadFail |= (LeadStatus & ADS1298_LL_LA) >> 12;
S		LeadFail |= (LeadStatus & ADS1298_RA) >> 5;
N#endif
N		LeadOffStatus = (Uint16) ~LeadFail;
X		LeadOffStatus = (unsigned short) ~LeadFail;
N		LeadSts = (Uint16) LeadFail;
X		LeadSts = (unsigned short) LeadFail;
N		LeadSts &= 0x1FF;
N		LeadOffStatus &= 0x1FF;
N		Debaunce = 1500;
N
N	}
N	else
N	{ 
N		if ( Debaunce == 0 && LeadOffStatus != 0x1FF)
N		{
N			LeadOff_Flag = 1;
N			LeadOffStatus = 0x1FF;
N			LeadSts = 0;
N		}
N		else Debaunce--;
N	}
N
N
N
N
N
N
N    /*vai print the raw data*/
N //   printf("%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n",ECGData[0],ECGData[1],ECGData[2],ECGData[3],ECGData[4],ECGData[5],ECGData[6],ECGData[7]);
N
N    /* Filter the data*/
N    ECGData[7]=ECGData[7]*10;
N
N
N	SystemState_ProcessingECGdataStatus=SystemState_ProcessingECGdata_NotchFiltering;
N	#if ENABLE_NOTCH_FILTER
X	#if 1
N		ECG_ProcessCurrSample(ECGData, Filterd_Out);
N	#endif
N    /*Arrythmia*/
N
N
N    /*low pass filter 2 hz*/
N/*25/08/16
N   	ECGData[7]=ECGData[7]/8;
N
N	LPF_2_hz_out=Low_Pass_Filter(ECGData[7]);
N
N
N	LPF_2_hz_out1=Notch_Filter(LPF_2_hz_out);
N
N
N	long dc_buffer_output;
N	dc_buffer_output=DC_removal(LPF_2_hz_out1);
N*/
N
N
N
N//		ECGData[7]=ECGData[7]*10;
N//		LPF_2_hz_out1=Notch_Filter(ECGData[7]);
N
N//		ECGData[7]=LPF_2_hz_out1;
N	/*respiration function call*/
N	Calculate_Respiration(ECGData[7]);
N
N
N
N
N
N#if ENABLE_NOTCH_FILTER
X#if 1
N
N	if ( LeadSts & 0x01)
N	{
N		Filterd_Out[0] =0;
N		Filterd_Out[1] =0;
N	}
N	else if ( LeadSts & 0x02)
N	{
N		Filterd_Out[0] =0;
N	}
N	else if ( LeadSts & 0x04)
N	{
N		Filterd_Out[1] =0;
N	}
N	if ( LeadSts & 0x08)
N	{
N		Filterd_Out[2] =0;
N	}
N	if ( LeadSts & 0x10)
N	{
N		Filterd_Out[3] =0;
N	}
N	if ( LeadSts & 0x20)
N	{
N		Filterd_Out[4] =0;
N	}
N	if ( LeadSts & 0x40)
N	{
N		Filterd_Out[5] =0;
N	}
N	if ( LeadSts & 0x80)
N	{
N		Filterd_Out[6] =0;
N	}
N	if ( LeadSts & 0x100)
N	{
N		Filterd_Out[7] =0;
N	}
N#else
S	if ( LeadSts & 0x01)
S	{
S		ECGData[0] =0;
S		ECGData[1] =0;
S	}
S	else if ( LeadSts & 0x02)
S	{
S		ECGData[0] =0;
S	}
S	else if ( LeadSts & 0x04)
S	{
S		ECGData[1] =0;
S	}
S	if ( LeadSts & 0x08)
S	{
S		ECGData[2] =0;
S	}
S	if ( LeadSts & 0x10)
S	{
S		ECGData[3] =0;
S	}
S	if ( LeadSts & 0x20)
S	{
S		ECGData[4] =0;
S	}
S	if ( LeadSts & 0x40)
S	{
S		ECGData[5] =0;
S	}
S	if ( LeadSts & 0x80)
S	{
S		ECGData[6] =0;
S	}
S	if ( LeadSts & 0x100)
S	{
S		ECGData[7] =0;
S	}
N#endif
N
N#if DISP_LCD
X#if 1
N
N	/*  Display the HR in LCD*/
N	ECG_LCD_HRDisplay();	
N
N	if(LeadOff_Flag  )
N	{
N		ECG_LCD_LeadOffDisplay();		 
N//		PrevLeadOffStatus = LeadOffStatus;
N	}
N#endif
N
N
N
N
N	//update global variable for Heart rate from QRS detection(ready code)
N	Heart_Rate=(Uint8)QRS_Heart_Rate;
X	Heart_Rate=(unsigned char)QRS_Heart_Rate;
N
N
N
N	if ( (LeadSts & 0x04 ) == 0)
N	{
N		if ( (LeadSts & 0x01 ) == 0)
N		{
N	/* Call QRS algorithm interface */
N			QRS_Algorithm_Interface(Filterd_Out[1]);
N		}
N		else if ( (LeadSts & 0x02 ) == 0)
N		{
N	/* Call QRS algorithm interface */
N			QRS_Algorithm_Interface(Filterd_Out[0]);
N		}
N		else
N		{
N			QRS_Heart_Rate = 0;
N			ECG_Info.ECG_HeartRate = 0;
N		}
N	}
N
N
N	/*for testing purpose only*/
N
N	Uint8 size_of_packet;
X	unsigned char size_of_packet;
N	Int16 testdata[8]={0x1111,0x2222,0x3333,0,0,0,0,0};
X	short testdata[8]={0x1111,0x2222,0x3333,0,0,0,0,0};
N	unsigned int testdata16={0x2346};
N	unsigned char testdata8=0x23;
N	unsigned char testdata_8[4]={11,22,33,44};
N	unsigned char testdata_buf_8[5]={0x11,0x22,0x33,0x0,0x0};
N
N
N	SystemState_ProcessingECGdataStatus=SystemState_ProcessingECGdata_SendingECGDATAtoMB;
N
N/*update ECG values onto mother board at this point*/
N/*		*GPIO_DOUT0_ADDR = (*GPIO_DOUT0_ADDR & 0xFFFB);
N		wait(20);
N		size_of_packet=pack(ECGData,ECG_DATA_LEAD1,1,Packet);
N		send_via_spi(size_of_packet,Packet);
N		*GPIO_DOUT0_ADDR = (*GPIO_DOUT0_ADDR & 0xFFFB)|0x0004;
N*/
N
N
N
N#if DISP_LCD
X#if 1
N
N	if(LCDDataReady == 1)
N	{
N		LCDDataReady = 0; 
N		//Plot the current processed sample in the LCD
N
N		ECG_LCDPlotCurrSample(ECGData, LeadSelect);
N	}
N
N	if(frameCnt == 2)
N	{
N	
N		if ( KeyPressed == 1)
N		{
N			ECG_LCD_LeadDisplay(LeadSelect);
N		}
N		ReadKey =1;
N	}
N#endif
N
N	*GPIO_DOUT0_ADDR&=~0x0008;
X	*((ioport volatile unsigned*)0x1C0A)&=~0x0008;
N	return Status;
W "../src/ECGSystem_main.c" 432 7 variable "LPF_2_hz_out" was declared but never referenced
W "../src/ECGSystem_main.c" 433 7 variable "LPF_2_hz_out1" was declared but never referenced
W "../src/ECGSystem_main.c" 656 8 variable "size_of_packet" was declared but never referenced
W "../src/ECGSystem_main.c" 657 8 variable "testdata" was declared but never referenced
W "../src/ECGSystem_main.c" 658 15 variable "testdata16" was declared but never referenced
W "../src/ECGSystem_main.c" 659 16 variable "testdata8" was declared but never referenced
W "../src/ECGSystem_main.c" 660 16 variable "testdata_8" was declared but never referenced
W "../src/ECGSystem_main.c" 661 16 variable "testdata_buf_8" was declared but never referenced
N}
N
N
N//timer will not be used
Nvoid ECG_StartTimer0()
N{
N	/* Start the Timer 0*/
N	*CPU_TIM0_CTRL = *CPU_TIM0_CTRL | 0x0001; 
X	*((ioport volatile unsigned*)0x1810) = *((ioport volatile unsigned*)0x1810) | 0x0001; 
N}
N
N/*-------------------------------------------------------------------------
N**    Function name: commandMSPfromMBviaUART
N**    Description  : for testing purpose only
N**
N**    Parameters   : None
N** 	  Return	   : none
N**-------------------------------------------------------------------------*/
N
Nvoid commandMSPfromMBviaUART(unsigned char command_index)
N{	unsigned long j;
N	EVM5515_UART_putChar(0x24);
N
N	for (j=0;j<100000;j++);
N	for (j=0;j<100000;j++);
N	for (j=0;j<100000;j++);
N	for (j=0;j<100000;j++);
N
N	EVM5515_UART_putChar(command_index);
N
N	for (j=0;j<100000;j++);
N	for (j=0;j<100000;j++);
N	for (j=0;j<100000;j++);
N	for (j=0;j<100000;j++);
N
N	EVM5515_UART_putChar(0x00);
N	for (j=0;j<100000;j++);
N	for (j=0;j<100000;j++);
N	for (j=0;j<100000;j++);
N	for (j=0;j<100000;j++);
N
N	EVM5515_UART_putChar(0x2A);
N
N	for (j=0;j<100000;j++);
N	for (j=0;j<100000;j++);
N	for (j=0;j<100000;j++);
N	for (j=0;j<100000;j++);
N}
N
N/*--------------------------------------------------------------------------
N**     Function Name	: main()
N**     Description 		: Does the following:
N**     					  Initialize EVM C5515
N**     					  Initialize interrupts
N**     					  Check for ECG dataready, SPO2 dataready,UART and I2C
N**     					  interrupts in an infinite loop
N**
N**
N**  Intput Parameters   : None
N** 	Output Parameters	: None
N** 	Returns				: void
N**-------------------------------------------------------------------------*/
Nvoid main(void)
N{
N	unsigned int k;/*for SPO2*/
N    ECG_Status Status;
N    int timer_counter_test=0;
N    /* Call  Initialization function for the ECG system */
N    Status = ECG_C5505Init();
N    if(Status != ECG_OK)
X    if(Status != 0)
N    {
N		printf("ECG System Initialization FAIL\n");
N		while(1);
N    }
N	else
N	{
N//		printf("ECG System Initialization PASS\n");
N
N	}
N
N	wait(1000);
N
N	/* Display the ECG startup screen in LCD */
N#if DISP_LCD
X#if 1
N    ECG_DisplayLCDStartUpScreen();
N#endif
N	asm("\tBIT (ST1, #ST1_INTM) = #0");	  		/*Enable GLobal Int.Mask*/
N
N	/* Start the Timer0*/
N	/*timer will not be used*/
N
N	/*SAR will not be used*/
N
N
N	/*Enable SAR INT1 ,INT0 and UART interrupt*/
N	//*CPU_IER0_ADDR = *CPU_IER0_ADDR | 0x105C;
N	*CPU_IER0_ADDR = *CPU_IER0_ADDR | 0x0010;
X	*((volatile unsigned*)0x0000) = *((volatile unsigned*)0x0000) | 0x0010;
N	ECG_StartTimer0();
N	/*enable GPIO interrupt */
N	//*CPU_IER1_ADDR = *CPU_IER1_ADDR | 0x0020;
N
N
N	/* Infinite loop - the ECG processing will be done according to timer*/
N	KeyPressed = 0;
N
N
N	while(1)
N	{
N		/*check for ECG dataready interrupt*/
N
N		if(GPIO_interrupt_flag)
N		{
N			GPIO_interrupt_flag=0;
N			Detect_GPIO_interrupt();
N
N		}
N		if(QRSDataReady == 1)
N		{
N			QRSDataReady =0;
N			ECG_SubSystem();
N		}
N
N		if(buf1_ready_to_process==1)
N		{	buf1_ready_to_process=0;
N		//	arrythmia_detection(ECG_arrythmia_buf1,1);
N
N		}
N		if(buf2_ready_to_process==1)
N		{	buf2_ready_to_process=0;
N		//	arrythmia_detection(ECG_arrythmia_buf2,0);
N		}
N
N
N
N		if ( ReadKey )
N		{
N			KeyDetect();
N			ReadKey = 0;
N		}
N		/*check for uart interrupt*/
N		if(UART_dataready_flag==1)
N		{	UART_dataready_flag=0;
N			receive_uartdata();
N		}
N		/*check for I2C interrupt*/
N		if(I2C_dataready_flag==1)
N		{
N			I2C_dataready_flag=0;
N			receive_I2Cdata();
N		}
N		/*check for SPO2 data ready interrupt*/
N		if(SPO2_dataready_flag==1)
N		{
N			SPO2_dataready_flag=0;
N			AFE44xx_SPO2_Data_buf[0] = AFE44xx_Reg_Read(42);  //read RED Data
N			AFE44xx_SPO2_Data_buf[1] = AFE44xx_Reg_Read(43);  //read Ambient data
N			AFE44xx_SPO2_Data_buf[2] = AFE44xx_Reg_Read(44);  //read IR Data
N			AFE44xx_SPO2_Data_buf[3] = AFE44xx_Reg_Read(45);  //read Ambient Data
N			AFE44xx_SPO2_Data_buf[4] = AFE44xx_Reg_Read(46);  //read RED - Ambient Data
N			AFE44xx_SPO2_Data_buf[5] = AFE44xx_Reg_Read(47);  //read IR - Ambient Data
N
N
N
N			/*check for faults*/
N//			if(AFE44xx_SPO2_Data_buf[6]&0x1FFF)
N//			{
N//				Uint8 datapack_MSP[4]={0};
N//				Diagnostics_StsRegister|=0x02;/*representing AFE failure*/
N//				pack_to_MSP(Diagnostics_StsRegister,Diagnostics_status_request,datapack_MSP);
N//				send_via_uart(4,datapack_MSP);
N//			}
N
N
N
N			/*moving window average filter*/
N
N			/*shifting the values */
N			for(k=0;k<(FILTER_LENGTH_SPO2-1);k++)
X			for(k=0;k<(5-1);k++)
N			{
N				raw_RED_buffer[k]=raw_RED_buffer[k+1];
N				raw_IR_buffer[k]=raw_IR_buffer[k+1];
N
N			}
N
N			/*updating last value of buffer*/
N			raw_RED_buffer[FILTER_LENGTH_SPO2-1]= (AFE44xx_SPO2_Data_buf[4] & 0x003FFFFF);
X			raw_RED_buffer[5-1]= (AFE44xx_SPO2_Data_buf[4] & 0x003FFFFF);
N			raw_IR_buffer[FILTER_LENGTH_SPO2-1] = (AFE44xx_SPO2_Data_buf[5] & 0x003FFFFF);
X			raw_IR_buffer[5-1] = (AFE44xx_SPO2_Data_buf[5] & 0x003FFFFF);
N
N
N
N			/*moving window average filter */
N			moving_average_filter();
N
N
N			SPO2=calulate_AC_to_DC_Ratio(adc_value,adc_value1);
N			//Enable_AFE44xx_DRDY_Interrupt();			// Enable DRDY interrupt
N
N			AFE44xx_SPO2_Data_buf[0] = 0;
N			AFE44xx_SPO2_Data_buf[1] = 0;
N			AFE44xx_SPO2_Data_buf[2] = 0;
N			AFE44xx_SPO2_Data_buf[3] = 0;
N			AFE44xx_SPO2_Data_buf[4] = 0;
N			AFE44xx_SPO2_Data_buf[5] = 0;
N			AFE44xx_SPO2_Data_buf[6] = 0;
N		}
N
N	}
N	    
W "../src/ECGSystem_main.c" 764 9 variable "timer_counter_test" was declared but never referenced
N}
N   
N
N
N//lcd will not be used
Nvoid ECG_LCD_HRDisplay(void)	
N{
N
N	extern Uint16 QRS_Heart_Rate; 
X	extern unsigned short QRS_Heart_Rate; 
N   	Uint16 HR;
X   	unsigned short HR;
N	Uint16 new_val; 
X	unsigned short new_val; 
N	static Uint16 PrevHr = 0;
X	static unsigned short PrevHr = 0;
N	static Uint16 digit1_count = 0;  
X	static unsigned short digit1_count = 0;  
N
N	if(HR_flag)
N	{
N		if(PrevHr != QRS_Heart_Rate)
N		{
N		if(Initial_HR_Flag)
N		{
N			HR = QRS_Heart_Rate;
N			digit1 = HR/100;
N			new_val = HR % 100;
N 			digit2 = new_val / 10;
N			digit3 = new_val % 10;
N			Initial_HR_Flag = 0;
N		}
N		if(digit1_count<8)
N		{	
N		//	digit1 = HR/100;				/*getting the first digit of HR value*/
N 			draw_font_HR(82,13,(digit1+48),BLUE);/*displaying it by adding ASCII value*/
X 			draw_font_HR(82,13,(digit1+48),0x00003F); 
N		}
N
N 		if((digit1_count >= 8)&&(digit1_count <= 15))
N		{	
N		//	new_val = HR % 100;
N 		//	digit2 = new_val / 10;				/*getting the second digit of HR value*/
N 			draw_font_HR(90,13,(digit2+48),BLUE);/*displaying it by adding ASCII value*/
X 			draw_font_HR(90,13,(digit2+48),0x00003F); 
N		}
N		if((digit1_count >15)&&(digit1_count <= 23))
N		{							
N 		//	digit3 = new_val % 10;				/*getting the third digit of HR value*/
N			draw_font_HR(98,13,(digit3+48),BLUE);/*displaying it by adding ASCII value*/
X			draw_font_HR(98,13,(digit3+48),0x00003F); 
N		}
N		
N		digit1_count++;	
N		if(digit1_count >= 24)
N		{
N			digit1_count = 0;
N			HR_flag = 0;	
N			PrevHr = QRS_Heart_Rate;	
N		}	
N	}
N	}
N
N}
N
N
N//lcd will not be used
Nvoid ECG_LCD_LeadOffDisplay(void)	
N{
N	if(LeadOffStatus != 0x01FF)
N 		{
N			if(LeadOffStatus == ELECTRODE_RA)
X			if(LeadOffStatus == 0x01FE)
N				{
N					/*Electrode  RA is OFF*/
N					if(lead_digit_count<8)
N					{
N						draw_font_HR(45,13,82,RED);
X						draw_font_HR(45,13,82,0x3F0000);
N						lead_digit_count++;
N					}
N					if(lead_digit_count >= 8)
N					{
N						draw_font_HR(53,13,65,RED);
X						draw_font_HR(53,13,65,0x3F0000);
N						lead_digit_count++;
N						if(lead_digit_count >= 16)
N						{
N							lead_digit_count = 0;
N							LeadOff_Flag = 0;
N						}
N					}
N				}
N			else if(LeadOffStatus == ELECTRODE_LA)
X			else if(LeadOffStatus == 0x01FD)
N				{
N					/*Electrode  LA is OFF*/
N					if(lead_digit_count<8)
N					{
N						draw_font_HR(45,13,76,RED);
X						draw_font_HR(45,13,76,0x3F0000);
N						lead_digit_count++;
N					}
N					if(lead_digit_count >= 8)
N					{
N						draw_font_HR(53,13,65,RED);
X						draw_font_HR(53,13,65,0x3F0000);
N						lead_digit_count++;
N						if(lead_digit_count >= 16)
N						{
N							lead_digit_count = 0;
N							LeadOff_Flag = 0;
N						}
N					}
N				}
N			else if(LeadOffStatus == ELECTRODE_LL)
X			else if(LeadOffStatus == 0x01FB)
N				{
N					/*Electrode  LL is OFF*/
N				if(lead_digit_count<8)
N					{
N						draw_font_HR(45,13,76,RED);
X						draw_font_HR(45,13,76,0x3F0000);
N						lead_digit_count++;
N					}
N					if(lead_digit_count >= 8)
N					{
N						draw_font_HR(53,13,76,RED);
X						draw_font_HR(53,13,76,0x3F0000);
N						lead_digit_count++;
N						if(lead_digit_count >= 16)
N						{
N							lead_digit_count = 0;
N							LeadOff_Flag = 0;
N						}
N					}	
N				}
N			
N			else if(LeadOffStatus == ELECTRODE_V1)
X			else if(LeadOffStatus == 0x01F7)
N				{
N					/*Electrode  V1 is OFF*/
N					if(lead_digit_count < 8)
N					{
N						draw_font_HR(45,13,86,RED);
X						draw_font_HR(45,13,86,0x3F0000);
N						lead_digit_count++;
N					}
N					if(lead_digit_count >= 8)
N					{
N						draw_font_HR(53,13,49,RED);
X						draw_font_HR(53,13,49,0x3F0000);
N						lead_digit_count++;
N						if(lead_digit_count >= 16)
N						{
N							lead_digit_count = 0;
N							LeadOff_Flag = 0;
N						}
N					}	
N				}
N			else if(LeadOffStatus == ELECTRODE_V2)
X			else if(LeadOffStatus == 0x01EF)
N				{
N					/*Electrode  V2 is OFF*/
N					if(lead_digit_count < 8)
N					{
N						draw_font_HR(45,13,86,RED);
X						draw_font_HR(45,13,86,0x3F0000);
N						lead_digit_count++;
N					}
N					if(lead_digit_count >= 8)
N					{
N						draw_font_HR(53,13,50,RED);
X						draw_font_HR(53,13,50,0x3F0000);
N						lead_digit_count++;
N						if(lead_digit_count >= 16)
N						{
N							lead_digit_count = 0;
N							LeadOff_Flag = 0;
N						}
N					}		
N				}
N			else if(LeadOffStatus == ELECTRODE_V3)
X			else if(LeadOffStatus == 0x01DF)
N				{
N					/*Electrode  V3 is OFF*/
N					if(lead_digit_count < 8)
N					{
N						draw_font_HR(45,13,86,RED);
X						draw_font_HR(45,13,86,0x3F0000);
N						lead_digit_count++;
N					}
N					if(lead_digit_count >= 8)
N					{
N						draw_font_HR(53,13,51,RED);
X						draw_font_HR(53,13,51,0x3F0000);
N						lead_digit_count++;
N						if(lead_digit_count >= 16)
N						{
N							lead_digit_count = 0;
N							LeadOff_Flag = 0;
N						}
N					}	
N				}
N			else if(LeadOffStatus == ELECTRODE_V4)
X			else if(LeadOffStatus == 0x01BF)
N				{
N					/*Electrode  V4 is OFF*/
N					if(lead_digit_count < 8)
N					{
N						draw_font_HR(45,13,86,RED);
X						draw_font_HR(45,13,86,0x3F0000);
N						lead_digit_count++;
N					}
N					if(lead_digit_count >= 8)
N					{
N						draw_font_HR(53,13,52,RED);
X						draw_font_HR(53,13,52,0x3F0000);
N						lead_digit_count++;
N						if(lead_digit_count >= 16)
N						{
N							lead_digit_count = 0;
N							LeadOff_Flag = 0;
N						}
N					}		
N				}
N			else if(LeadOffStatus == ELECTRODE_V5)
X			else if(LeadOffStatus == 0x017F)
N				{
N					/*Electrode  V5 is OFF*/
N					if(lead_digit_count < 8)
N					{
N						draw_font_HR(45,13,86,RED);
X						draw_font_HR(45,13,86,0x3F0000);
N						lead_digit_count++;
N					}
N					if(lead_digit_count >= 8)
N					{
N						draw_font_HR(53,13,53,RED);
X						draw_font_HR(53,13,53,0x3F0000);
N						lead_digit_count++;
N						if(lead_digit_count >= 16)
N						{
N							lead_digit_count = 0;
N							LeadOff_Flag = 0;
N						}
N					}	
N				}
N			else if(LeadOffStatus == ELECTRODE_V6)
X			else if(LeadOffStatus == 0x00FF)
N				{
N					/*Electrode  V6 is OFF*/
N					if(lead_digit_count < 8)
N					{
N						draw_font_HR(45,13,86,RED);
X						draw_font_HR(45,13,86,0x3F0000);
N						lead_digit_count++;
N					}
N					if(lead_digit_count >= 8)
N					{
N						draw_font_HR(53,13,54,RED);
X						draw_font_HR(53,13,54,0x3F0000);
N						lead_digit_count++;
N						if(lead_digit_count >= 16)
N						{
N							lead_digit_count = 0;
N							LeadOff_Flag = 0;
N						}
N					}	
N				}
N			
N
N			else 
N				/*Red circle indicating 2 or more leads are off*/
N				{
N					if(lead_digit_count < 8)
N					{
N						draw_font_HR(45,13,38,RED);
X						draw_font_HR(45,13,38,0x3F0000);
N						LCD_clear_window(53,13,61,21);
N						lead_digit_count++;
N					}
N					if(lead_digit_count >= 8)
N					{
N						lead_digit_count = 0;
N						LeadOff_Flag = 0;
N					}
N				}	
N 		}
N	else
N		/*Green circle indicating all the leads are connected*/
N		{
N			//LCD_clear_window(45,13,60,20);
N			if(lead_digit_count < 8)
N			{
N	/*Green Circle indicating Lead_Off Status*/
N				draw_font_HR(45,13,38,GREEN);
X				draw_font_HR(45,13,38,0x003F00);
N				LCD_clear_window(53,13,61,21);
N				lead_digit_count++;
N			}
N			if(lead_digit_count >= 8)
N			{
N				lead_digit_count = 0;
N				LeadOff_Flag = 0;
N			}
N		}
N}
N
N
N/*EOF*/
