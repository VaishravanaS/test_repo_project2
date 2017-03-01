L 1 "../src/ECG_Filter_Co-efficients.c"
N /******************************************************************************
N**File Name			: ECG_Filter_Co-efficeints.c
N**File Description	:
N**Created By		:
N**Creation Date		:
N**Last modified date:
N**Remarks			:
N*******************************************************************************/
N
N
N#include "ECGGlobals.h"
L 1 "../inc/ECGGlobals.h" 1
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
L 12 "../src/ECG_Filter_Co-efficients.c" 2
N#include "tistdtypes.h"
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
N#define _TI_STD_TYPES
N
N/*
N * This '#ifndef STD_' is needed to protect from duplicate definitions
N * of Int, Uns, etc. in DSP/BIOS v4.x (e.g. 4.90, 4.80) since these versions
N * of DSP/BIOS did not contain the '#ifndef_TI_STD_TYPES' logic.
N */
N#ifndef STD_
N
N/*
N * Aliases for standard C types
N */
Ntypedef int                 Int;
Ntypedef unsigned            Uns;
Ntypedef char                Char;
N
N/* pointer to null-terminated character sequence */
Ntypedef char                *String;
N                            
Ntypedef void                *Ptr;       /* pointer to arbitrary type */
N                            
Ntypedef unsigned short      Bool;       /* boolean */
N
N#endif /* STD_ */
N
N/*
N * Uint8, Uint16, Uint32, etc are defined to be "smallest unit of
N * available storage that is large enough to hold unsigned or integer
N * of specified size".
N */
N #ifndef TRUE
N  #define TRUE  1
N  #define FALSE 0
N#endif
N
N
N/* Handle the 6x ISA */
N#if defined(_TMS320C6X)
X#if 0L
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
N#elif defined(_TMS320C5XX) || defined(__TMS320C55X__) || defined(_TMS320C28X)
X#elif 0L || 1L || 0L
N    /* Unsigned integer definitions (32bit, 16bit, 8bit) follow... */
N    typedef unsigned long   Uint32;
N    typedef unsigned short  Uint16;
N    typedef unsigned char   Uint8;
N
N    /* Signed integer definitions (32bit, 16bit, 8bit) follow... */
N    typedef long            Int32;
N    typedef short           Int16;
N    typedef char            Int8;
N
N#else
S    /* Other ISAs not supported */
S    #error <tistdtypes.h> is not supported for this target
N#endif  /* defined(_6x_) */
N
N#endif  /* _TI_STD_TYPES */
L 13 "../src/ECG_Filter_Co-efficients.c" 2
N
N#define SPS 500
N
NInt16 CoeffBuf[FILTERORDER] = {             
XInt16 CoeffBuf[(351)] = {             
N
N#if NOTCHFILTERSEL
X#if 0
S/* Coeff for Notch @ 60Hz for 500SPS/60Hz Notch coeff13102008*/
S       28,     23,      2,    -25,    -43,    -38,     -9,     29,     56,
S       54,     20,    -29,    -68,    -71,    -34,     26,     76,     87,
S       50,    -18,    -81,   -102,    -68,      7,     82,    115,     86,
S        8,    -78,   -124,   -103,    -24,     71,    130,    119,     42,
S      -59,   -131,   -132,    -61,     44,    127,    141,     78,    -27,
S     -118,   -145,    -93,      9,    105,    144,    104,      9,    -89,
S     -138,   -111,    -25,     71,    126,    112,     39,    -52,   -111,
S     -107,    -48,     33,     91,     97,     51,    -17,    -69,    -80,
S      -48,      4,     47,     59,     38,      3,    -25,    -33,    -21,
S       -4,      6,      4,     -3,     -3,      8,     25,     33,     18,
S      -16,    -53,    -67,    -42,     16,     79,    106,     74,     -8,
S      -99,   -145,   -113,    -10,    112,    184,    159,     39,   -116,
S     -220,   -209,    -77,    110,    251,    261,    124,    -93,   -274,
S     -314,   -179,     65,    288,    364,    240,    -24,   -290,   -409,
S     -306,    -27,    280,    446,    372,     88,   -257,   -474,   -437,
S     -157,    220,    490,    498,    232,   -171,   -493,   -553,   -310,
S      110,    481,    597,    389,    -39,   -455,   -631,   -464,    -40,
S      414,    650,    535,    125,   -360,   -655,   -597,   -212,    293,
S      644,    648,    298,   -217,   -618,   -686,   -381,    133,    577,
S      709,    456,    -45,   -523,  32051,   -523,    -45,    456,    709,
S      577,    133,   -381,   -686,   -618,   -217,    298,    648,    644,
S      293,   -212,   -597,   -655,   -360,    125,    535,    650,    414,
S      -40,   -464,   -631,   -455,    -39,    389,    597,    481,    110,
S     -310,   -553,   -493,   -171,    232,    498,    490,    220,   -157,
S     -437,   -474,   -257,     88,    372,    446,    280,    -27,   -306,
S     -409,   -290,    -24,    240,    364,    288,     65,   -179,   -314,
S     -274,    -93,    124,    261,    251,    110,    -77,   -209,   -220,
S     -116,     39,    159,    184,    112,    -10,   -113,   -145,    -99,
S       -8,     74,    106,     79,     16,    -42,    -67,    -53,    -16,
S       18,     33,     25,      8,     -3,     -3,      4,      6,     -4,
S      -21,    -33,    -25,      3,     38,     59,     47,      4,    -48,
S      -80,    -69,    -17,     51,     97,     91,     33,    -48,   -107,
S     -111,    -52,     39,    112,    126,     71,    -25,   -111,   -138,
S      -89,      9,    104,    144,    105,      9,    -93,   -145,   -118,
S      -27,     78,    141,    127,     44,    -61,   -132,   -131,    -59,
S       42,    119,    130,     71,    -24,   -103,   -124,    -78,      8,
S       86,    115,     82,      7,    -68,   -102,    -81,    -18,     50,
S       87,     76,     26,    -34,    -71,    -68,    -29,     20,     54,
S       56,     29,     -9,    -38,    -43,    -25,      2,     23,     28
S
N#else
N/* Coeff for Notch @ 50Hz @ 500 SPS*/
N	      -28,    -26,    -11,     12,     35,     47,     41,     17,    -18,
N	      -50,    -65,    -56,    -22,     24,     65,     83,     70,     28,
N	      -29,    -78,   -100,    -84,    -33,     34,     91,    115,     95,
N	       37,    -38,   -101,   -128,   -105,    -41,     41,    110,    137,
N	      112,     43,    -44,   -115,   -144,   -117,    -45,     45,    118,
N	      146,    118,     45,    -45,   -117,   -144,   -116,    -44,     43,
N	      112,    137,    109,     41,    -40,   -103,   -125,    -99,    -37,
N	       36,     91,    108,     84,     31,    -29,    -73,    -86,    -65,
N	      -23,     22,     52,     59,     42,     14,    -12,    -27,    -26,
N	      -16,     -4,      1,     -2,    -11,    -15,     -8,     11,     35,
N	       52,     49,     22,    -24,    -71,    -97,    -86,    -36,     39,
N	      110,    145,    126,     51,    -54,   -150,   -196,   -167,    -67,
N	       70,    193,    249,    210,     83,    -87,   -236,   -302,   -253,
N	     -100,    103,    279,    356,    297,    117,   -120,   -322,   -409,
N	     -339,   -133,    136,    364,    460,    380,    148,   -151,   -404,
N	     -508,   -419,   -163,    166,    441,    554,    455,    176,   -179,
N	     -475,   -595,   -487,   -188,    191,    505,    631,    515,    199,
N	     -201,   -530,   -661,   -539,   -208,    209,    551,    685,    558,
N	      214,   -215,   -566,   -703,   -571,   -219,    219,    576,    714,
N	      578,    221,   -221,   -580,  32051,   -580,   -221,    221,    578,
N	      714,    576,    219,   -219,   -571,   -703,   -566,   -215,    214,
N	      558,    685,    551,    209,   -208,   -539,   -661,   -530,   -201,
N	      199,    515,    631,    505,    191,   -188,   -487,   -595,   -475,
N	     -179,    176,    455,    554,    441,    166,   -163,   -419,   -508,
N	     -404,   -151,    148,    380,    460,    364,    136,   -133,   -339,
N	     -409,   -322,   -120,    117,    297,    356,    279,    103,   -100,
N	     -253,   -302,   -236,    -87,     83,    210,    249,    193,     70,
N	      -67,   -167,   -196,   -150,    -54,     51,    126,    145,    110,
N	       39,    -36,    -86,    -97,    -71,    -24,     22,     49,     52,
N	       35,     11,     -8,    -15,    -11,     -2,      1,     -4,    -16,
N	      -26,    -27,    -12,     14,     42,     59,     52,     22,    -23,
N	      -65,    -86,    -73,    -29,     31,     84,    108,     91,     36,
N	      -37,    -99,   -125,   -103,    -40,     41,    109,    137,    112,
N	       43,    -44,   -116,   -144,   -117,    -45,     45,    118,    146,
N	      118,     45,    -45,   -117,   -144,   -115,    -44,     43,    112,
N	      137,    110,     41,    -41,   -105,   -128,   -101,    -38,     37,
N	       95,    115,     91,     34,    -33,    -84,   -100,    -78,    -29,
N	       28,     70,     83,     65,     24,    -22,    -56,    -65,    -50,
N	      -18,     17,     41,     47,     35,     12,    -11,    -26,    -28
N
N#endif
N};
N
N/*   Q15 Multiplication Format  */
Nconst Int16 AlisnCoeffBuf[51] = {
N        6,      7,      8,      9,     10,     10,     11,     12,     13,
N       14,     14,     15,     16,     17,     17,     18,     18,     19,
N       19,     20,     20,     20,     21,     21,     21,     21,     21,
N       21,     21,     20,     20,     20,     19,     19,     18,     18,
N       17,     17,     16,     15,     14,     14,     13,     12,     11,
N       10,     10,      9,      8,      7,      6
N};
N
N
N
N
N#if RAW_DATA
SInt16 Sample_Data[100000] ;
N#endif
N
N/*EOF*/
N
N
N
N
N
N
N
N
