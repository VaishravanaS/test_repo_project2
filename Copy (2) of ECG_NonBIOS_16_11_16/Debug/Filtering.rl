L 1 "../src/Filtering.c"
N /******************************************************************************
N**File Name			: Filtering.c
N**File Description	:
N**Author    		: Vuon1
N**Creation Date		: 05-Aug-2016
N**Last modified date:
N**Remarks			:
N*******************************************************************************/
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
L 10 "../src/Filtering.c" 2
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
L 11 "../src/Filtering.c" 2
N
N
N
N
N#define FILTER_LENGTH   201
N
N#define FILTER_LENGTH1   10
N
Nextern unsigned char Notch_Filter_60_Hz;
N
N	/*dc removal filter*/
Nlong DC_removal(long LPF_2_hz_out1)
N{
N
N
N
N
N	static long dc_buf_in[2],dc_buf_out[2];
N	//LPF_2_hz_out1=LPF_2_hz_out1/500;
N	Uint8 k;
N		   for (k=0;k<(2-1);k++)
N			{
N			   dc_buf_in[k]=dc_buf_in[k+1];
N			}
N		   dc_buf_in[1]=LPF_2_hz_out1;
N
N			for (k=0;k<(2-1);k++)
N			{
N				dc_buf_out[k]=dc_buf_out[k+1];
N			}
N
N			dc_buf_out[1]=(1000*dc_buf_in[1]-1000*dc_buf_in[0]+992*dc_buf_out[0])/1000;
N		//	dc_buf_out[1]=dc_buf_out[1]/8;
Nreturn dc_buf_out[1];
N}
N
N
N
N/*notch filter to remove 50 hz noise -works with 500sps*/
Nlong Notch_Filter(long LPF_2_hz_out)
N{
N	Int16 Notch50[FILTER_LENGTH1]={1,1,1,1,1,1,1,1,1,1};
X	Int16 Notch50[10]={1,1,1,1,1,1,1,1,1,1};
N	static Int16 in_buf1[FILTER_LENGTH1]={ 0,0,0,0,0,0,0,0,0,0
X	static Int16 in_buf1[10]={ 0,0,0,0,0,0,0,0,0,0
N								//	0,0,0,0,0,0,0,0,0,0
N								/*	0,0,0,0,0,0,0,0,0,
N									0,0,0,0,0,0,0,0,0,
N									0,0,0,0,0,0,0,0,0,
N									0,0,0,0,0,0,0,0,0,
N									0,0,0,0,0,0,0,0,0,
N									0,0,0,0,0,0,0,0,0,
N									0,0,0,0,0,0,0,0,0,
N									0,0,0,0,0,0,0,0,0,
N									0,0,0,0,0,0,0,0,0,
N									0,0,0,0,0,0,0,0,0,
N									0,0,0,0,0,0,0,0,0,
N									0,0,0,0,0,0,0,0,0,
N									0,0,0,0,0,0,0,0,0,
N									0,0,0,0,0,0,0,0,0,
N									0,0,0,0,0,0,0,0,0,
N									0,0,0,0,0,0,0,0,0,
N									0,0,0,0,0,0,0,0,0,
N									0,0,0,0,0,0,0,0,0,
N									0,0,0,0,0,0,0,0,0,
N									0,0,0,0,0,0,0,0,0,
N									0,0,0,0,0,0,0,0,0,
N									0,0,0,0,0,0,0,0,0,
N									0,0,0,0,0,0,0,0,0,
N									0,0,0,0,0,0,0,0,0,
N									0,0,0,0,0,0,0,0,0,
N									0,0,0,0,0,0,0,0,0,
N									0,0,0,0,0,0,0,0,0,
N									0,0,0,0,0,0,0,0,0,
N									0,0,0,0,0,0,0,0,0,
N									0,0,0,0,0,0,0,0,0,
N									0,0,0,0,0,0,0,0,0,
N									0,0,0,0,0,0,0,0,0,
N									0,0,0,0,0,0,0,0,0,
N									0,0,0,0,0,0,0,0,0,
N									0,0,0,0,0,0,0,0,0,
N									0,0,0,0,0,0,0,0,0,
N									0,0,0,0,0,0,0,0,0*/
N
N		};
N
N
N
N	long LPF_2_hz_out1;
N
N	//LPF_2_hz_out=LPF_2_hz_out/4;
N	//LPF_2_hz_out=LPF_2_hz_out/2;
N	Uint8 k1;
N	{
N
N			   for (k1=0;k1<(FILTER_LENGTH1-1);k1++)
X			   for (k1=0;k1<(10-1);k1++)
N				{
N					in_buf1[k1]=in_buf1[k1+1];
N				}
N				#if ENABLE_NOTCH_FILTER
S				in_buf1[FILTER_LENGTH1-1]=LPF_2_hz_out;
N				#else
N				in_buf1[FILTER_LENGTH1-1]=LPF_2_hz_out;
X				in_buf1[10-1]=LPF_2_hz_out;
N				#endif
N				// a=0;
N
N				for( k1=0;k1<FILTER_LENGTH1;k1++)
X				for( k1=0;k1<10;k1++)
N				{
N
N
N					switch(Notch_Filter_60_Hz)
N					{	case 0:
N						//LPF_2_hz_out1+=(Int32)(Int16)in_buf1[FILTER_LENGTH1-(k1+1)]*(Int32)(Int16)Notch[k1];
N						break;
N
N						case 1:
N							LPF_2_hz_out1+=(Int32)(Int16)in_buf1[FILTER_LENGTH1-(k1+1)]*(Int32)(Int16)Notch50[k1];
X							LPF_2_hz_out1+=(Int32)(Int16)in_buf1[10-(k1+1)]*(Int32)(Int16)Notch50[k1];
N						break;
N						default:
N						break;
N
N
N
N					}
N				}
N
N
N	}
N	return LPF_2_hz_out1;
N}
N
Nlong Low_Pass_Filter(int ecgdata)
N{
N	int Low_Pass_Filter_2Hz_coeffs[FILTER_LENGTH]={	7,	7,	7,	8,	8,	9,	9,	10,	11,	12,	12,	13,	15,	16,	17,	18,	20,	21,	23,	25,	27,	29,	31,	34,	36,	39,	41,	44,	47,	50,	54,	57,	61,	64,	68,	72,	76,	80,	85,	89,	94,	99,	103,	108,	113,	119,	124,	129,	135,	140,	146,	151,	157,	163,	169,	175,	181,	186,	192,	198,	204,	210,	216,	222,	228,	234,	240,	246,	251,	257,	262,	268,	273,	278,	284,	289,	294,	298,	303,	307,	312,	316,	320,	324,	327,	331,	334,	337,	340,	343,	345,	347,	349,	351,	352,	354,	355,	356,	356,	357,	357,	357,	356,	356,	355,	354,	352,	351,	349,	347,	345,	343,	340,	337,	334,	331,	327,	324,	320,	316,	312,	307,	303,	298,	294,	289,	284,	278,	273,	268,	262,	257,	251,	246,	240,	234,	228,	222,	216,	210,	204,	198,	192,	186,	181,	175,	169,	163,	157,	151,	146,	140,	135,	129,	124,	119,	113,	108,	103,	99,	94,	89,	85,	80,	76,	72,	68,	64,	61,	57,	54,	50,	47,	44,	41,	39,	36,	34,	31,	29,	27,	25,	23,	21,	20,	18,	17,	16,	15,	13,	12,	12,	11,	10,	9,	9,	8,	8,	7,	7,	7
X	int Low_Pass_Filter_2Hz_coeffs[201]={	7,	7,	7,	8,	8,	9,	9,	10,	11,	12,	12,	13,	15,	16,	17,	18,	20,	21,	23,	25,	27,	29,	31,	34,	36,	39,	41,	44,	47,	50,	54,	57,	61,	64,	68,	72,	76,	80,	85,	89,	94,	99,	103,	108,	113,	119,	124,	129,	135,	140,	146,	151,	157,	163,	169,	175,	181,	186,	192,	198,	204,	210,	216,	222,	228,	234,	240,	246,	251,	257,	262,	268,	273,	278,	284,	289,	294,	298,	303,	307,	312,	316,	320,	324,	327,	331,	334,	337,	340,	343,	345,	347,	349,	351,	352,	354,	355,	356,	356,	357,	357,	357,	356,	356,	355,	354,	352,	351,	349,	347,	345,	343,	340,	337,	334,	331,	327,	324,	320,	316,	312,	307,	303,	298,	294,	289,	284,	278,	273,	268,	262,	257,	251,	246,	240,	234,	228,	222,	216,	210,	204,	198,	192,	186,	181,	175,	169,	163,	157,	151,	146,	140,	135,	129,	124,	119,	113,	108,	103,	99,	94,	89,	85,	80,	76,	72,	68,	64,	61,	57,	54,	50,	47,	44,	41,	39,	36,	34,	31,	29,	27,	25,	23,	21,	20,	18,	17,	16,	15,	13,	12,	12,	11,	10,	9,	9,	8,	8,	7,	7,	7
N	};
N
N
N	static Int16 in_buf[FILTER_LENGTH]={  0,0,0,0,0,0,0,0,0,
X	static Int16 in_buf[201]={  0,0,0,0,0,0,0,0,0,
N						0,0,0,0,0,0,0,0,0,
N						0,0,0,0,0,0,0,0,0,
N						0,0,0,0,0,0,0,0,0,
N						0,0,0,0,0,0,0,0,0,
N						0,0,0,0,0,0,0,0,0,
N						0,0,0,0,0,0,0,0,0,
N						0,0,0,0,0,0,0,0,0,
N						0,0,0,0,0,0,0,0,0,
N						0,0,0,0,0,0,0,0,0,
N						0,0,0,0,0,0,0,0,0,
N						0,0,0,0,0,0,0,0,0,
N						0,0,0,0,0,0,0,0,0,
N						0,0,0,0,0,0,0,0,0,
N						0,0,0,0,0,0,0,0,0,
N						0,0,0,0,0,0,0,0,0,
N						0,0,0,0,0,0,0,0,0,
N						0,0,0,0,0,0,0,0,0,
N						0,0,0,0,0,0,0,0,0,
N						0,0,0,0,0,0,0,0,0,
N						0,0,0,0,0,0,0,0,0,
N						0,0,0,0,0,0,0,0,0,
N						0,0,0
N	};
N	Uint16 k;
N	long LPF_2_hz_out=0;
N	{
N
N			   for (k=0;k<(FILTER_LENGTH-1);k++)
X			   for (k=0;k<(201-1);k++)
N				{
N					in_buf[k]=in_buf[k+1];
N				}
N
N				 in_buf[FILTER_LENGTH-1]=ecgdata;
X				 in_buf[201-1]=ecgdata;
N				// a=0;
N
N				for( k=0;k<FILTER_LENGTH;k++)
X				for( k=0;k<201;k++)
N				{
N				LPF_2_hz_out+=((Int32)(Int16)in_buf[FILTER_LENGTH-(k+1)]*(Int32)(Int16)Low_Pass_Filter_2Hz_coeffs[k]);
X				LPF_2_hz_out+=((Int32)(Int16)in_buf[201-(k+1)]*(Int32)(Int16)Low_Pass_Filter_2Hz_coeffs[k]);
N				}
N
N
N	}
N	return LPF_2_hz_out;
N}
