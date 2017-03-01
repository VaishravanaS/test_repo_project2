L 1 "../src/Respiration_Module.c"
N /******************************************************************************
N**File Name			: Respiration_Module.c
N**File Description	:
N**Author    		: Vuon1
N**Creation Date		: 05-Aug-2016
N**Last modified date:
N**Remarks			:
N*******************************************************************************/
N
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
L 11 "../src/Respiration_Module.c" 2
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
L 12 "../src/Respiration_Module.c" 2
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
L 13 "../src/Respiration_Module.c" 2
N
N
Nextern Uint8 Respiration_Rate;
Nint Calculate_Respiration(long LPF_2_hz_out1)
N{
N
N	extern Int16 ECGData[8];
N
N	static long zero_crossing_count=0;
N	static Uint8 present_dir,previous_dir;
N
N/*determination of respiration rate by zero crossing detection*/
N	if(LPF_2_hz_out1>0)
N		present_dir=1;
N	else present_dir=0;
N	if(present_dir!=previous_dir)
N	{//send 32 bit code-32770 before sending zero_crossing_count byte by byte
N		zero_crossing_count++;
N		/*EVM5515_UART_putChar(0x02);
N		EVM5515_UART_putChar(0x80);
N		EVM5515_UART_putChar(0x00);
N		EVM5515_UART_putChar(0x00);
N
N		//send 32 bit data-zero_crossing_count byte by byte
N		EVM5515_UART_putChar(zero_crossing_count&0x00ff);
N		EVM5515_UART_putChar((zero_crossing_count&0xff00)>>8);
N		EVM5515_UART_putChar((zero_crossing_count&0xff0000)>>16);
N		EVM5515_UART_putChar((zero_crossing_count&0xff000000)>>24);*/
N
N
N	}
N	previous_dir=present_dir;
N
N/*determination of peak in respiration wave using a fixed threshold */
N	static long peak=0;
N	static Uint8 sent_flag=0;
N	static Uint8 threshold_crossed_flag;
N	static unsigned int count=0,peak_count=0,samples_count=0;
N	static float resp_rate;
N
N
N	samples_count++;
N	if(samples_count==30000)/*1 minute count 500 samples per second*/
N	{
N		samples_count=0;
N//		resp_rate=(float)(peak_count*SAMPLING_RATE)/30000;
N
N		/*update global variable for respiration rate*/
N
N//		if((resp_rate-((unsigned char)resp_rate))>0.5)
N//			Respiration_Rate=((unsigned char)resp_rate)+1;
N//		else
N//			Respiration_Rate=(unsigned char)resp_rate;
N
N		/*update global buffer with the number of peaks every minute*/
N		Respiration_Rate=peak_count;
N		peak_count=0;
N
N	}
N	if(LPF_2_hz_out1>5)
N	{//check for peak
N		threshold_crossed_flag=1;
N	}
N	else
N	{
N		threshold_crossed_flag=0;
N		peak=0;
N		sent_flag=0;
N	}
N	if(threshold_crossed_flag==1)
N	{
N
N		if(peak<LPF_2_hz_out1)
N			{
N				peak=LPF_2_hz_out1;
N				count=0;
N			}
N		count++;
N		if(count==10)
N		{
N			//send 32 bit code-32772 before sending peak byte by byte
N			if(sent_flag==0)
N			{
N		/*	EVM5515_UART_putChar(0x04);
N			EVM5515_UART_putChar(0x80);
N			EVM5515_UART_putChar(0x00);
N			EVM5515_UART_putChar(0x00);
N
N			//send 32 bit data-peak byte by byte
N			EVM5515_UART_putChar(peak&0x00ff);
N			EVM5515_UART_putChar((peak&0xff00)>>8);
N			EVM5515_UART_putChar((peak&0xff0000)>>16);
N			EVM5515_UART_putChar((peak&0xff000000)>>24);
N		*/	peak_count++;
N			count=0;
N			sent_flag=1;
N
N			}
N		}
N		threshold_crossed_flag=0;
N	}
N
N
N
N
N/*determination of peak in respiration curve*/
N	{
N		static long present_adc_value[BUFFER]={0,0,0,0,0,0,0,0,0,0};
X		static long present_adc_value[10]={0,0,0,0,0,0,0,0,0,0};
N		Uint8 k;
N
N		static long pos_avg_peak,neg_avg_peak;
N		static long pos_peak,neg_peak,temp_peak;
N
N		static Int16 present_direction_flag,previous_direction_flag,temp_pos_direction_count,temp_neg_direction_count;
N	//	static long red_adc_buffer[ACCURACY];
N		present_adc_value[0]=LPF_2_hz_out1;
N
N
N
N
N
N		if(temp_peak<present_adc_value[0])
N		{
N			if(temp_pos_direction_count==0)
N			{	neg_peak=present_adc_value[1];
N				for (k=2;k<BUFFER;k++)
X				for (k=2;k<10;k++)
N				{
N					if(present_adc_value[k]<neg_peak)
N					{
N						neg_peak=present_adc_value[k];
N					}
N				}
N			}
N			temp_pos_direction_count++;
N			temp_neg_direction_count=0;
N
N			if(temp_pos_direction_count==ACCURACY)
X			if(temp_pos_direction_count==10)
N			{
N				present_direction_flag=1;
N				temp_peak=present_adc_value[1];
N				temp_pos_direction_count=0;
N			}
N		}
N		else if(temp_peak>present_adc_value[0])
N		{
N			if(temp_neg_direction_count==0)
N			{
N				pos_peak=present_adc_value[1];
N
N
N				for (k=2;k<BUFFER;k++)
X				for (k=2;k<10;k++)
N				{
N					if(present_adc_value[k]>pos_peak)
N					{
N						pos_peak=present_adc_value[k];
N					}
N				}
N
N			}
N			temp_neg_direction_count++;
N
N			temp_pos_direction_count=0;
N			if(temp_neg_direction_count==ACCURACY)
X			if(temp_neg_direction_count==10)
N			{
N				present_direction_flag=0;
N				temp_peak=present_adc_value[1];
N				temp_neg_direction_count=0;
N			}
N		}
N
N		if(present_direction_flag!=previous_direction_flag)
N		{
N			if(present_direction_flag==1)
N			{
N				neg_avg_peak=neg_peak;
N				#ifdef PRINT_PEAKS
S				printf("neg_avg_peak=%lu\r\n",neg_avg_peak);
N				#endif
N			/*	//send 32 bit code-32767 before sending neg_avg_peak byte by byte
N				EVM5515_UART_putChar(0xFF);
N				EVM5515_UART_putChar(0x7F);
N				EVM5515_UART_putChar(0x00);
N				EVM5515_UART_putChar(0x00);
N
N				//send 32 bit data-neg_avg_peak byte by byte
N				EVM5515_UART_putChar(neg_avg_peak&0x00ff);
N				EVM5515_UART_putChar((neg_avg_peak&0xff00)>>8);
N				EVM5515_UART_putChar((neg_avg_peak&0xff0000)>>16);
N				EVM5515_UART_putChar((neg_avg_peak&0xff000000)>>24);*/
N
N			}
N			else
N			{
N				pos_avg_peak=pos_peak;
N				#ifdef PRINT_PEAKS
S				printf("pos_avg_peak=%lu\r\n",pos_avg_peak);				//for debug only
N				#endif
N			/*	//send 32 bit code-32769 before sending pos_avg_peak byte by byte
N				EVM5515_UART_putChar(0x01);
N				EVM5515_UART_putChar(0x80);
N				EVM5515_UART_putChar(0x00);
N				EVM5515_UART_putChar(0x00);
N
N				//send 32 bit data-pos_avg_peak byte by byte
N				EVM5515_UART_putChar(pos_avg_peak&0x00ff);
N				EVM5515_UART_putChar((pos_avg_peak&0xff00)>>8);
N				EVM5515_UART_putChar((pos_avg_peak&0xff0000)>>16);
N				EVM5515_UART_putChar((pos_avg_peak&0xff000000)>>24);*/
N
N			}
N
N		}
N
N
N
N		for(k=(BUFFER-1);k!=0;k--)
X		for(k=(10-1);k!=0;k--)
N		{
N			present_adc_value[k]=present_adc_value[k-1];
N
N
N		}
N
N		previous_direction_flag=present_direction_flag;
N
W "../src/Respiration_Module.c" 123 15 variable "pos_avg_peak" was set but never used
W "../src/Respiration_Module.c" 123 28 variable "neg_avg_peak" was set but never used
N	}
N
N	return 0;
W "../src/Respiration_Module.c" 51 15 variable "resp_rate" was declared but never referenced
N}
