L 1 "../src/Arrythmia.c"
N /******************************************************************************
N**File Name			: Arrythmia.c
N**File Description	:
N**Author    		: Vuon1
N**Creation Date		: 03-Aug-2016
N**Last modified date:
N**Remarks			:
N*******************************************************************************/
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
L 15 "../inc/Arrythmia.h" 2
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
L 16 "../inc/Arrythmia.h" 2
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
L 10 "../src/Arrythmia.c" 2
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
L 11 "../src/Arrythmia.c" 2
N
Nextern Uint8 SystemState_DetectingArrythmiaInBufferStatus;
N
Nextern int ECG_arrythmia_buf2[];
Nextern int ECG_arrythmia_buf1[];
N
N
N/*enum of states here*/
Nshort int i,j0,k,M=2500,r=0;
Nint sum=0;
Nfloat mean;
Nfloat x0[2000]={0};
Nfloat x1[2000]={0};
Nfloat x2[3000]={0};
Nfloat x3[3000]={0};
Nfloat x4[3000]={0};
Nfloat x5[2000]={0};
Nfloat x6[2000]={0};
Nfloat x7[2000]={0};
Nfloat x8[2000]={0};
Nfloat x9[2000]={0};
Nfloat x10[2000]={0};
Nfloat xm[2000]={0};
Nfloat x11[2000]={0};
Nfloat x12[2000]={0};
Nfloat x13[2000]={0};
Nint remainig[10]={0};
Nfloat *p;
Nint *D;
Nint N1=13;
Nint N2=33;
Nint N3=31;
Nint peak;
Nshort int itterations=0;
Nshort  int h[2000]={1,2,3,4,5,6,5,4,3,2,1,0,0};
Nint h1[2000]={-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,31,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,0};
Nfloat h2[2000]={0.0323,0.0323,0.0323,0.0323,0.0323,0.0323,0.0323,0.0323,0.0323,0.0323,
N	            0.0323,0.0323,0.0323,0.0323,0.0323,0.0323,0.0323,0.0323,0.0323,0.0323,
N	            0.0323,0.0323,0.0323,0.0323,0.0323,0.0323,0.0323,0.0323,0.0323,0.0323,0.0323};
N//----------------------------2 R peaks------------------
Nfloat small,small_1;
Nshort int r_count=0;
Nshort int start1,stop1,start2,stop2;
Nfloat r_peak1,r_peak2;
Nshort int r_peak1_loc,r_peak2_loc;
Nshort int flag,flag1,flag2,flag3,flag4,flag5,flag6,flag7,ptflag=0,flag_r;
Nfloat big,big1;
Nshort int start_point;
N//------------------For PVC------------------------------
Nfloat  big_first,peak1_pvc;
Nshort int  peak1_pvc_loc,c1,c2,dip_min1,dip_min2;
Nfloat X0,X1,X2;
N//-----------------Flutter-------------------------------
Nshort int flutter_flag=0,f,f1;
Nint F1[10]={0};
Nint F2[10]={0};
Nfloat offset0,offset1;
Nshort int count_arryth,count_arryth2;
N//-----------------Flutter-------------------------------
Nfloat threshold;
N//------------------Normal-------------------------------
N//-----------------s wave location and amplitude---------
Nshort int s_peak1_loc,s_peak2_loc;
Nfloat s_peak1,s_peak2;
N//-----------------first J point-------------------------
Nshort int j_point,Fs,q_peak2_loc,first_point,c1,c2,t_point,p_point,int_point,t_point_start,t_point_end,min_pt_loc;
Nshort int p_start,r_point,j_point2;
Nfloat q_peak2,first_wave_amplitude,t_wave_peak,p_wave_peak,qrs_duration,rr_interval,normal,pr_duration;
N//----------------Macros---------------------------------
N#define b_f 0.05 // Threshod value above which atrial flutter would be present
N
N
N
N
N
N
N
N
N//To find the absolute value
Nfloat* absolute(float* x,float m)
N{
N	int j0;
N	for(j0=0;j0<=m;j0++)
N	{
N		if(x[j0]<0)
N		{
N			x[j0]=(-1)*x[j0];
N		}
N	}
N	return x;
N}
N//To normalize
Nfloat* normalize(float* x,float *y,float m)
N{
N	int j0;
N	float big=x[0];
N	for(j0=0;j0<m;j0++)
N	{
N		if(big<x[j0])
N		{
N			big=x[j0];
N		}
N	}
N
N	for(j0=0;j0<m;j0++)
N	{
N		y[j0]=y[j0]/big;
N	}
N	return y;
N}
N//To find the largest of the array
Nfloat find_big(short int s1,short int s2,float* x)
N{
N
N	big1=x[s1];
N	for(i=s1;i<=s2;i++)
N	{
N		if(big1<x[i])
N		{
N			big1=x[i];
N		}
N
N	}
N	return big1;
N
N}
N
N//To find the smallest of the array
Nfloat find_small(int s1,int s2,float* x)
N{
N	small_1=x[s1];
N	for(i=s1;i<=s2;i++)
N	{
N		if(small_1>x[i])
N		{
N			small_1=x[i];
N		}
N	}
N	return small_1;
N}
N//To search the location
Nint search(short int s1,float *x,float search_value)
N{
N	while(x[s1]!=search_value)
N	{
N		s1++;
N	}
N	return s1;
N}
N
Nint up_slope_right(short int s,float* x,short int fs)
N{
N	float X0,X1;
N	float s_slope,s1_slope;
N	X0=x[s];
N	s++;
N	X1=x[s];
N	s_slope=(X1-X0)*fs;
N	s1_slope=s_slope;
N	while(s1_slope==s_slope)
N	{
N		X0=X1;
N		s++;
N		X1=x[s];
N		s1_slope=(X1-X0)*fs;
N	}
N	return s;
N}
N
Nshort int up_slope_left(short int s,float* x,short int fs)
N{
N
N	float X0,X1;
N	float s_slope,s1_slope;
N	X0=x[s];
N	s--;
N	X1=x[s];
N	s_slope=(X1-X0)*fs;
N	s1_slope=s_slope;
N	while(s1_slope==s_slope)
N	{
N		X0=X1;
N		s--;
N		X1=x[s];
N		s1_slope=(X1-X0)*fs;
N	}
N	return s;
N
N}
N
Nshort int up_right(short int s,float* x)
N{
N	float X0,X1;
N
N	X0=x[s];
N	s++;
N	X1=x[s];
N	while(X1>X0)
N	{
N		X0=X1;
N		s++;
N		X1=x[s];
N	}
N	return s;
N}
N
Nint down_slope_left(short int s,float* x,short int fs)
N{
N	float X0,X1;
N	float s_slope,s1_slope;
N	X0=x[s];
N	s--;
N	X1=x[s];
N	s_slope=(X1-X0)*fs;
N	s1_slope=s_slope;
N	while(s1_slope==s_slope)
N	{
N		X0=X1;
N		s--;
N		X1=x[s];
N		s1_slope=(X1-X0)*fs;
N	}
N	return s;
N}
N
Nshort int down_left(short int s,float* x)
N{
N	float X0,X1;
N
N	X0=x[s];
N	s--;
N	X1=x[s];
N	while(X1<X0)
N	{
N		X0=X1;
N		s--;
N		X1=x[s];
N	}
N	return s;
N}
N
Nshort int down_right(short int s,float* x)
N{
N	float X0,X1;
N
N	X0=x[s];
N	s++;
N	X1=x[s];
N	while(X1<X0)
N	{
N		X0=X1;
N		s++;
N		X1=x[s];
N	}
N	return s;
N}
N
Nshort int up_left(short int s,float* x)
N{
N	float X0,X1;
N
N	X0=x[s];
N	s--;
N	X1=x[s];
N	while(X1>X0)
N	{
N		X0=X1;
N		s--;
N		X1=x[s];
N	}
N	return s;
N}
N
Nint down_slope_right(short int s,float* x,short int fs)
N{
N	float X0,X1;
N	float s_slope,s1_slope;
N	X0=x[s];
N	s++;
N	X1=x[s];
N	s_slope=(X1-X0)*fs;
N	s1_slope=s_slope;
N	while(s1_slope==s_slope)
N	{
N		X0=X1;
N		s++;
N		X1=x[s];
N		s1_slope=(X1-X0)*fs;
N	}
N	return s;
N}
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
L 41 "C:/ti/ccsv6/tools/compiler/c5500_4.4.1/include/stdio.h" 2
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
L 314 "../src/Arrythmia.c" 2
N#include<stdlib.h>
L 1 "C:/ti/ccsv6/tools/compiler/c5500_4.4.1/include/stdlib.h" 1
N/*****************************************************************************/
N/* stdlib.h   v4.4.1                                                         */
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
N#ifndef _STDLIB
N#define _STDLIB
N
N/*---------------------------------------------------------------------------*/
N/* Attributes are only available in relaxed ANSI mode.                       */
N/*---------------------------------------------------------------------------*/
N#ifndef __ATTRIBUTE
S#if __TI_STRICT_ANSI_MODE__
S#define __ATTRIBUTE(attr)
S#else
S#define __ATTRIBUTE(attr) __attribute__(attr)
S#endif
N#endif
N
N
N#ifdef __cplusplus
S//----------------------------------------------------------------------------
S// <cstdlib> IS RECOMMENDED OVER <stdlib.h>.  <stdlib.h> IS PROVIDED FOR 
S// COMPATIBILITY WITH C AND THIS USAGE IS DEPRECATED IN C++
S//----------------------------------------------------------------------------
Sextern "C" namespace std {
N#endif /* !__cplusplus */
N
Ntypedef struct { int quot, rem; } div_t;
N
Ntypedef struct { long quot, rem; } ldiv_t;
N
N#define _LLONG_AVAILABLE 1
Ntypedef struct { long long quot, rem; } lldiv_t;
N
N#define MB_CUR_MAX    1
N
N#ifndef NULL
S#define NULL          0
N#endif
N
N#ifndef _SIZE_T
S#define _SIZE_T
Stypedef __SIZE_T_TYPE__ size_t;
N#endif
N
N#ifndef __cplusplus
N#ifndef _WCHAR_T
N#define _WCHAR_T
Ntypedef __WCHAR_T_TYPE__ wchar_t;
Xtypedef unsigned int wchar_t;
N#endif
N#endif
N
N#define EXIT_FAILURE  1
N#define EXIT_SUCCESS  0
N
N#define RAND_MAX      32767
N
N#include <linkage.h>
N
N/*---------------------------------------------------------------*/
N/* NOTE - Normally, abs, labs, and fabs are expanded inline, so  */
N/*        no formal definition is really required. However, ANSI */
N/*        requires that they exist as separate functions, so     */
N/*        they are supplied in the library.  The prototype is    */
N/*        here mainly for documentation.                         */
N/*---------------------------------------------------------------*/
N    _CODE_ACCESS  int       abs(int _val); 
X      int       abs(int _val); 
N    _CODE_ACCESS  long      labs(long _val);
X      long      labs(long _val);
N#if defined(_LLONG_AVAILABLE)
X#if 1L
N    _CODE_ACCESS  long long llabs(long long _val);
X      long long llabs(long long _val);
N#endif
N    _CODE_ACCESS int       atoi(const char *_st);
X     int       atoi(const char *_st);
N    _CODE_ACCESS long      atol(const char *_st);
X     long      atol(const char *_st);
N#if defined(_LLONG_AVAILABLE)
X#if 1L
N    _CODE_ACCESS long long atoll(const char *_st);
X     long long atoll(const char *_st);
N#endif
N    _CODE_ACCESS int       ltoa(long val, char *buffer);
X     int       ltoa(long val, char *buffer);
N          _IDECL double    atof(const char *_st);
X           double    atof(const char *_st);
N
N    _CODE_ACCESS long      strtol(const char *_st, char **_endptr, int _base);
X     long      strtol(const char *_st, char **_endptr, int _base);
N    _CODE_ACCESS unsigned long strtoul(const char *_st, char **_endptr,
X     unsigned long strtoul(const char *_st, char **_endptr,
N    					  int _base);
N#if defined(_LLONG_AVAILABLE)
X#if 1L
N    _CODE_ACCESS long long strtoll(const char *_st, char **_endptr, int _base);
X     long long strtoll(const char *_st, char **_endptr, int _base);
N    _CODE_ACCESS unsigned long long strtoull(const char *_st, char **_endptr,
X     unsigned long long strtoull(const char *_st, char **_endptr,
N					     int _base);
N#endif
N    _CODE_ACCESS double    strtod(const char *_st, char **_endptr);
X     double    strtod(const char *_st, char **_endptr);
N    _CODE_ACCESS long double strtold(const char *_st, char **_endptr);
X     long double strtold(const char *_st, char **_endptr);
N    
N    _CODE_ACCESS int    rand(void);
X     int    rand(void);
N    _CODE_ACCESS void   srand(unsigned _seed);
X     void   srand(unsigned _seed);
N    
N    _CODE_ACCESS void  *calloc(size_t _num, size_t _size)
X     void  *calloc(size_t _num, size_t _size)
N               __ATTRIBUTE((malloc));
X               ;
N    _CODE_ACCESS void  *malloc(size_t _size)
X     void  *malloc(size_t _size)
N               __ATTRIBUTE((malloc));
X               ;
N    _CODE_ACCESS void  *realloc(void *_ptr, size_t _size)
X     void  *realloc(void *_ptr, size_t _size)
N               __ATTRIBUTE((malloc));
X               ;
N    _CODE_ACCESS void   free(void *_ptr);
X     void   free(void *_ptr);
N    _CODE_ACCESS void  *memalign(size_t _aln, size_t _size)
X     void  *memalign(size_t _aln, size_t _size)
N               __ATTRIBUTE((malloc));
X               ;
N    
N    _CODE_ACCESS void   abort(void); 
X     void   abort(void); 
N    _CODE_ACCESS int    atexit(void (*_func)(void));
X     int    atexit(void (*_func)(void));
N    _CODE_ACCESS void  *bsearch(const void *_key, const void *_base,
X     void  *bsearch(const void *_key, const void *_base,
N    				   size_t _nmemb, size_t _size, 
N    			           int (*compar)(const void *,const void *));
N    _CODE_ACCESS void   qsort(void *_base, size_t _nmemb, size_t _size, 
X     void   qsort(void *_base, size_t _nmemb, size_t _size, 
N    			         int (*_compar)(const void *, const void *));
N    _CODE_ACCESS void   exit(int _status);
X     void   exit(int _status);
N    
N    _CODE_ACCESS div_t  div(int _numer, int _denom);
X     div_t  div(int _numer, int _denom);
N    _CODE_ACCESS ldiv_t ldiv(long _numer, long _denom);
X     ldiv_t ldiv(long _numer, long _denom);
N#if defined(_LLONG_AVAILABLE)
X#if 1L
N    _CODE_ACCESS lldiv_t lldiv(long long _numer, long long _denom);
X     lldiv_t lldiv(long long _numer, long long _denom);
N#endif
N
N    _CODE_ACCESS char  *getenv(const char *_string);
X     char  *getenv(const char *_string);
N    _CODE_ACCESS int    system(const char *_name);
X     int    system(const char *_name);
N
N    _CODE_ACCESS int    mblen(const char *, size_t);
X     int    mblen(const char *, size_t);
N    _CODE_ACCESS size_t mbstowcs(wchar_t *, const char *, size_t);
X     size_t mbstowcs(wchar_t *, const char *, size_t);
N    _CODE_ACCESS int    mbtowc(wchar_t *, const char *, size_t);
X     int    mbtowc(wchar_t *, const char *, size_t);
N
N    _CODE_ACCESS size_t wcstombs(char *, const wchar_t *, size_t);
X     size_t wcstombs(char *, const wchar_t *, size_t);
N    _CODE_ACCESS int    wctomb(char *, wchar_t);
X     int    wctomb(char *, wchar_t);
N
N#ifdef __cplusplus
S} /* extern "C" namespace std */
N#endif /* __cplusplus */
N
N
N#ifdef _INLINE
S
S#ifdef __cplusplus
Snamespace std {
S#endif
S
Sstatic __inline double atof(const char *_st) 
S{
S  return strtod(_st, (char **)0); 
S}
S
S#ifdef __cplusplus
S} /* namespace std */
S#endif
S
N#endif  /* _INLINE */
N
N#endif  /* ! _STDLIB */
N
N#if defined(__cplusplus) && !defined(_CPP_STYLE_HEADER)
X#if 0L && !0L
Susing std::div_t;
Susing std::ldiv_t;
S#if defined(_LLONG_AVAILABLE)
Susing std::lldiv_t;
S#endif
Susing std::size_t;
Susing std::abs;
Susing std::labs;
Susing std::atoi;
Susing std::atol;
S#if defined(_LLONG_AVAILABLE)
Susing std::llabs;
Susing std::atoll;
S#endif
Susing std::atof;
Susing std::strtol;
Susing std::strtoul;
S#if defined(_LLONG_AVAILABLE)
Susing std::strtoll;
Susing std::strtoull;
S#endif
Susing std::strtod;
Susing std::rand;
Susing std::srand;
Susing std::calloc;
Susing std::malloc;
Susing std::realloc;
Susing std::free;
Susing std::memalign;
Susing std::abort;
Susing std::atexit;
Susing std::bsearch;
Susing std::qsort;
Susing std::exit;
Susing std::div;
Susing std::ldiv;
S#if defined(_LLONG_AVAILABLE)
Susing std::lldiv;
S#endif
Susing std::getenv;
Susing std::system;
N#endif /* ! _CPP_STYLE_HEADER */
N
L 315 "../src/Arrythmia.c" 2
N/*int main()*/
Nvoid arrythmia_detection(int *buffer,int buff_idf)
N{
N	*GPIO_DOUT0_ADDR|=0x0004;
X	*((ioport volatile unsigned*)0x1C0A)|=0x0004;
N	i=700;
N	j0=0;
N	while(i!=M)
N	{
N		if(i%2==0)
N		{
N		x0[j0++]=*buffer++;
N		}
N		else
N		{
N			*buffer++;
N		}
N		i++;
N	}
N
N/*	FILE *myFile;*/
N/*	myFile = fopen("sinus.txt","r");*/
N/*	if (myFile == NULL)*/
N/*	{*/
N/*		printf("Cannot open file\n");*/
N/*		exit(0);*/
N/*	}*/
N/*	else*/
N/*	{*/
N/*		printf("file opened\n");	*/
N/*	}*/
N
N/*	printf("Enter the length of file\n");*/
N/*	scanf("%d",&M);	*/
N/*	printf("Enter the sampling Frequency\n");*/
N/*	scanf("%d",&Fs);*/
N
N/*	for(i=0;i<M;i++)*/
N/*	{*/
N/*		fscanf(myFile,"%d",&x1[i]);*/
N		//printf("%d\n",x1[i]);
N	/*}*/
N	/*	for(i=0;i<M;i++)*/
N	/*	{*/
N	/*		printf("%d\n",x1[i]);*/
N	/*	}	*/
N/*	printf("\n");*/
N	//To Find Bigger Value
N	/*	peak=find_big(0,M,x1);*/
N	/*	printf("Peak=%d\n",peak);*/
N	//
N
N
N
N
N
N
N
N
N
N	//To obtain mean------------------------------------------------------------
N//VS removed on trail
N	for(j0=0;j0<M;j0++)
N	{
N		//x1[j0]=ecg_data;
N		sum=sum+x0[j0];
N	}
N	mean=(float)sum/100;
N	//printf("Mean=%f\n",mean);
N	//Cancel DC Drift
N	for(j0=0;j0<M;j0++)
N	{
N		x2[j0]=x0[j0]-mean;
N		// printf("%f\n",x2[j0]);
N	}
N	//printf("\n");
N//*/
N
N
N
N
N
N
N
N/*VS removed on trial
N	//LPF
N	for(i=N1;i<M+N1-1;i++)
N	{
N		h[i]=0;
N	}
N
N	for(i=M;i<M+N1-1;i++)
N	{
N		x2[i]=0;
N	}
N
N	for(i=0;i<M+N1-1;i++)
N	{
N		x3[i]=0;
N		for(j0=0;j0<=i;j0++)
N		{
N			x3[i]=x3[i]+(x2[j0]*h[i-j0]);
N		}
N	}
N
N*/
N
N
N
N
N
N
N
N
N	/*	printf("Length=%d\n",M+N1-1);*/
N	/*	for(i=0;i<M+N1-1;i++)*/
N	/*	{*/
N	/*	    printf("%f\n",x3[i]);*/
N	/*	}*/
N//VS	printf("\n");
N
N
N
N
N
N
N
N
N
N	//HPF
N/*VS removed on trail
N	for(i=N2;i<M+N1+N2-2;i++)
N	{
N		h1[i]=0;
N	}
N	for(i=M+N1-1;i<M+N1+N2-2;i++)
N	{
N		x3[i]=0;
N	}
N
N	for(i=0;i<M+N1+N2-2;i++)
N	{
N		x4[i]=0;
N		for(j0=0;j0<=i;j0++)
N		{
N			x4[i]=x4[i]+(x3[j0]*h1[i-j0]);
N		}
N	}
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
N	/*printf("Length=%d\n",M+N1+N2-2);*/
N	/*	for(i=0;i<M+N1+N2-2;i++)*/
N	/*		{*/
N	/*	    printf("%f\n",x4[i]);*/
N	/*		}*/
N	//VS		printf("\n");
N
N
N
N
N
N	/*VS removed on trial
N	//Cancel Delay
N	for(i=0;i<M;i++)
N	{
N		x4[i]=x4[16+i];
N		//printf("%f\n",x4[i]);
N	}
N	//VS	printf("\n");
N	*/
N
N
N
N
N	/*VS removed on trial
N	//backup of signal
N	for(i=0;i<M;i++)
N	{
N		x5[i]=x4[i];
N	}
N	*/
N
N
N
N	/*VS removed on trial
N	//Absolute Value and Normalize
N	p=absolute(x4,M);
N	for(i=0;i<M;i++)
N	{
N		x6[i]=*p++;
N		//printf("%f\n",x4[i]);
N	}
N*/
N
N
N	/*VS removed on trial
N	p=normalize(x6,x5,M);
N	for(i=0;i<M;i++)
N	{
N		x7[i]=*p++;
N		//x7[i]=peak*(*p++);
N		//printf("%f\n",x7[i]);
N	}
N*/
N
N
N
N
N	//Squaring
N	for(i=0;i<M;i++)
N	{	x9[i]=x0[i]*x0[i];
N		//VS original x8[i]=x7[i]*x7[i];
N		/*printf("%f\n",x8[i]);*/
N	}
N
N
N	/*VS removed on trial
N	//normalize
N	p=normalize(x8,x8,M);
N	for(i=0;i<M;i++)
N	{
N		x9[i]=*p++;
N		//printf("%f\n",x9[i]);
N	}
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
N	//Moving Window Integration
N	/*printf("Length of integration=%d\n",M+N3-1);*/
N/*VS removed on trial
N	 for(i=N3;i<M+N3-1;i++)
N	{
N		h2[i]=0;
N	}
N
N	for(i=M;i<M+N3-1;i++)
N	{
N		x9[i]=0;
N	}
N
N	for(i=0;i<M+N3-1;i++)
N	{
N		x10[i]=0;
N		for(j0=0;j0<=i;j0++)
N		{
N			x10[i]=x10[i]+(x9[j0]*h2[i-j0]);
N		}
N	}
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
N
N
N
N	/*	for(i=0;i<M+N3-1;i++)*/
N	/*		{*/
N	/*		    printf("%f\n",x10[i]);*/
N	/*		}*/
N
N
N
N
N	//Cancel Delay
N	for(i=0;i<M;i++)
N	{
N		x10[i]=x10[6+i];
N		/*printf("%f\n",x10[i]);*/
N	}
N
N
N
N	//Normalize to one
N	p=normalize(x10,x10,M);
N	for(i=0;i<M;i++)
N	{
N		x10[i]=*p++;
N		/*printf("%f\n",x10[i]);*/
N	}
N
N
N
N
N	//Signal Pre-Processing
N	for(i=0;i<M;i++)
N	{
N		if(x10[i]<0.1)
N		{
N			x11[i]=1;
N		}
N		else if(x10[i]>0.6)
N		{
N			x11[i]=0;
N		}
N
N	}
N
N
N
N
N
N	/*TBD and removed*/
N    /*debug for preprocessed signal here*/
N	/*	for(i=0;i<M;i++)*/
N	/*	{*/
N	/*		printf("%d\n",x11[i]);*/
N	/*	}*/
N	for(i=0;i<M;i++)
N	{
N		if(x11[i]>0)
N		{
N			x11[i]=0;
N		}
N		else if(x11[i]==0)
N		{
N			x11[i]=1;
N		}
N	}
N	/*	for(i=0;i<M;i++)*/
N	/*	{*/
N	/*		printf("%d\n",x11[i]);*/
N	/*	}*/
N
N	for(i=0;i<M;i++)
N	{
N		x12[i]=x11[i]*x7[i];
N		/*printf("%f\n",x12[i]);*/
N	}
N
N
N
N
N
N
N
N	//Normalise
N	p=normalize(x12,x12,M);
N	for(i=0;i<M;i++)
N	{
N		x13[i]=*p++;
N		/*printf("%f\n",x13[i]);*/
N	}
N	j0=0;
N	/*	while(j0!=M)*/
N	/*	{*/
N	/*		printf("x13[%d]=%f\n",j0,x13[j0]);*/
N	/*		j0++;*/
N	/*	}*/
N
N
N
N
N/*vai ANALYSIS STARTS HERE*/
N
N
N	//Number of QRS Peaks at a time---------------------------------------------
N	j0=0;
N	r_count=0;
N	flag=0;
N	while(j0!=M)
N	{
N
N		while(x13[j0]==0&&j0!=M)
N		{
N			j0++;
N			//printf("x13[%d]=%f\n",j0,x13[j0]);
N		}
N
N		while(x13[j0]!=0&&j0!=M)
N		{
N			big=x13[j0];
N			if(big<x13[j0])
N			{
N				big=x13[j0];
N			}
N			if(flag==0&&big>0.6)
N			{
N				remainig[r++]=j0;
N				++r_count;
N				flag=1;
N			}
N			j0++;
N
N		}
N		flag=0;
N		big=0;
N
N	}
N
N
N
N
N
N
N
N	//VS printf("There are %d QRS Complexes in the signal\n",r_count);
N	//-------------Copy the remaining data to buffer----------------------------
N	j0=remainig[r];
N	k=700;
N	switch(buff_idf)
N	{
N	case 0:for(i=j0;i<=M;i++)
N			{
N		ECG_arrythmia_buf1[--k]=x0[i];
N			}
N			break;
N	case 1:for(i=j0;i<=M;i++)
N			{
N		ECG_arrythmia_buf2[--k]=x1[i];
N			}
N			break;
N	}
N
N
N
N
N
N
N
N
N	//ECG ANALYSIS BEGINS HERE--------------------------------------------------
N	start_point=0;
N	itterations=0;
Nlabel:	while(itterations!=(r_count-1))
N	{
N		itterations=itterations+1;
N		//printf("Result of %d RR peak are---------------------------\n\n\n",itterations);
N		//----------------------------R peak1 loc and amplitude-----------------
N		big=0;
N		i=start_point;
N		while(big<0.8)
N		{
N			while(x13[i]==0&&i!=M)
N			{
N				i++;
N			}
N			start1=i;
N
N			while(x13[i]!=0&&i!=M)
N			{
N				i++;
N			}
N			stop1=i;
N
N			big=find_big(start1,stop1,x13);
N
N
N			if(big>0.8)
N			{
N				r_peak1=big;
N				r_peak1_loc=search(start1,x13,r_peak1);
N			}
N		}
N
N
N
N
N
N		//printf("R peak1 loc=%d and amplitude=%f\n",r_peak1_loc,r_peak1);
N		//----------------------------R peak2 loc and amplitude-----------------
N		i=stop1;
N		big=0;
N		while(big<0.8)
N		{
N			while(x13[i]==0)
N			{
N				i++;
N			}
N			start2=i;
N			while(x13[i]!=0)
N			{
N				i++;
N			}
N			stop2=i;
N			big=find_big(start2,stop2,x13);
N			if(big>0.8)
N			{
N				r_peak2=big;
N				r_peak2_loc=search(start2,x13,r_peak2);
N			}
N		}
N		//printf("R peak2 loc=%d and amplitude=%f\n",r_peak2_loc,r_peak2);
N
N
N
N
N
N
N		/*VS
N		//-----------------------Detecting PVC--------------------------------------
N		flag1=0;flag2=0;flag3=0;flag4=0;flag5=0;flag6=0;flag7=0;
N		i=stop1+10;
N		big_first=find_big(stop1,start2,x7);
N		peak1_pvc=big_first;
N
N		if(peak1_pvc>0.3)
N		{
N			peak1_pvc_loc=search(stop1,x7,peak1_pvc);
N			c1=stop1-peak1_pvc_loc;
N			c2=peak1_pvc_loc-stop2;
N			if(c2<c1)
N			{
N				i=peak1_pvc_loc;
N				X1=x7[i];
N				i--;
N				X2=x7[i];
N				while(X2<X1)
N				{
N					X1=X2;
N					i=i-1;
N					X2=x7[i];
N				}
N				dip_min1=i;
N
N				if(x7[dip_min1]<0)
N				{
N					small=find_small(stop1,start2,x7);
N
N					dip_min2=search(stop1,x7,small);
N					i=dip_min2;
N					X0=x7[i];
N					i++;
N					X1=x7[i];
N					while(X1>X0)
N					{
N						X0=X1;
N						i++;
N						X1=x7[i];
N					}
N					if(x7[i]>0&&x7[dip_min2]<-0.3)
N					{
N						printf("PVC Detected\n");
N						flag1=1;
N					}
N				}
N			}
N			else if(c1<c2)
N			{
N				i=peak1_pvc_loc;
N				X0=x7[peak1_pvc_loc];
N				i++;
N				X1=x7[i];
N				while(X1<X0)
N				{
N					X0=X1;
N					i++;
N					X1=x7[i];
N				}
N				if(x7[i]<0)
N				{
N					small=find_small(stop1,start2,x7);
N					dip_min2=search(stop1,x7,small);
N					i=dip_min2;
N					X0=x7[i];
N					i++;
N					X1=x7[i];
N					while(X1>X0)
N					{
N						X0=X1;
N						i++;
N						X1=x7[i];
N					}
N					if(x7[i]>0&&x7[dip_min2]<-0.3)
N					{
N						printf("PVC Detected\n");
N						flag1=1;
N					}
N				}
N			}
N		}
N		//-------------------Detecting Flutter----------------------------------
N		if(flag1==0)
N		{
N			flutter_flag=0;
N			if(big_first>b_f)
N			{
N
N				offset0=0.2*big_first;
N				offset1=0.6*big_first;
N				count_arryth=0;
N				i=stop1;
N				X1=x7[i];
N				i++;
N				X2=x7[i];
N				f=0;
N
N				while(i!=start2)
N				{
N					if(X1<offset0&&X2>offset0)
N					{
N						count_arryth++;
N						F1[f]=i;
N						f++;
N					}
N					X1=X2;
N					i++;
N					X2=x7[i];
N				}
N				count_arryth2=0;
N				i=stop1;
N				X1=x7[i];
N				i++;
N				X2=x7[i];
N				f1=0;
N				//F2[f1]={0};
N				while(i!=start2)
N				{
N					if(X1>offset0&&X2<offset0)
N					{
N						count_arryth2++;
N						F2[f1]=i;
N						f1=f1+1;
N					}
N					X1=X2;
N					i++;
N					X2=x7[i];
N				}
N				if(count_arryth==2&&count_arryth2==2)
N				{
N					printf("Not suffering from flutter\n");
N				}
N				else if(count_arryth>1&&count_arryth2>1)
N				{
N					if(count_arryth==count_arryth2)
N					{
N						for(k=0;k<count_arryth;k++)
N						{
N							big=find_big(F1[k],F2[k],x7);
N							if(big<offset1||big>big_first)
N							{
N								flutter_flag=1;
N							}
N						}
N					}
N					else if(count_arryth<count_arryth2)
N					{
N						for(k=0;k<count_arryth;k++)
N						{
N							big=find_big(F1[k],F2[k+1],x7);
N							if(big<offset1||big>big_first)
N							{
N								flutter_flag=1;
N							}
N						}
N					}
N					if(flutter_flag==0)
N					{
N						printf("suffering from flutter\n");
N						flag2=1;
N					}
N					else
N					{
N						printf("suffering from flutter\n");
N						flag2=1;
N					}
N
N				}
N			}
N		}
N		//---------------------Detecting Fibbrilation-------------------------------
N		if(flag1==0&&flag2==0)
N		{
N			if(big_first<0.05)
N			{
N				threshold=0.5*big_first;
N				count_arryth=0;
N				i=stop1;
N				X1=x7[i];
N				i++;
N				X2=x7[i];
N				while(i!=start2)
N				{
N					if(X1<threshold&&X2>threshold)
N					{
N						count_arryth++;
N					}
N					X1=X2;
N					i++;
N					X2=x7[i];
N				}
N
N				count_arryth2=0;
N				i=stop1;
N				X1=x7[i];
N				i++;
N				X2=x7[i];
N				while(i!=start2)
N				{
N					if(X1<-threshold&&X2>-threshold)
N					{
N						count_arryth2++;
N					}
N					X1=X2;
N					i++;
N					X2=x7[i];
N				}
N				if(count_arryth>2&&count_arryth2>2)
N				{
N					printf("Atrial Fibbrilation\n");
N					flag3=1;
N				}
N
N			}
N		}
N		VS*/
N
N
N
N
N
N		//...................ECG SHAPE NORMAL---------------------------------------
N		//--------To obtain first s wave location and amplitude---------------------
N		//VS if(flag1==0&&flag2==0&&flag3==0)
N		//VS {
N/*VS*/
N
N
N		p=normalize(x0,x0,M);
N		for(i=0;i<M;i++)
N		{
N			x1[i]=*p++;
N			/*x7[i]=peak*(*p++);*/
N			/*printf("%f\n",x7[i]);*/
N		}
N
N
N
N			s_peak1=find_small(start1,stop1,x1);
N			s_peak1_loc=search(start1,x7,s_peak1);
N
N
N
N
N
N
N			//-------1st J point--------------------------------------------------------
N			j_point=up_right(s_peak1_loc,x1);
N			/*printf("j_point_amp=%f and j_point_loc=%d\n",x7[j_point],j_point);*/
N			//-------T and P wave amplitude and durations-------------------------------
N			i=j_point;
N			q_peak2=find_small(j_point,r_peak2_loc,x1);
N			q_peak2_loc=search(j_point,x1,q_peak2);
N			first_wave_amplitude=find_big(j_point,q_peak2_loc,x1);
N			first_point=search(j_point,x1,first_wave_amplitude);
N			c1=first_point-j_point;
N			c2=q_peak2_loc-first_point;
N
N			if(c2>c1)
N			{
N				t_point=first_point;
N				t_wave_peak=first_wave_amplitude;
N				//VS printf("T wave peak=%f\n",t_wave_peak);
N				/*VS */
N				if((t_wave_peak>0.4)||(t_wave_peak<0.2))
N				{
N					/////////////arrythmia flag raised here
N					start_point=start2;
N					goto label;
N				}
N			}
N			else if(c2<c1)
N			{
N				p_point=first_point;
N				p_wave_peak=first_wave_amplitude;
N				//VS printf("P wave peak=%f\n",p_wave_peak);
N				/////////////arrythmia flag raised here
N					start_point=start2;
N					goto label;
N			}
N			if(c2<c1)
N			{
N				i=p_point;
N				int_point=down_left(p_point,x1);
N				t_wave_peak=find_big(j_point,int_point,x1);
N				t_point=search(j_point,x1,t_wave_peak );
N				//VS printf("T wave peak=%f\n",t_wave_peak);
N				/////////////arrythmia flag raised here
N					start_point=start2;
N					goto label;
W "../src/Arrythmia.c" 1117 5 statement is unreachable
N				ptflag=1;
N			}
N			else if(c2>c1)
N			{
N				i=t_point;
N				int_point=down_right(t_point,x1);
N				p_wave_peak=find_big(int_point,q_peak2_loc,x1);
N				p_point=search(j_point,x1,t_wave_peak );
N				//VS printf("P wave peak=%f\n",p_wave_peak);
N				/////////////arrythmia flag raised here
N					start_point=start2;
N					goto label;
W "../src/Arrythmia.c" 1129 5 statement is unreachable
N				ptflag=1;
N			}
N
N
N
N
N			//----------Minimum between P and T-----------------------------------------
N			small=find_small(t_point,p_point,x1);
N			min_pt_loc=search(t_point,x1,small);
N			//----------To find end of T wave-------------------------------------------
N			t_point_end=up_slope_left(min_pt_loc,x1,Fs);
N			if(t_point_end==t_point||t_point_end==t_point+2||t_point_end==t_point+3)
N			{
N				t_point_end=(t_point+(t_point-t_point_start));
N			}
N
N
N
N
N			//---------To find start of P wave------------------------------------------
N			p_start=down_left(p_point,x1);
N			//-----------R Location-----------------------------------------------------
N			r_point=(down_slope_left(q_peak2_loc,x1,Fs)+6);
N
N
N
N
N
N
N
N			//-------Amplitude and Location of 2nd S peak-------------------------------
N			s_peak2=find_small(start2,stop2,x1);
N			s_peak2_loc=search(start2,x1,s_peak2);
N
N
N
N
N
N
N
N
N			//--------2nd J point-------------------------------------------------------
N			j_point2=(up_right(s_peak2_loc,x1)-6);
N		//vs }
N
N
N
N
N
N			//---------Tall T detection-------------------------------------------------
N
N		/*VS
N		if(flag1==0&&flag2==0&&flag3==0)
N		{
N			if(t_wave_peak>0.20)
N			{
N				printf("Tall T Detected\n");
N				flag4=1;
N			}
N		}
N
N
N
N
N
N
N		VS*/
N		//--------------QRS and RR Duration-----------------------------------------
N		//VS if(flas;
N			//VS printf("QRS Duration=%f\n",qrs_duration);
N			rr_interval=(float)(r_peak2_loc-r_peak1_loc-1)/Fs;
N			normal=0.9;
N			if(rr_interval>0.6&&rr_interval<1.2)
N			{
N				normal=	rr_interval;
N				//VS printf("RR interval is %f and is normal\n",rr_interval);
N				flag_r=1;
N			}
N			if(flag_r==0)
N
N		{
N			flag_r=0;
N			qrs_duration=(float)(j_point2-r_point-1);
N				if(rr_interval>=1.5*normal)
N				{
N					//VS printf("Beat Missing\n");
N					/////////////arrythmia flag raised here
N					start_point=start2;
N					goto label;
W "../src/Arrythmia.c" 1218 6 statement is unreachable
N					flag5=1;
N				}
N				else if(rr_interval>1.2)
N				{
N					//VS printf("Tachycardia\n");
N					/////////////arrythmia flag raised here
N					start_point=start2;
N					goto label;
W "../src/Arrythmia.c" 1226 6 statement is unreachable
N					flag5=1;
N				}
N				else if(rr_interval<0.6)
N				{
N					//VS printf("Bradycardia\n");
N					/////////////arrythmia flag raised here
N					start_point=start2;
N					goto label;
W "../src/Arrythmia.c" 1234 6 statement is unreachable
N					flag5=1;
N				}
N			}
N
N	//	}
N
N
N
N
N
N
N			//------------------------PR Duration-----------------------------------
N		// VS if(flag1==0&&flag2==0&&flag3==0&&flag4==0&&flag5==0)
N		{
N			pr_duration=(float)(r_point-p_start-1)/Fs;
N			if(pr_duration>0.2)
N			{
N				//VS printf("AV Block Detected\n");
N				/////////////arrythmia flag raised here
N					start_point=start2;
N					goto label;
W "../src/Arrythmia.c" 1255 5 statement is unreachable
N				flag6=1;
N			}
N			else if(pr_duration<=0.1)
N			{
N				//VS printf("PNC Detected\n");
N				/////////////arrythmia flag raised here
N					start_point=start2;
N					goto label;
W "../src/Arrythmia.c" 1263 5 statement is unreachable
N				flag6=1;
N			}
N		}
N		//VS if(flag1==0&&flag2==0&&flag3==0&&flag4==0&&flag5==0&&flag6==0)
N		{
N			//VS printf("ECG Normal\n");
N
N		}
N		start_point=start2;
N}
N*GPIO_DOUT0_ADDR&=~0x0004;
X*((ioport volatile unsigned*)0x1C0A)&=~0x0004;
N
N}
N
N
N
