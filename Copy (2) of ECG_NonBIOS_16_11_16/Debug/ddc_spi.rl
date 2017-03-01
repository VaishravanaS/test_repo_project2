L 1 "../src/ddc_spi.c"
N /******************************************************************************
N**File Name			: ddc_spi.c
N**File Description	:
N**Created By		:
N**Creation Date		:
N**Last modified date:
N**Remarks			:
N*******************************************************************************/
N
N#include "ddc_spi.h"
L 1 "../inc/ddc_spi.h" 1
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
L 15 "..\inc\llc_spi.h" 2
N#include <psp_common.h>
L 1 "../inc/psp_common.h" 1
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
N#define NULL 0
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
L 16 "..\inc\llc_spi.h" 2
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
L 15 "../inc/ddc_spi.h" 2
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
L 11 "../src/ddc_spi.c" 2
N
Nextern Uint16 LLC_SPI_WLenGet(void);
N
N/**<  Globals                                                       */
NSPI_Handle  spiInstance;
NSPI_Handle  *pSpiInst;
N
N/**
N *  \brief  spi instance initialization
N *
N *  \return status of init
N */
Nvoid spiInitInstance(void)
N{
N    pSpiInst = &spiInstance;
N
N    /* Transition parameters init */
N    pSpiInst->spiTransParam.frameLen = 0;
N    pSpiInst->spiTransParam.isFrameComplete = FALSE;
X    pSpiInst->spiTransParam.isFrameComplete = 0;
N    pSpiInst->spiTransParam.isWordComplete = FALSE;
X    pSpiInst->spiTransParam.isWordComplete = 0;
N    pSpiInst->spiTransParam.spiCmd = (SPI_Command)0;
N    pSpiInst->spiTransParam.wordCnt = 0;
N    pSpiInst->spiTransParam.wordLen = 0;
N
N    /* controller status init */
N    pSpiInst->spiCtrlrStatus.isAddrError = FALSE;
X    pSpiInst->spiCtrlrStatus.isAddrError = 0;
N    pSpiInst->spiCtrlrStatus.spiBusy = (SPI_TransactionStatus)FALSE;
X    pSpiInst->spiCtrlrStatus.spiBusy = (SPI_TransactionStatus)0;
N
N    /* slave config init */
N    pSpiInst->spiSlaveConfig.slvClkPhase = (SPI_SlaveClkPh)0xFF;
N    pSpiInst->spiSlaveConfig.slvClkPolr = (SPI_SlaveClkPolarity)0xFF;
N    pSpiInst->spiSlaveConfig.csPolr = (SPI_SlaveSelPol)0xFF;
N    pSpiInst->spiSlaveConfig.slvDataDly = (SPI_SlaveDataDly)0xFF;
N
N    /* no of SPI instance open */
N    pSpiInst->noOfOpen = 0;
N    pSpiInst->spiSlaveState = SPI_SLAVE_CREATED;
N}
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
N                  Uint32 spiClkRate,
N                  Uint16 frameLength,
N                  Uint16 wordLength)
N{
N    PSP_Result result = PSP_E_INVAL_PARAM;
X    PSP_Result result = (-5);
N
N    /* spi Instance init */
N    spiInitInstance();
N
N    pSpiInst->spiSlaveState = SPI_SLAVE_DELETED;
N
N    /* set clock for the desired SPI clock rate */
N    result = (PSP_Result) LLC_spiInit(spiClkRate, frameLength, wordLength);
N    if (result != PSP_SOK)
X    if (result != (0))
N    {
N        return (result);
N    }
N
N    return (result);
N}
N
N/**
N *  \brief  DDC SPI Deinitialization API
N *
N *  \param  void
N *
N *  \return status of deInit
N */
NPSP_Result DDC_SPI_DeInit(void)
N{
N    pSpiInst->spiSlaveState = SPI_SLAVE_DELETED;
N    pSpiInst = NULL;
X    pSpiInst = 0;
N    return (PSP_SOK);
X    return ((0));
N}
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
N                     SPI_HwMode         mode,
N                     SPI_SlaveConfig    slvConfig)
N{
N    PSP_Result sts;
N
N    sts = 0;
N
N    if( (pSpiInst->noOfOpen > SPI_OPEN_INSTANCES_MAX) ||
X    if( (pSpiInst->noOfOpen > (1)) ||
N        (mode >= SPI_MODE_UNKNOWN)                          ||
N        (slvConfig.slvDataDly >= SPI_SLV_DATA_DLY_UNKNOWN))
N    {
N        return ((SPI_Handle*)NULL);
X        return ((SPI_Handle*)0);
N    }
N    else
N    {
N        sts = LLC_SPI_SlaveSelect(slvConfig.slaveNo);
N        if(sts == SPI_FAILURE) { return (NULL); }
X        if(sts == (0xFFFF)) { return (0); }
N
N        sts = LLC_SPI_DataDelay(slvConfig.slvDataDly);
N        if(sts == SPI_FAILURE) { return (NULL); }
X        if(sts == (0xFFFF)) { return (0); }
N
N        sts = LLC_SPI_SlaveSelectPolSet(slvConfig.csPolr);
N        if(sts == SPI_FAILURE) { return (NULL); }
X        if(sts == (0xFFFF)) { return (0); }
N        
N        sts = LLC_SPI_ModeSelect(mode);
N        if(PSP_SOK == sts)
X        if((0) == sts)
N        {
N            pSpiInst->noOfOpen += 1;
N            /* update SPI handle */
N            pSpiInst->spiSlaveState = SPI_SLAVE_OPENED;
N            pSpiInst->spiSlaveConfig = slvConfig;
N
N            return (pSpiInst);
N        }
N        else
N        {
N            return ((SPI_Handle*)NULL);
X            return ((SPI_Handle*)0);
N        }
N    }
N}
N
N/**
N *  \brief  DDC SPI instance close
N *
N *  \param  SPI_Handle *spiHandle
N *
N *  \return status of spi instance close
N */
NUint16 DDC_SPI_Close(
N               SPI_Handle *spiHandle)
N{
N    Uint16 result = SPI_FAILURE;
X    Uint16 result = (0xFFFF);
N    if((SPI_SLAVE_OPENED == pSpiInst->spiSlaveState))
N    {
N        if((pSpiInst->noOfOpen) > 0)
N        {
N            pSpiInst->noOfOpen -= 1;
N        }
N        pSpiInst->spiSlaveState = SPI_SLAVE_CLOSED;
N        result = PSP_SOK;
X        result = (0);
N        pSpiInst = NULL;
X        pSpiInst = 0;
N    }
N
N    return (result);
N}
N
N/**
N *  \brief  spi Data transaction
N *
N *  \return status of data transfer
N */
NPSP_Result DDC_SpiDataTransaction(
N                         SPI_Handle *hSPI,
N                         Uint16     *transactionBuffer,
N                         Uint16     count,
N                         Uint16     readOrWrite)
N{
N    PSP_Result  result = PSP_SOK;
X    PSP_Result  result = (0);
N
N    Uint16 getWLen;
N
N    getWLen = 0;
N    if (hSPI == NULL)
X    if (hSPI == 0)
N        return (PSP_E_INVAL_PARAM);
X        return ((-5));
N
N    if (transactionBuffer == NULL)
X    if (transactionBuffer == 0)
N        return (PSP_E_INVAL_PARAM);
X        return ((-5));
N
N    /*
N     * check if the instance is open, if not read/write
N     * operation should not be performed, return error
N     * check for bus busy status. return if bus busy
N     */
N    if(SPI_SLAVE_OPENED != hSPI->spiSlaveState)
N    {
N        return (PSP_E_INVAL_STATE);
X        return ((-4));
N    }
N    getWLen = LLC_SPI_WLenGet();
N    if (getWLen > SPI_MAX_WORD_LEN)
X    if (getWLen > (0x32))
N    {
N        return (PSP_E_INVAL_PARAM);
X        return ((-5));
N    }
N
N    /*
N     * based on the operation requested, and on the wlen
N     * call the appropriate read/write apis
N     */
N    if(SPI_READ == readOrWrite)
N    {
N        /* perform read operation for the count provided */
N        result = spiReadData(transactionBuffer,count,getWLen);
N        if (PSP_SOK != result)
X        if ((0) != result)
N            return (result);
N    }
N    else if(SPI_WRITE == readOrWrite)
N    {
N        /* perform write operation for the count provided */
N        result = spiWriteData(transactionBuffer,count,getWLen);
N        if (PSP_SOK != result)
X        if ((0) != result)
N            return (result);
N    }
N    else
N    {
N        return (PSP_E_INVAL_PARAM);
X        return ((-5));
N    }
N
N    return (result);
N}
N
N/**
N *  \brief  spi Read Data
N *
N *  \param  readBuf
N *  \param  readCnt
N *  \param  wLen
N *
N *  \return status of spi read
N */
NPSP_Result spiReadData(
N                 Uint16 *readBuffer,
N                 Uint16 readCnt,
N                 Uint16 wLen)
N{
N    PSP_Result result = PSP_SOK;
X    PSP_Result result = (0);
N    Uint16 bufIndex,index;
N	Uint32 ReadData;
N
N    if (NULL == readBuffer)
X    if (0 == readBuffer)
N        return (PSP_E_INVAL_PARAM);
X        return ((-5));
N
N    if (0 == readCnt)
N        return (PSP_E_INVAL_PARAM);
X        return ((-5));
N
N
N    if(SPI_WORD_LEN_16 == wLen)
X    if((16) == wLen)
N    {
N        for(bufIndex = 0; bufIndex < readCnt; bufIndex++)
N        {
N             /* set operation */
N            result = LLC_SPI_CmdSet(SPI_READ);
N            if (result != PSP_SOK)
X            if (result != (0))
N                {
N                return (result);
N                }
N
N            readBuffer[bufIndex] = LLC_SPI_WordRead();
N        }
N    }
N    else if(SPI_WORD_LEN_8 == wLen)
X    else if((8) == wLen)
N    {
N        for(bufIndex = 0; bufIndex < readCnt; bufIndex++)
N        {
N            result = LLC_SPI_CmdSet(SPI_READ);
N            if (result != PSP_SOK)
X            if (result != (0))
N                {
N                return (result);
N                }
N
N            readBuffer[bufIndex] = LLC_SPI_ByteRead();
N        }
N    }
N	else if(SPI_WORD_LEN_24 == wLen)
X	else if((24) == wLen)
N    {
N        for(bufIndex = 0, index=0; bufIndex < readCnt; bufIndex++)
N        {
N             /* set operation */
N            result = LLC_SPI_CmdSet(SPI_READ);
N            if (result != PSP_SOK)
X            if (result != (0))
N                {
N                return (result);
N                }
N
N            //readBuffer[bufIndex] = LLC_SPI_DoubleWordRead();
N			ReadData = LLC_SPI_DoubleWordRead();
N			readBuffer[index++] = (ReadData >> 16 & 0xFFFF);
N			readBuffer[index++] = (ReadData & 0xFFFF);
N        }
N    }
N    else
N    {
N        result = PSP_E_INVAL_PARAM;
X        result = (-5);
N    }
N
N    return (result);
N}
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
N                  Uint16 *writeBuf,
N                  Uint16 writeCnt,
N                  Uint16  wLen)
N{
N    PSP_Result result = PSP_SOK;
X    PSP_Result result = (0);
N    Uint16 bufIndex;
N    Uint16 sts;
N
N    if (NULL == writeBuf)
X    if (0 == writeBuf)
N        return (PSP_E_INVAL_PARAM);
X        return ((-5));
N
N    if (0 == writeCnt)
N        return (PSP_E_INVAL_PARAM);
X        return ((-5));
N
N    /* set write operation */
N    if(SPI_WORD_LEN_16 == wLen)
X    if((16) == wLen)
N    {
N        for(bufIndex = 0; bufIndex < writeCnt; bufIndex++)
N        {
N            LLC_SPI_WordWrite(writeBuf[bufIndex]);
N            result = LLC_SPI_CmdSet(SPI_WRITE);
N
N            if (result != PSP_SOK)
X            if (result != (0))
N            {
N                return (result);
N            }
N
N            do
N            {
N            LLC_SPI_StatusRead(&sts);
N            }while(((sts & 0x02) != 0x01) && ((sts & 0x01) == 0x01));
N        }
N    }
N    else if(SPI_WORD_LEN_8 == wLen || SPI_WORD_LEN_24 == wLen )
X    else if((8) == wLen || (24) == wLen )
N    {
N        for(bufIndex = 0; bufIndex < writeCnt; bufIndex++)
N        {
N            LLC_SPI_ByteWrite(writeBuf[bufIndex]);
N
N            result = LLC_SPI_CmdSet(SPI_WRITE);
N            if (result != PSP_SOK)
X            if (result != (0))
N            {
N                return (result);
N            }
N
N            do
N            {
N            LLC_SPI_StatusRead(&sts);
N            }while(((sts & 0x02) != 0x01) && ((sts & 0x01) == 0x01));
N        }
N    }
N    else
N    {
N        result = PSP_E_INVAL_PARAM;
X        result = (-5);
N    }
N
N    return (result);
N}
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
N                  Uint16 *cmdBuf,
N                  Uint16 cmdCnt,
N                  Uint16 cLen,
N                  Uint16 rdOrWr)
N{
N    PSP_Result result = PSP_SOK;
X    PSP_Result result = (0);
N    Uint16 bufIndex;
N    Uint16 sts;
N
N    if (NULL == cmdBuf)
X    if (0 == cmdBuf)
N        return (PSP_E_INVAL_PARAM);
X        return ((-5));
N
N    if (0 == cmdCnt)
N        return (PSP_E_INVAL_PARAM);
X        return ((-5));
N
N    /* write data into data register and set the command based on the operation */
N    if(SPI_WORD_LEN_16 == cLen)
X    if((16) == cLen)
N    {
N        //for(bufIndex = 0; bufIndex < cmdCnt; bufIndex++)
N        {
N            LLC_SPI_ByteWrite(cmdBuf[0]);
N            result = LLC_SPI_CmdSet((SPI_Command)rdOrWr);
N            if (result != PSP_SOK)
X            if (result != (0))
N            {
N                return (result);
N            }
N            do
N            {
N            LLC_SPI_StatusRead(&sts);
N            }while(((sts & 0x02) != 0x01) && ((sts & 0x01) == 0x01));
N
N            LLC_SPI_WordWrite((cmdBuf[1] << 8) | (cmdBuf[1]  & 0xFF));
N            result = LLC_SPI_CmdSet((SPI_Command)rdOrWr);
N            if (result != PSP_SOK)
X            if (result != (0))
N            {
N                return (result);
N            }
N            do
N            {
N            LLC_SPI_StatusRead(&sts);
N            }while(((sts & 0x02) != 0x01) && ((sts & 0x01) == 0x01));
N        }
N    }
N    else if(SPI_WORD_LEN_8 == cLen)
X    else if((8) == cLen)
N    {
N        for(bufIndex = 0; bufIndex < cmdCnt; bufIndex++)
N        {
N            LLC_SPI_ByteWrite(cmdBuf[bufIndex]);
N            result = LLC_SPI_CmdSet((SPI_Command)rdOrWr);
N            if (result != PSP_SOK)
X            if (result != (0))
N            {
N                return (result);
N            }
N
N            do
N            {
N            LLC_SPI_StatusRead(&sts);
N            }while(((sts & 0x02) != 0x01) && ((sts & 0x01) == 0x01));
N        }
N    }
N	else if(SPI_WORD_LEN_24 == cLen)
X	else if((24) == cLen)
N    {
N        for(bufIndex = 0; bufIndex < cmdCnt; bufIndex++)
N        {
N            LLC_SPI_ByteWrite(cmdBuf[bufIndex]);
N            result = LLC_SPI_CmdSet((SPI_Command)rdOrWr);
N            if (result != PSP_SOK)
X            if (result != (0))
N            {
N                return (result);
N            }
N
N            do
N            {
N            LLC_SPI_StatusRead(&sts);
N            }while(((sts & 0x02) != 0x01) && ((sts & 0x01) == 0x01));
N        }
N    }
N    else
N    {
N        result = PSP_E_INVAL_PARAM;
X        result = (-5);
N    }
N    return (result);
N}
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
NPSP_Result DDC_DeviceConfig(SPI_SlaveSel slNo, SPI_SlaveDataDly DD, SPI_SlaveSelPol csPol)
N{
N    PSP_Result  result = PSP_SOK;
X    PSP_Result  result = (0);
N
N    result = LLC_SPI_SlaveSelect(slNo);
N    if (PSP_SOK != result)
X    if ((0) != result)
N        return (result);
N
N    result = LLC_SPI_DataDelay(DD);
N    if (PSP_SOK != result)
X    if ((0) != result)
N        return (result);
N
N    result = LLC_SPI_SlaveSelectPolSet(csPol);
N    if (PSP_SOK != result)
X    if ((0) != result)
N        return (result);
N
N    result = LLC_SPI_ModeSelect(SPI_MODE_0);
N    if (PSP_SOK != result)
X    if ((0) != result)
N        return (result);
N
N    return (result);
N}
N
N/*EOF*/
N
