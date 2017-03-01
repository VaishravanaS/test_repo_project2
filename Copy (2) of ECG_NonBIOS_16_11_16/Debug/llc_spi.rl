L 1 "../src/llc_spi.c"
N/******************************************************************************
N**File Name			: llc_spi.c
N**File Description	:
N**Created By		:
N**Creation Date		:
N**Last modified date:
N**Remarks			:
N*******************************************************************************/
N
N#include "llc_spi.h"
L 1 "../inc/llc_spi.h" 1
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
L 15 "../inc/llc_spi.h" 2
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
L 16 "../inc/llc_spi.h" 2
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
L 17 "../inc/llc_spi.h" 2
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
L 11 "../src/llc_spi.c" 2
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
L 51 "../common_inc/corazon.h" 2
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
L 12 "../src/llc_spi.c" 2
N
N/*	function give wait cycles*/
Nextern void wait(Uint32);
N
N/**< Globals                                                        */
N//ioport CSL_SpiRegs *spiReg;
NUint8 inter;
N
N/**
N *  \brief  LLC SPI initialization API
N *
N *  \param  Uint16 spiClkRate,
N *  \param  Uint16 frameLength,
N *  \param  Uint16 wordLength
N *
N *  \return status of init
N */
NPSP_Result LLC_spiInit(
N                 Uint32 spiClkRate,
N                 Uint16 frameLen,
N                 Uint16 wordLen)
N{
N    PSP_Result result = PSP_SOK;
X    PSP_Result result = (0);
N
N//    spiReg = (CSL_SpiRegs *)(CSL_SPI_REGS);
N
N    LLC_SPI_Disable();
N
N    LLC_SPI_ClockSet(spiClkRate);
N
N    result = LLC_SPI_WLenSet(wordLen);
N    if (result != PSP_SOK)
X    if (result != (0))
N        return (result);
N
N    result = LLC_SPI_FLenSet(frameLen);
N    if (result != PSP_SOK)
X    if (result != (0))
N        return (result);
N
N    LLC_SPI_WordInterruptEnable();
N
N    LLC_SPI_Enable();
N
N    return (result);
N}
N
N/**
N *  \brief  enable spi clock
N *
N *  \return status of spi enable
N */
Nvoid LLC_SPI_Enable(void)
N{
N    CSL_SPI_REGS->SPICC2 = (Uint16)(SPI_CLK_ENABLE << CSL_SPI_SPICC2_CLKEN_SHIFT);
X    ((CSL_SpiRegsOvly) 0x3000)->SPICC2 = (Uint16)((1) << (0x000Fu));
N}
N
N/**
N *  \brief  disable spi clock
N *
N *  \return status of spi disable
N */
Nvoid LLC_SPI_Disable(void)
N{
N    CSL_SPI_REGS->SPICC2 = (Uint16)(SPI_CLK_DISABLE << CSL_SPI_SPICC2_CLKEN_SHIFT);
X    ((CSL_SpiRegsOvly) 0x3000)->SPICC2 = (Uint16)((0) << (0x000Fu));
N}
N
N/**
N *  \brief  set clock divisor
N *
N *  \return status of set clock
N */
Nvoid LLC_SPI_ClockSet(Uint32 spiClkRate)
N{
N	Uint16 sysClkDiv;
N	extern Uint32 sysClk;
N
N    if(spiClkRate > 7000000 || spiClkRate < 1000000)
N	{
N        sysClkDiv = SPI_PRESCALE;	//99
X        sysClkDiv = (13);	
N	}
N	else
N	{
N		sysClkDiv = spiDivFun(sysClk, spiClkRate);		
N	    sysClkDiv = sysClkDiv - 1;
N	}
N    CSL_SPI_REGS->SPICC1 = sysClkDiv;
X    ((CSL_SpiRegsOvly) 0x3000)->SPICC1 = sysClkDiv;
N}
N
N/**
N *  \brief  select a slave
N *
N *  \param SPI_SlaveSel spiSlaveSelect
N *
N *  \return status of slave select
N */
NPSP_Result LLC_SPI_SlaveSelect(SPI_SlaveSel spiSlaveSelect)
N{
N    if( spiSlaveSelect < SPI_SLAVE_UNKNOWN )
N    {
N        (CSL_SPI_REGS->SPICR2) |= ((Uint16)(spiSlaveSelect << CSL_SPI_SPICR2_CSNUM_SHIFT));
X        (((CSL_SpiRegsOvly) 0x3000)->SPICR2) |= ((Uint16)(spiSlaveSelect << (0x000Cu)));
N        return (PSP_SOK);
X        return ((0));
N    }
N    else
N    {
N        return (PSP_E_INVAL_PARAM);
X        return ((-5));
N    }
N}
N
N/**
N *  \brief  get current slave selected
N *
N *  \return slave number
N */
NSPI_SlaveSel LLC_SPI_SlaveGet(void)
N{
N    SPI_SlaveSel spiSlave;
N    spiSlave = (SPI_SlaveSel)((CSL_SPI_REGS->SPICR2 & CSL_SPI_SPICR2_CSNUM_MASK)
X    spiSlave = (SPI_SlaveSel)((((CSL_SpiRegsOvly) 0x3000)->SPICR2 & (0x3000u))
N                                      >> CSL_SPI_SPICR2_CSNUM_SHIFT);
X                                      >> (0x000Cu));
N    return (spiSlave);
N}
N
N/**
N *  \brief  select spi mode
N *
N *  \param SPI_HwMode spiModeSelect
N *
N *  \return status of mode select
N */
NPSP_Result LLC_SPI_ModeSelect(SPI_HwMode spiModeSelect)
N{
N    SPI_SlaveSel spiSlave;
N    PSP_Result result;
N    Uint16 clk_pol, clk_ph;
N
N    result = PSP_SOK;
X    result = (0);
N    spiSlave = LLC_SPI_SlaveGet();
N
N    if(spiSlave >= SPI_SLAVE_UNKNOWN)
N        return (PSP_E_INVAL_PARAM);
X        return ((-5));
N
N    switch(spiModeSelect)
N    {
N        default:
N        case SPI_MODE_0:
N            clk_ph =0;
N            clk_pol =0;
N        break;
N        case SPI_MODE_1:
N            clk_ph =1;
N            clk_pol =0;
N        break;
N        case SPI_MODE_2:
N            clk_ph =1;
N            clk_pol =1;
N        break;
N        case SPI_MODE_3:
N            clk_ph =0;
N            clk_pol =1;
N        break;
N        
N    }
N    
N
N    switch(spiSlave)
N    {
N        case SPI_EEPROM:
N            /* CLKP0 */
N            CSL_SPI_REGS->SPIDC1 = ((CSL_SPI_REGS->SPIDC1 & (Uint16)(~CSL_SPI_SPIDC1_CKP0_MASK)) |
X            ((CSL_SpiRegsOvly) 0x3000)->SPIDC1 = ((((CSL_SpiRegsOvly) 0x3000)->SPIDC1 & (Uint16)(~(0x0001u))) |
N                                    (Uint16)(clk_pol << CSL_SPI_SPIDC1_CKP0_SHIFT));
X                                    (Uint16)(clk_pol << (0x0000u)));
N
N            /* CLKPH0 */
N            CSL_SPI_REGS->SPIDC1 = ((CSL_SPI_REGS->SPIDC1 & (Uint16)(~CSL_SPI_SPIDC1_CKPH0_MASK))|
X            ((CSL_SpiRegsOvly) 0x3000)->SPIDC1 = ((((CSL_SpiRegsOvly) 0x3000)->SPIDC1 & (Uint16)(~(0x0004u)))|
N                                    (Uint16)(clk_ph << CSL_SPI_SPIDC1_CKPH0_SHIFT));
X                                    (Uint16)(clk_ph << (0x0002u)));
N            result = PSP_SOK;
X            result = (0);
N            break;
N
N        case SPI_LCD:
N            /* CLKP1 */
N            CSL_SPI_REGS->SPIDC1 = ((CSL_SPI_REGS->SPIDC1 & (Uint16)(~CSL_SPI_SPIDC1_CKP1_MASK)) |
X            ((CSL_SpiRegsOvly) 0x3000)->SPIDC1 = ((((CSL_SpiRegsOvly) 0x3000)->SPIDC1 & (Uint16)(~(0x0100u))) |
N                                    (Uint16)(clk_pol << CSL_SPI_SPIDC1_CKP1_SHIFT));
X                                    (Uint16)(clk_pol << (0x0008u)));
N
N            /* CLKPH1 */
N            CSL_SPI_REGS->SPIDC1 = ((CSL_SPI_REGS->SPIDC1 & (Uint16)(~CSL_SPI_SPIDC1_CKPH1_MASK))|
X            ((CSL_SpiRegsOvly) 0x3000)->SPIDC1 = ((((CSL_SpiRegsOvly) 0x3000)->SPIDC1 & (Uint16)(~(0x0400u)))|
N                                    (Uint16)(clk_ph << CSL_SPI_SPIDC1_CKPH1_SHIFT));
X                                    (Uint16)(clk_ph << (0x000Au)));
N            result = PSP_SOK;
X            result = (0);
N            break;
N
N        default:
N            result = PSP_E_INVAL_PARAM;
X            result = (-5);
N            break;
N    }
N    return (result);
N}
N
N/**
N *  \brief  slave data delay select
N *
N *  \param SPI_SlaveDataDly spiDataDelay
N *
N *  \return status of slave data delay set
N */
NPSP_Result LLC_SPI_DataDelay(SPI_SlaveDataDly spiDataDelay)
N{
N    SPI_SlaveSel spiSlave;
N    PSP_Result result;
N
N    result = PSP_SOK;
X    result = (0);
N
N    spiSlave = LLC_SPI_SlaveGet();
N    switch(spiSlave)
N    {
N        case SPI_EEPROM:
N            /* CSP0 */
N            CSL_SPI_REGS->SPIDC1 = ((CSL_SPI_REGS->SPIDC1 & (Uint16)(~CSL_SPI_SPIDC1_DD0_MASK)) |
X            ((CSL_SpiRegsOvly) 0x3000)->SPIDC1 = ((((CSL_SpiRegsOvly) 0x3000)->SPIDC1 & (Uint16)(~(0x0018u))) |
N                                    (Uint16)(spiDataDelay << CSL_SPI_SPIDC1_DD0_SHIFT));
X                                    (Uint16)(spiDataDelay << (0x0003u)));
N            result = PSP_SOK;
X            result = (0);
N            break;
N
N        case SPI_LCD:
N            /* CSP0 */
N            CSL_SPI_REGS->SPIDC1 = ((CSL_SPI_REGS->SPIDC1 & (Uint16)(~CSL_SPI_SPIDC1_DD1_MASK)) |
X            ((CSL_SpiRegsOvly) 0x3000)->SPIDC1 = ((((CSL_SpiRegsOvly) 0x3000)->SPIDC1 & (Uint16)(~(0x1800u))) |
N                                    (Uint16)(spiDataDelay << CSL_SPI_SPIDC1_DD1_SHIFT));
X                                    (Uint16)(spiDataDelay << (0x000Bu)));
N            result = PSP_SOK;
X            result = (0);
N            break;
N
N        default:
N            result = PSP_E_INVAL_PARAM;
X            result = (-5);
N            break;
N    }
N    return (result);
N}
N
N/**
N *  \brief  select a slave
N *
N *  \param Bool loopBackEnable
N *
N *  \return status loopbackmode select
N */
NPSP_Result LLC_SPI_LoopBackMode(Bool loopBackMode)
N{
N    if((SPI_LOOP_BACK_ENABLE == loopBackMode) ||
X    if(((1) == loopBackMode) ||
N       (SPI_LOOP_BACK_DISABLE == loopBackMode))
X       ((2) == loopBackMode))
N    {
N        CSL_SPI_REGS->SPIDC2 |= (Uint16)(loopBackMode << CSL_SPI_SPIDC2_LPBK_SHIFT);
X        ((CSL_SpiRegsOvly) 0x3000)->SPIDC2 |= (Uint16)(loopBackMode << (0x000Fu));
N        return (PSP_SOK);
X        return ((0));
N    }
N    else
N    {
N        return (PSP_E_INVAL_PARAM);
X        return ((-5));
N    }
N}
N
N/**
N *  \brief  get loopback mode
N *
N *  \return loopback mode
N */
NBool LLC_SPI_LoopBackGet(void)
N{
N    Uint16 loopBackStatus;
N    loopBackStatus = (Uint16)((CSL_SPI_REGS->SPIDC2 & CSL_SPI_SPIDC2_LPBK_MASK)
X    loopBackStatus = (Uint16)((((CSL_SpiRegsOvly) 0x3000)->SPIDC2 & (0x8000u))
N                                          >> CSL_SPI_SPIDC2_LPBK_SHIFT);
X                                          >> (0x000Fu));
N    return (loopBackStatus);
N}
N
N/**
N *  \brief  select slave polarity
N *
N *  \param SPI_SlaveSelPol slaveSelPol
N *
N *  \return status of slave polarity set
N */
NPSP_Result LLC_SPI_SlaveSelectPolSet(SPI_SlaveSelPol slaveSelPol)
N{
N    SPI_SlaveSel spiSlave;
N    PSP_Result result;
N
N    result = PSP_SOK;
X    result = (0);
N
N    spiSlave = LLC_SPI_SlaveGet();
N    if(spiSlave >= SPI_SLAVE_UNKNOWN)
N        return (PSP_E_INVAL_PARAM);
X        return ((-5));
N
N    switch(spiSlave)
N    {
N        case SPI_EEPROM:
N            /* CSP0 */
N            CSL_SPI_REGS->SPIDC1 |= (Uint16)(slaveSelPol << CSL_SPI_SPIDC1_CSP0_SHIFT);
X            ((CSL_SpiRegsOvly) 0x3000)->SPIDC1 |= (Uint16)(slaveSelPol << (0x0001u));
N            result = PSP_SOK;
X            result = (0);
N            break;
N
N        case SPI_LCD:
N            /* CSP1 */
N            CSL_SPI_REGS->SPIDC1 |= (Uint16)(slaveSelPol << CSL_SPI_SPIDC1_CSP1_SHIFT);
X            ((CSL_SpiRegsOvly) 0x3000)->SPIDC1 |= (Uint16)(slaveSelPol << (0x0009u));
N            //LLC_SPI_WordInterruptEnable();
N            result = PSP_SOK;
X            result = (0);
N            break;
N
N        default:
N            result = PSP_E_INVAL_PARAM;
X            result = (-5);
N            break;
N    }
N    return (result);
N}
N
N/**
N *  \brief  wordlength set
N *
N *  \param Uint16 wordLen
N *
N *  \return status of word length set
N */
NPSP_Result LLC_SPI_WLenSet(Uint16 wordLen)
N{
N    CSL_SPI_REGS->SPICR2 |= (Uint16)((wordLen - 1) << CSL_SPI_SPICR2_WLEN_SHIFT);
X    ((CSL_SpiRegsOvly) 0x3000)->SPICR2 |= (Uint16)((wordLen - 1) << (0x0003u));
N    return (PSP_SOK);
X    return ((0));
N}
N
N/**
N *  \brief  get wordlength
N *
N *  \return word length
N */
NUint16 LLC_SPI_WLenGet(void)
N{
N    Uint16   wordLen;
N
N    wordLen = (((CSL_SPI_REGS->SPICR2 & CSL_SPI_SPICR2_WLEN_MASK)
X    wordLen = (((((CSL_SpiRegsOvly) 0x3000)->SPICR2 & (0x00F8u))
N                >> CSL_SPI_SPICR2_WLEN_SHIFT) + 1);
X                >> (0x0003u)) + 1);
N    return (wordLen);
N}
N
N/**
N *  \brief  send SPI command
N *
N *  \param SPI_Command spiCmd
N *
N *  \return status of command set
N */
NPSP_Result LLC_SPI_CmdSet(SPI_Command spiCmd)
N{
N    if((SPI_READ == spiCmd) ||
N       (SPI_WRITE == spiCmd))
N    {
N        CSL_SPI_REGS->SPICR2 = (((CSL_SPI_REGS->SPICR2) & (Uint16)(~CSL_SPI_SPICR2_CMD_MASK))
X        ((CSL_SpiRegsOvly) 0x3000)->SPICR2 = (((((CSL_SpiRegsOvly) 0x3000)->SPICR2) & (Uint16)(~(0x0003u)))
N                  | ((Uint16)(spiCmd << CSL_SPI_SPICR2_CMD_SHIFT)));
X                  | ((Uint16)(spiCmd << (0x0000u))));
N
N        return (PSP_SOK);
X        return ((0));
N    }
N    else
N    {
N        return (PSP_E_INVAL_PARAM);
X        return ((-5));
N    }
N}
N
N/**
N *  \brief  set frame length
N *
N *  \param Uint16 frameLen
N *
N *  \return status of frame length set
N */
NPSP_Result LLC_SPI_FLenSet(Uint16 frameLen)
N{
N    if(frameLen <= SPI_MAX_FLEN)
X    if(frameLen <= (4096))
N    {
N        CSL_SPI_REGS->SPICR1 = (Uint16)((CSL_SPI_REGS->SPICR1) & ((Uint16)~CSL_SPI_SPICR1_FLEN_MASK) )|(frameLen - 1);
X        ((CSL_SpiRegsOvly) 0x3000)->SPICR1 = (Uint16)((((CSL_SpiRegsOvly) 0x3000)->SPICR1) & ((Uint16)~(0x0FFFu)) )|(frameLen - 1);
N//		temp =  *(ioport volatile unsigned*)0x3004;
N        return (PSP_SOK);
X        return ((0));
N    }
N    else
N    {
N        return (PSP_E_INVAL_PARAM);
X        return ((-5));
N    }
N}
N
N/**
N *  \brief get frame length
N *
N *  \return frame length
N */
NUint16   LLC_SPI_FLenGet(void)
N{
N    Uint16   frameLen;
N
N    frameLen = (((CSL_SPI_REGS->SPICR1 & CSL_SPI_SPICR1_FLEN_MASK)
X    frameLen = (((((CSL_SpiRegsOvly) 0x3000)->SPICR1 & (0x0FFFu))
N                >> CSL_SPI_SPICR1_FLEN_SHIFT) + 1);
X                >> (0x0000u)) + 1);
N
N    return (frameLen);
N}
N
N/**
N *  \brief  read status register
N *
N *  \param Uint16 *spiStatus
N *
N */
Nvoid LLC_SPI_StatusRead(Uint16 *spiStatus)
N{
N    *spiStatus = (CSL_SPI_REGS->SPISR1);
X    *spiStatus = (((CSL_SpiRegsOvly) 0x3000)->SPISR1);
N//    *spiStatus = *spiStatus;
N}
N
N/**
N *  \brief  read byte
N *
N *  \return Uint16 byteRead
N */
NUint16 LLC_SPI_ByteRead(void)
N{
N    Uint16 spiStatusReg;
N    Uint16 wordComplete;
N    Uint16 spiBusStatus;
N    Uint16 byteRead;
N
N    spiStatusReg = 0;
N    wordComplete = 0;
N    spiBusStatus = 0;
N    byteRead = 0;
N
N    do {
N        LLC_SPI_StatusRead(&spiStatusReg);
N        spiBusStatus = (spiStatusReg & SPI_STATUS_BUSY_MASK);
X        spiBusStatus = (spiStatusReg & (0x00000001));
N        wordComplete = (spiStatusReg & SPI_STATUS_WC_MASK );
X        wordComplete = (spiStatusReg & (0x00000002) );
N    } while((SPI_STATUS_BUSY == spiBusStatus) && (wordComplete != SPI_WORD_COMPLETE));
X    } while(((0x0001) == spiBusStatus) && (wordComplete != (0x0002)));
N
N    byteRead = (CSL_SPI_REGS->SPIDR1 & 0x00FF);
X    byteRead = (((CSL_SpiRegsOvly) 0x3000)->SPIDR1 & 0x00FF);
N
N    return (byteRead);
N}
N
N/**
N *  \brief  read word
N *
N *  \return Uint16 wordRead
N *
N */
NUint16 LLC_SPI_WordRead(void)
N{
N    Uint16 wordRead;
N    Uint16 spiStatusReg;
N    Uint16 wordComplete;
N    Uint16 spiBusStatus;
N
N    spiStatusReg = 0;
N    wordComplete = 0;
N    spiBusStatus = 0;
N    wordRead = 0;
N
N    do {
N        LLC_SPI_StatusRead(&spiStatusReg);
N        spiBusStatus = (spiStatusReg & SPI_STATUS_BUSY_MASK);
X        spiBusStatus = (spiStatusReg & (0x00000001));
N        wordComplete = (spiStatusReg & SPI_STATUS_WC_MASK );
X        wordComplete = (spiStatusReg & (0x00000002) );
N    } while((SPI_STATUS_BUSY == spiBusStatus) && (wordComplete != SPI_WORD_COMPLETE));
X    } while(((0x0001) == spiBusStatus) && (wordComplete != (0x0002)));
N
N    wordRead = CSL_SPI_REGS->SPIDR1;
X    wordRead = ((CSL_SpiRegsOvly) 0x3000)->SPIDR1;
N
N    return (wordRead);
N}
N
N/**
N *  \brief  read double word
N *
N *  \return Uint32 dWordRead
N *
N */
NUint32 LLC_SPI_DoubleWordRead(void)
N{
N
N    Uint16 spiStatusReg;
N    Uint16 wordComplete;
N    Uint16 spiBusStatus;     
N    Uint16 dwordLSW;
N    Uint32 dWordRead; 
N
N    spiStatusReg = 0;
N    wordComplete = 0;
N    spiBusStatus = 0;   
N    
N    
N
N    do {
N        LLC_SPI_StatusRead(&spiStatusReg);
N        spiBusStatus = (spiStatusReg & SPI_STATUS_BUSY_MASK);
X        spiBusStatus = (spiStatusReg & (0x00000001));
N        wordComplete = (spiStatusReg & SPI_STATUS_WC_MASK );
X        wordComplete = (spiStatusReg & (0x00000002) );
N    } while((SPI_STATUS_BUSY == spiBusStatus) && (wordComplete != SPI_WORD_COMPLETE));
X    } while(((0x0001) == spiBusStatus) && (wordComplete != (0x0002)));
N  
N
N    dWordRead = (CSL_SPI_REGS->SPIDR2 & 0xFFFF);
X    dWordRead = (((CSL_SpiRegsOvly) 0x3000)->SPIDR2 & 0xFFFF);
N    dwordLSW= (CSL_SPI_REGS->SPIDR1);
X    dwordLSW= (((CSL_SpiRegsOvly) 0x3000)->SPIDR1);
N
N	dWordRead = (dWordRead & 0x00ff) << 16;
N
N    dWordRead = (dWordRead | dwordLSW);
N
N    return (dWordRead);
N}
N
N/**
N *  \brief  write byte
N *
N *  \param Uint16 byteWrite
N *
N */
Nvoid LLC_SPI_ByteWrite(Uint16 byteWrite)
N{
N//Uint16 temp;
N    CSL_SPI_REGS->SPIDR2 = (Uint16)(byteWrite <<0x08);
X    ((CSL_SpiRegsOvly) 0x3000)->SPIDR2 = (Uint16)(byteWrite <<0x08);
N//	temp = CSL_SPI_REGS->SPIDR2;
N    CSL_SPI_REGS->SPIDR1 = 0x0000;
X    ((CSL_SpiRegsOvly) 0x3000)->SPIDR1 = 0x0000;
N}
N
N/**
N *  \brief  write word
N *
N *  \param Uint16 wordWrite
N *
N */
Nvoid LLC_SPI_WordWrite(Uint16 wordWrite)
N{
N     CSL_SPI_REGS->SPIDR2 = wordWrite;
X     ((CSL_SpiRegsOvly) 0x3000)->SPIDR2 = wordWrite;
N     CSL_SPI_REGS->SPIDR1 = 0x0000;
X     ((CSL_SpiRegsOvly) 0x3000)->SPIDR1 = 0x0000;
N}
N
N/**
N *  \brief  write double word
N *
N *  \param Uint32 dWordWrite
N *
N */
Nvoid LLC_SPI_DoubleWordWrite(Uint32 dWordWrite)
N{
N     CSL_SPI_REGS->SPIDR2 = (Uint16)(dWordWrite & 0xFFFF);
X     ((CSL_SpiRegsOvly) 0x3000)->SPIDR2 = (Uint16)(dWordWrite & 0xFFFF);
N     CSL_SPI_REGS->SPIDR1 = (Uint16)((dWordWrite >> 0x08) & 0xFFFF);
X     ((CSL_SpiRegsOvly) 0x3000)->SPIDR1 = (Uint16)((dWordWrite >> 0x08) & 0xFFFF);
N}
N
N/**
N *  \brief  enable word interrupt
N *
N */
Nvoid LLC_SPI_WordInterruptEnable(void)
N{
N    CSL_SPI_REGS->SPICR1 = ((CSL_SPI_REGS->SPICR1 & CSL_SPI_SPICR1_WIRQ_MASK) |
X    ((CSL_SpiRegsOvly) 0x3000)->SPICR1 = ((((CSL_SpiRegsOvly) 0x3000)->SPICR1 & (0x4000u)) |
N                            (1 << CSL_SPI_SPICR1_WIRQ_SHIFT));
X                            (1 << (0x000Eu)));
N}
N
N/**
N *  \brief  enable frame interrupt
N *
N */
Nvoid LLC_SPI_FrameInterruptEnable(void)
N{
N    /* dummy functions */
N}
N
N/**
N *  \brief  get word interrupt state
N *
N */
Nvoid LLC_SPI_WordIntrRx_Get(void)
N{
N    /* dummy functions */
N}
N
N/**
N *  \brief  get frame interrupt state
N *
N */
Nvoid LLC_SPI_FrameIntrRx_Get(void)
N{
N    /* dummy functions */
N}
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
N          Uint32 Divisor  )
N{
N    Uint16 Quotient;
N
N    Quotient = 0;
N
N    while (Dividend >= Divisor)
N    {
N        Quotient++;
N        Dividend = Dividend - Divisor;
N    }
N
N    return Quotient;
N}
N
N
NPSP_Result LLC_SPI_WordLengthWrite(Uint32* InpBuf, Uint16 wordLength, Uint16 FrameLen)
N{
N  Uint32 mask = 0;
N  Uint16 shift = 0; 
N  Uint32 WriteVal = 0;
N  PSP_Result result = PSP_E_INVAL_PARAM;
X  PSP_Result result = (-5);
N  Uint8 bufIndex = 0;
N  Uint16 sts;
N  Uint16 temp;
N
N
N  	mask = (Uint32) 1 << wordLength;
N  	mask -= 1;
N	shift = 32- wordLength;
N
N	result = (PSP_Result) LLC_SPI_FLenSet(FrameLen);
N	wait(200);
N    if (result != PSP_SOK)
X    if (result != (0))
N        return (result);
N
N
N        for(bufIndex = 0; bufIndex < FrameLen; bufIndex++)
N        {
N			WriteVal = (InpBuf[bufIndex] & mask) << shift;
N			temp = WriteVal>>16;
N    	    CSL_SPI_REGS->SPIDR2 = temp;
X    	    ((CSL_SpiRegsOvly) 0x3000)->SPIDR2 = temp;
N			temp = WriteVal & 0xFFFF;
N		    CSL_SPI_REGS->SPIDR1 = temp;
X		    ((CSL_SpiRegsOvly) 0x3000)->SPIDR1 = temp;
N
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
N
N		return (result);
N
N}
N
N
NPSP_Result LLC_SPI_WordLengthRead(Uint32* InpBuf, Uint16 wordLength, Uint16 FremLen)
N{
N	Uint32 mask = 0;
N
N	PSP_Result result = PSP_E_INVAL_PARAM;
X	PSP_Result result = (-5);
N	Uint8 bufIndex = 0;
N    Uint16 spiStatusReg;
N    Uint16 wordComplete;
N    Uint16 spiBusStatus; 
N    Uint32 ReadVal = 0;   
N
N  	mask = (Uint32) 1 << wordLength;
N  	mask -= 1;	
N
N	result = (PSP_Result) LLC_SPI_FLenSet(FremLen);
N    if (result != PSP_SOK)
X    if (result != (0))
N        return (result);
N
N
N        for(bufIndex = 0; bufIndex < FremLen; bufIndex++)
N        {
N             CSL_SPI_REGS->SPIDR1 = 0x00;
X             ((CSL_SpiRegsOvly) 0x3000)->SPIDR1 = 0x00;
N             CSL_SPI_REGS->SPIDR2 = 0x00;
X             ((CSL_SpiRegsOvly) 0x3000)->SPIDR2 = 0x00;
N             /* set operation */
N            result = LLC_SPI_CmdSet(SPI_READ);
N            if (result != PSP_SOK)
X            if (result != (0))
N            {
N                return (result);
N            }
N		    do {
N		        LLC_SPI_StatusRead(&spiStatusReg);
N		        spiBusStatus = (spiStatusReg & SPI_STATUS_BUSY_MASK);
X		        spiBusStatus = (spiStatusReg & (0x00000001));
N		        wordComplete = (spiStatusReg & SPI_STATUS_WC_MASK );
X		        wordComplete = (spiStatusReg & (0x00000002) );
N		    } while((SPI_STATUS_BUSY == spiBusStatus) && (wordComplete != SPI_WORD_COMPLETE));
X		    } while(((0x0001) == spiBusStatus) && (wordComplete != (0x0002)));
N
N			ReadVal = CSL_SPI_REGS->SPIDR2; 
X			ReadVal = ((CSL_SpiRegsOvly) 0x3000)->SPIDR2; 
N			ReadVal = ReadVal << 16;
N			ReadVal |= CSL_SPI_REGS->SPIDR1  ;
X			ReadVal |= ((CSL_SpiRegsOvly) 0x3000)->SPIDR1  ;
N			ReadVal &= mask;
N			InpBuf[bufIndex] = ReadVal;
N        }
N
N        return (result);
N}
N
NPSP_Result LLC_SPI_CmdSet_WLenSet_SlaveSelect(SPI_Command spiCmd,Uint16 wordLen,SPI_SlaveSel spiSlaveSelect)
N{
N    if((SPI_READ == spiCmd) ||
N       (SPI_WRITE == spiCmd))
N    {
N        CSL_SPI_REGS->SPICR2 = (CSL_SPI_REGS->SPICR2) & ((Uint16)(~(CSL_SPI_SPICR2_CMD_MASK|CSL_SPI_SPICR2_WLEN_MASK|CSL_SPI_SPICR2_CSNUM_MASK)))
X        ((CSL_SpiRegsOvly) 0x3000)->SPICR2 = (((CSL_SpiRegsOvly) 0x3000)->SPICR2) & ((Uint16)(~((0x0003u)|(0x00F8u)|(0x3000u))))
N                  | ((Uint16)(spiCmd << CSL_SPI_SPICR2_CMD_SHIFT))|( (Uint16)((wordLen - 1) << CSL_SPI_SPICR2_WLEN_SHIFT))| ((Uint16)(spiSlaveSelect << CSL_SPI_SPICR2_CSNUM_SHIFT));
X                  | ((Uint16)(spiCmd << (0x0000u)))|( (Uint16)((wordLen - 1) << (0x0003u)))| ((Uint16)(spiSlaveSelect << (0x000Cu)));
N
N        return (PSP_SOK);
X        return ((0));
N    }
N    else
N    {
N        return (PSP_E_INVAL_PARAM);
X        return ((-5));
N    }
N}
N
NPSP_Result LLC_SPI_WordLengthWriteRead(Uint32* InpBuf, Uint32* CMDBuf, Uint16 wordLength, Uint16 FremLen, Uint16 CMDwordLength, Uint16 CMDFremLen)
N{
N
N  Uint32 mask = 0;
N  Uint16 shift = 0;
N  Uint16 temp;
N  Uint32 WriteVal = 0;
N  Uint16 spiStatusReg;
N  Uint16 wordComplete;
N  Uint16 spiBusStatus; 
N  Uint8 bufIndex = 0; 
N  PSP_Result result = PSP_E_INVAL_PARAM; 
X  PSP_Result result = (-5); 
N  Uint16 sts = 0; 
N  Uint32 ReadVal = 0;
N
N  	mask = (Uint32) 1 << wordLength;
N  	mask -= 1;
N	shift = 32- wordLength;
N
N	result = (PSP_Result) LLC_SPI_FLenSet(FremLen+CMDFremLen);
N    if (result != PSP_SOK)
X    if (result != (0))
N        return (result);
N
N        for(bufIndex = 0; bufIndex < CMDFremLen; bufIndex++)
N        {
N		   
N			WriteVal = (CMDBuf[bufIndex] & mask) << shift;
N			temp = WriteVal >> 16;
N    	    CSL_SPI_REGS->SPIDR2 = temp;
X    	    ((CSL_SpiRegsOvly) 0x3000)->SPIDR2 = temp;
N			temp = WriteVal & 0xFFFF;
N		    CSL_SPI_REGS->SPIDR1 = temp;
X		    ((CSL_SpiRegsOvly) 0x3000)->SPIDR1 = temp;
N
N			
N
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
N
N
N        for(bufIndex = 0; bufIndex < FremLen; bufIndex++)
N        {
N
N             CSL_SPI_REGS->SPIDR1 = 0x00;
X             ((CSL_SpiRegsOvly) 0x3000)->SPIDR1 = 0x00;
N             CSL_SPI_REGS->SPIDR2 = 0x00;
X             ((CSL_SpiRegsOvly) 0x3000)->SPIDR2 = 0x00;
N             /* set operation */
N            result = LLC_SPI_CmdSet(SPI_READ);
N            if (result != PSP_SOK)
X            if (result != (0))
N            {
N                return (result);
N            }
N		    do {
N		        LLC_SPI_StatusRead(&spiStatusReg);
N		        spiBusStatus = (spiStatusReg & SPI_STATUS_BUSY_MASK);
X		        spiBusStatus = (spiStatusReg & (0x00000001));
N		        wordComplete = (spiStatusReg & SPI_STATUS_WC_MASK );
X		        wordComplete = (spiStatusReg & (0x00000002) );
N		    } while((SPI_STATUS_BUSY == spiBusStatus) && (wordComplete != SPI_WORD_COMPLETE));
X		    } while(((0x0001) == spiBusStatus) && (wordComplete != (0x0002)));
N
N			ReadVal = CSL_SPI_REGS->SPIDR2;
X			ReadVal = ((CSL_SpiRegsOvly) 0x3000)->SPIDR2;
N			ReadVal = ReadVal  << 16;
N			ReadVal |= CSL_SPI_REGS->SPIDR1  ;
X			ReadVal |= ((CSL_SpiRegsOvly) 0x3000)->SPIDR1  ;
N			ReadVal &= mask;
N			InpBuf[bufIndex] = ReadVal;
N        }
N
N
N		return (result);
N
N}
N
N/*EOF*/
N
N
