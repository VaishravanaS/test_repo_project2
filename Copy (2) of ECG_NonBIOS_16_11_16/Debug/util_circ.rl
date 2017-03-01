L 1 "../src/util_circ.c"
N /*****************************************************************************
N * File Name : util_circ.c 
N *
N * Brief	 : Circular buffer implementation
N *
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
N#include "util_circ.h"
L 1 "../inc/util_circ.h" 1
N/******************************************************************************
N**File Name			:util_circ.h
N**File Description	:Circular buffer header
N**Created By		:
N**Creation Date		:
N**Last modified date:
N**Remarks			:
N*******************************************************************************/
N
N
N#ifndef _UTIL_CIRC_H_
N#define _UTIL_CIRC_H_
N
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
L 14 "../inc/psp_common.h" 2
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
L 15 "../inc/util_circ.h" 2
N
Nextern Uint16 offset ;
N
N#define UTIL_circFullCount(hCirc)       ((hCirc)->charCount)
N#define UTIL_circEmptyCount(hCirc)      ((hCirc)->size - (hCirc)->charCount)
N/* NOTE: The circular buffers are expected to be a power of 2. If not, then 
N * the following macros will not work
N */
N#define UTIL_circNextIndex(hCirc,index) (((index) + offset) & ((hCirc)->size - offset))
N#define UTIL_circPrevIndex(hCirc,index) (((index) - offset) & ((hCirc)->size - offset))
N#define UTIL_circIsFull(hCirc)          ((hCirc)->charCount == (hCirc)->size)
N
Ntypedef struct
N{
N    Uint16  writeIndex; /* write pointer for the buffer                     */
N    Uint16  readIndex;  /* read pointer fro the buffer                      */
N    Uint16  charCount;  /* buffer character count                           */
N    Uint16  size;       /* buffer size                                      */
N    Char    *buf;       /* circular buffer                                  */
N} UTIL_CircObj, *UTIL_CircHandle;
N
N
N
Nextern void UTIL_circNew(UTIL_CircHandle hCirc, Ptr hBuf, Uint16 size);
Nextern Char UTIL_circReadChar(UTIL_CircHandle hCirc);
Nextern void UTIL_circWriteChar(UTIL_CircHandle hCirc, Char c);
Nextern Int UTIL_circReadMultiple(UTIL_CircHandle hCirc, Ptr hBuf, Int num);
Nextern Int UTIL_circWriteMultiple(UTIL_CircHandle hCirc, const Char *buf, Int num);
Nextern void UTIL_circUnget(UTIL_CircHandle hCirc);
N
N
N
N#endif /* _UTIL_CIRC_H_ */
N/*EOF*/
L 41 "../src/util_circ.c" 2
N
N
NUint16 offset = 1u;
N
N/*
N *  ======== UTIL_circNew ========
N *
N *  Initializes the circular buffer structure
N */
Nvoid UTIL_circNew(UTIL_CircHandle hCirc, Ptr hBuf, Uint16 size)
N{
N    Char * buf = (Char*)hBuf;
N    hCirc->writeIndex = 0;
N    hCirc->readIndex = 0;
N    hCirc->charCount = 0;
N    hCirc->buf = buf;
N    hCirc->size = size;
N}
N
N/*
N *  ======== UTIL_circReadChar ========
N *
N *  Reads a character from the circular buffer.
N */
NChar UTIL_circReadChar(UTIL_CircHandle hCirc)
N{
N    Char c;
N
N    /* read character and increment the character count */
N    c = hCirc->buf[hCirc->readIndex];
N    hCirc->readIndex = (Uint16)UTIL_circNextIndex(hCirc, hCirc->readIndex);
X    hCirc->readIndex = (Uint16)(((hCirc->readIndex) + offset) & ((hCirc)->size - offset));
N    hCirc->charCount--;
N
N    return (c);
N}
N
N/*
N *  ======== UTIL_circWriteChar ========
N *
N *  Writes a character into the circular buffer
N */
Nvoid UTIL_circWriteChar(UTIL_CircHandle hCirc, Char c)
N{
N    /* write character and decrement the character count */
N    hCirc->buf[hCirc->writeIndex] = c;
N    hCirc->writeIndex = (Uint16)UTIL_circNextIndex(hCirc, hCirc->writeIndex);
X    hCirc->writeIndex = (Uint16)(((hCirc->writeIndex) + offset) & ((hCirc)->size - offset));
N    hCirc->charCount++;
N}
N
N/*
N *  ======== UTIL_circReadMultiple ========
N *
N *  Reads multiple chars from the buffer
N */
NInt UTIL_circReadMultiple(UTIL_CircHandle hCirc, Ptr hBuf, Int num)
N{
N    Int     i;
N    Int     available;
N	Char    *buf;
N
N	buf = (Char*)hBuf;
N
N    available = (Int)UTIL_circFullCount(hCirc);
X    available = (Int)((hCirc)->charCount);
N    num = (num < available) ? num : available;
N
N    for (i = 0; i < num; i++) {
N        *buf++ = hCirc->buf[hCirc->readIndex];
N        hCirc->readIndex = (Uint16)UTIL_circNextIndex(hCirc, hCirc->readIndex);
X        hCirc->readIndex = (Uint16)(((hCirc->readIndex) + offset) & ((hCirc)->size - offset));
N        hCirc->charCount--;
N    }
N
N    return (num);
N}
N
N/*
N *  ======== UTIL_circWriteMultiple ========
N *
N *  Writes multiple characters into the circular buffer
N */
NInt UTIL_circWriteMultiple(UTIL_CircHandle hCirc, const Char *buf, Int num)
N{
N    Int     i;
N    Int     available;
N
N    available = (Int)UTIL_circEmptyCount(hCirc);
X    available = (Int)((hCirc)->size - (hCirc)->charCount);
N    num = (num < available) ? num : available;
N
N    for (i = 0; i < num; i++) {
N        hCirc->buf[hCirc->writeIndex] = (*buf);
N		buf++;
N        hCirc->writeIndex = (Uint16)UTIL_circNextIndex(hCirc, hCirc->writeIndex);
X        hCirc->writeIndex = (Uint16)(((hCirc->writeIndex) + offset) & ((hCirc)->size - offset));
N        hCirc->charCount++;
N    }
N
N    return (num);
N}
N
N/*
N *  ======== UTIL_circUnget ========
N *
N *  Move the read-index back 1 space to unget the character read
N */
Nvoid UTIL_circUnget(UTIL_CircHandle hCirc)
N{
N    hCirc->readIndex = (Uint16)UTIL_circPrevIndex(hCirc, hCirc->readIndex);
X    hCirc->readIndex = (Uint16)(((hCirc->readIndex) - offset) & ((hCirc)->size - offset));
N}
N
