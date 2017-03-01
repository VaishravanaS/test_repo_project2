L 1 "../src/InterruptHandlers.c"
N/******************************************************************************
N**File Name			: InterruptHandler.c
N**File Description	:
N**Created By		:
N**Creation Date		:
N**Last modified date:
N**Remarks			:
N*******************************************************************************/
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
L 11 "../src/InterruptHandlers.c" 2
N#include "InterruptHandlers.h"
L 1 "../inc/InterruptHandlers.h" 1
N/******************************************************************************
N**File Name			: InterruptHandler.h
N**File Description	:Function Prototypes for isr funcitons
N**Created By		:
N**Creation Date		:
N**Last modified date:
N**Remarks			:
N*******************************************************************************/
N
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
L 11 "..\inc\sar.h" 2
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
L 11 "../inc/InterruptHandlers.h" 2
N#include "psp_common.h"
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
L 13 "../inc/InterruptHandlers.h" 2
N#include "I2C.h"
L 1 "..\inc\I2C.h" 1
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
N/* function to read register through i2c */
Nvoid i2c_read_reg(Uint8 cmd, Uint8* data);
NInt16 EVM5515_I2C_init( void);
NInt16 EVM5515_I2C_close( void);
NInt16 EVM5515_I2C_reset(void );
NInt16 EVM5515_I2C_write( Uint16 i2c_addr, Uint8* data, Uint16 len );
NInt16 EVM5515_I2C_read( Uint16 i2c_addr, Uint8* data, Uint16 len );
N#endif
N/*EOF*/
L 14 "../inc/InterruptHandlers.h" 2
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
L 15 "../inc/InterruptHandlers.h" 2
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
L 16 "../inc/InterruptHandlers.h" 2
N//#include "lcdFontTable.h"
N//#include "ECGDemoNonBios.h"
N
N/*	Mask used to complement the Leadoff status */
N#define LEADOFFMASK 0xFFFF
N
Ninterrupt void i2c_isr();
N
Ninterrupt void spi_isr();
N
Ninterrupt void lcd_isr();
N
Ninterrupt void uart_isr();
N
Ninterrupt void int0_isr();
N
Ninterrupt void int1_isr();
N
Ninterrupt void tim0_isr();
N
Ninterrupt void saradc_isr();
N
Ninterrupt void gpio_isr();
N
Nextern void LCD_clear_window();
Nextern void draw_string ();
N/*	function give wait cycles*/
Nextern void wait(Uint32);
N/*EOF*/
L 12 "../src/InterruptHandlers.c" 2
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
L 13 "../src/InterruptHandlers.c" 2
N#include "ADS1298.h"
L 1 "../inc/ADS1298.h" 1
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
L 14 "../src/InterruptHandlers.c" 2
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
L 15 "../src/InterruptHandlers.c" 2
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
L 1 "..\inc\evm5515.h" 1
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
L 11 "../inc/evm5515_uart.h" 2
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
L 16 "../src/InterruptHandlers.c" 2
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
L 17 "../src/InterruptHandlers.c" 2
N#include "csl_ioport.h"
N
N//#include "Communication_protocol.h"
N
N#define ARRYTHMIA_DECIMATOR 1
N#define ARRYTHMIA_BUFFER_SIZE 2500
N#define BLUE 0x03
N#define QRSSAMP 1 // to be changed to 16 later
N#define UARTSAMP 32
N#define LCDSAMP 100
N#define RESETVAL 800
N#define TWENTY_FOUR_BIT_IMPLEMENTATION 
N
N#define ECG_DECIMATOR 1
N
N#define NEW_PG_BOARD    1
N
N
N
N
N/*system status*/
Nextern Uint8 SystemState_ProcessingECGdataStatus;
Xextern unsigned char SystemState_ProcessingECGdataStatus;
N
N
N/* function to process samples for 1 sec*/	
Nextern Int16 ECG_SubSystem();	
Xextern short ECG_SubSystem();	
N
Nextern void	ECG_AntiAlising_FilterProcess(Int16 *, Int16 *,Int16 *);
Xextern void	ECG_AntiAlising_FilterProcess(short *, short *,short *);
N
NUint8 LCDDataReady = 0;
Xunsigned char LCDDataReady = 0;
NUint8 UARTDataReady = 0;
Xunsigned char UARTDataReady = 0;
NUint8 QRSDataReady = 0;
Xunsigned char QRSDataReady = 0;
NInt8 intCount = 0;
Xchar intCount = 0;
N
N
N
N//Global buffer to hold converted data
NInt16 ECGData[8] = {0};
Xshort ECGData[8] = {0};
NUint32 LeadStatus = 0;
Xunsigned long LeadStatus = 0;
N
N/*variables used in GPIO interrupt*/
N
NUint8 GPIO_interrupt_flag=(Uint8)0;
Xunsigned char GPIO_interrupt_flag=(unsigned char)0;
N
N
Nextern unsigned long AFE44xx_SPO2_Data_buf[7];
NUint8 SPO2_dataready_flag=0;
Xunsigned char SPO2_dataready_flag=0;
NUint8 I2C_dataready_flag=0;
Xunsigned char I2C_dataready_flag=0;
NUint8 UART_dataready_flag=0;
Xunsigned char UART_dataready_flag=0;
N
Ninterrupt void i2c_isr()
N{	I2C_dataready_flag=1;
N	return;	
N}
N
Ninterrupt void spi_isr()
N{
N	return;		
N}
N
Ninterrupt void lcd_isr()
N{
N	return;		
N}
N
Ninterrupt void uart_isr()
N{	extern Uint8 uart_msp_data;
X{	extern unsigned char uart_msp_data;
N	UART_dataready_flag=1;
N	EVM5515_UART_getChar(&uart_msp_data);
N
N	return;		
N}
N
Ninterrupt void int0_isr()
N{
N	SPO2_dataready_flag=1;
N	return;		
N}
N
N
Ninterrupt void gpio_isr()
N{
N	GPIO_interrupt_flag=1;
N	return;
N}
N
N
N/*-----------------------------------------------------------------------**
N** 	FunctionName: int1_isr                                               **
N** 	Description : - The INT1 ISR is used to read the 8 channel ADC data  **
N** 					and the lead off status of all the channels.         **
N**-----------------------------------------------------------------------*/
N
NUint16 count = 0;
Xunsigned short count = 0;
NUint16 Pacer_detected;
Xunsigned short Pacer_detected;
NInt16 bank_flag = 0;
Xshort bank_flag = 0;
NInt16 even_flag = 0;
Xshort even_flag = 0;
NInt32 ptr =0;
Xint ptr =0;
NUint16 ChannelNo =1;
Xunsigned short ChannelNo =1;
Nextern Int16 Sample_Data[];
Xextern short Sample_Data[];
Nextern Int16 AlisnCoeffBuf[51];
Xextern short AlisnCoeffBuf[51];
N#define ALSIN_FILTERORDER 51
NInt16 AlisnWorkingBuff[MAXCHAN][2 * ALSIN_FILTERORDER]={0};
Xshort AlisnWorkingBuff[(8)][2 * 51]={0};
N
Ninterrupt void int1_isr()
N{
Nreturn;
N
N}
N
Ninterrupt void tim0_isr()
N{
N
N//	ECG_System();
N	*GPIO_DOUT0_ADDR|=0x0004;
X	*((ioport volatile unsigned*)0x1C0A)|=0x0004;
N	int timer_counter;
W "../src/InterruptHandlers.c" 133 2 variable "timer_counter" is used before its value is set
N	timer_counter++;
N	*CPU_TIMINT_AGGR = *CPU_TIMINT_AGGR | 0x0001;
X	*((ioport volatile unsigned*)0x1c14) = *((ioport volatile unsigned*)0x1c14) | 0x0001;
N
N
N
N
N
N
N//	 int Status = 0;
N//	 Uint32 SPIBuf[32] = {0};
N//	 Uint16 wLen = 0;
N//	 Uint16 fLen = 0;
N
N
N
N
N//	/* Read data continuous mode command*/
N//	SPIBuf[0] = CMD_START;			/* SPI Tx Buffer*/
N//	wLen = 8;                        /*No. of bits per word to be transmitted*/
N//	fLen = 2;
N//
N//	/*Sending the RDATAC command to ADS1298 in order to write into registers*/
N//   Status = LLC_SPI_WordLengthWrite(SPIBuf, wLen, fLen);
N//    if(Status != PSP_SOK)
N//	 {
N//       // return (Status);
N//   }
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
N	 Uint32 SPIBuf[MAXCHAN+2] = {0};			/* SPI Rx Buffer*/
X	 unsigned long SPIBuf[(8)+2] = {0};			 
N
N	 	Uint16 wLen = 0; 				/* No. of bits per word to read*/
X	 	unsigned short wLen = 0; 				 
N	 	Uint16 fLen = 0;				/* No. words to be read*/
X	 	unsigned short fLen = 0;				 
N	 	Uint8 col = 0;
X	 	unsigned char col = 0;
N	 		static Uint16 QRSDataReady1 = 0;
X	 		static unsigned short QRSDataReady1 = 0;
N	 	Uint32 ECGDataSample[MAXCHAN];
X	 	unsigned long ECGDataSample[(8)];
N	 	short ECG_DataSample[MAXCHAN];
X	 	short ECG_DataSample[(8)];
N	 	static Int32 Pvev_Sample[MAXCHAN], Pvev_DC_Sample[MAXCHAN] ;
X	 	static int Pvev_Sample[(8)], Pvev_DC_Sample[(8)] ;
N
N	 	//	static Int16 PrevPrevPrevECGData[MAXCHAN]={0},PrevPrevECGData[MAXCHAN]={0},PrevECGData[MAXCHAN]={0};
N
N
N
N	 	#if (NEW_PG_BOARD ==1)
X	 	#if (1 ==1)
N	 	static Int16 LeadAray[8] = {2,3,8,4,5,6,7,1};		// PG 2.0
X	 	static short LeadAray[8] = {2,3,8,4,5,6,7,1};		
N	 	#else
S	 	static Int16 LeadAray[8] = {1,2,4,5,6,7,3,8};
N	 	#endif
N	 	static Uint16 bufStart=0, bufCur = ALSIN_FILTERORDER-1;
X	 	static unsigned short bufStart=0, bufCur = 51-1;
N
N	 	Int16 FiltOut;
X	 	short FiltOut;
N	 	Int32 FilterOut[2], temp1, temp2;
X	 	int FilterOut[2], temp1, temp2;
N
N	 	intCount++;
N	 	//   *GPIO_DOUT0_ADDR = *GPIO_DOUT0_ADDR | 0x0400;
N	 	*GPIO_DOUT0_ADDR = (*GPIO_DOUT0_ADDR & 0xFFEF);/*chip select to be used in evaluation board only*/
X	 	*((ioport volatile unsigned*)0x1C0A) = (*((ioport volatile unsigned*)0x1C0A) & 0xFFEF); 
N
N	 		/* Read data- continuous mode */
N	 	wLen = 24;
N	 	fLen = 9;
N	 	if (QRSDataReady1)/*to be used if down sampling (ECG_DECIMATOR) is done*/
N	 	{
N	 		QRSDataReady = 1;
N	 		QRSDataReady1 = 0;
N
N	 	}
N
N	 	/*Reading data from ADS1298 */
N	 	SystemState_ProcessingECGdataStatus=SystemState_ProcessingECGdata_ServicingInterruptReadingECGdata;
N	 	LLC_SPI_WordLengthRead(SPIBuf, wLen, fLen);
N	 	*GPIO_DOUT0_ADDR = (*GPIO_DOUT0_ADDR & 0xFFEF)|0x0010;
X	 	*((ioport volatile unsigned*)0x1C0A) = (*((ioport volatile unsigned*)0x1C0A) & 0xFFEF)|0x0010;
N
N
N	 	even_flag++;
N
N	 	for ( col =0; col < MAXCHAN; col++)
X	 	for ( col =0; col < (8); col++)
N	 	{
N	 		ECGDataSample[col] = SPIBuf[LeadAray[col]];
N	 		ECGDataSample[col] = ECGDataSample[col] >> 4;
N	 		ECG_DataSample[col] = (short) ECGDataSample[col];
N
N	 	#if RAW_DATA
S
S	 		if ( col == ChannelNo && bank_flag == 0)
S	 		{
S
S	 			Sample_Data[ptr++] = ECGDataSample[col];
S
S	 			if(ptr == 80000)
S	 			{
S	 				bank_flag = 1;
S	 				ptr =0;
S	 			}
S	 		}
N	 	#endif
N
N	 	}
N
N
N	 	/*	EVM5515_UART_putChar((ECG_DataSample[0]&0x00ff));
N	 	wait(1000);
N	 	EVM5515_UART_putChar((ECG_DataSample[0]&0xff00)>>8);
N	 	wait(1000);
N	 	*/
N
N
N
N	 	SystemState_ProcessingECGdataStatus=    SystemState_ProcessingECGdata_ServicingInterruptRemovingDC;
N
N	 	if ( intCount == ECG_DECIMATOR)/*ECG decimator not used*/
X	 	if ( intCount == 1) 
N	 	{
N	 		intCount = 0;
N	 		QRSDataReady1 = 1;
N	 		LeadStatus = SPIBuf[0];
N	 		/*IIR filter with the response      y[n]=(x[n]-x[n-1])+(0.992*y[n-1])  to remove DC component        */
N	 				for(col = 0; col < MAXCHAN; col++)
X	 				for(col = 0; col < (8); col++)
N	 		{
N	 			temp1 = NRCOEFF * Pvev_DC_Sample[col];
X	 			temp1 = (0.992) * Pvev_DC_Sample[col];
N	 			Pvev_DC_Sample[col] = (ECG_DataSample[col]  - Pvev_Sample[col]) + temp1;
N	 			Pvev_Sample[col] = ECG_DataSample[col];
N	 			temp2 = Pvev_DC_Sample[col] >> 2;
N	 			ECGData[col] = (Int16) temp2;
X	 			ECGData[col] = (short) temp2;
N	 		}
N
N	 	}
N
N
N
N	 	// *GPIO_DOUT0_ADDR = *GPIO_DOUT0_ADDR & 0xFBFF;
N
N	 	/* Arrythmia detection*/
N	 	/*extern Int16 ECG_arrythmia_buf2[2500];
N	 	extern Int16 ECG_arrythmia_buf1[2500];
N	 	static Uint16 arryt_buf_counter1=700;
N	 	static Uint16 arryt_buf_counter2=700;
N	 	static Uint8 arrythmia_samples_counter=0;
N	 	extern Uint8 buf_ready_to_process;
N	 	extern Uint8 buf1_ready_to_process;
N	 	extern Uint8 buf2_ready_to_process;
N	 	static Uint8 fill_ECG_arrythmia_buf=1;
N
N	 	arrythmia_samples_counter++;
N
N	 	if(arrythmia_samples_counter==ARRYTHMIA_DECIMATOR)
N	 	{	arrythmia_samples_counter=0;
N
N	 		if((arryt_buf_counter1<ARRYTHMIA_BUFFER_SIZE)&&(fill_ECG_arrythmia_buf==1))
N	 		{
N	 			ECG_arrythmia_buf1[arryt_buf_counter1]=ECGData[1];
N	 			arryt_buf_counter1++;
N	 		}
N	 		else if(arryt_buf_counter1==ARRYTHMIA_BUFFER_SIZE)
N	 		{
N	 			arryt_buf_counter1=0;
N	 			fill_ECG_arrythmia_buf=0;
N	 			buf1_ready_to_process=1;
N
N	 		}
N	 		if((arryt_buf_counter2<ARRYTHMIA_BUFFER_SIZE)&&(fill_ECG_arrythmia_buf==0))
N	 		{
N	 			ECG_arrythmia_buf2[arryt_buf_counter2]=ECGData[1];
N	 			arryt_buf_counter2++;
N	 		}
N	 		else if(arryt_buf_counter2==ARRYTHMIA_BUFFER_SIZE)
N	 		{
N	 			arryt_buf_counter2=0;
N	 			fill_ECG_arrythmia_buf=1;
N	 			buf2_ready_to_process=1;
N
N
N	 		}
N	 	}
N
N	 	return;*/
N
N
N
N
N	 	*GPIO_DOUT0_ADDR&=~0x0004;
X	 	*((ioport volatile unsigned*)0x1C0A)&=~0x0004;
N
N
N
N
N
N
N
Nreturn;
N
N
N
N
N
N
W "../src/InterruptHandlers.c" 190 18 variable "bufStart" was declared but never referenced
W "../src/InterruptHandlers.c" 190 30 variable "bufCur" was declared but never referenced
W "../src/InterruptHandlers.c" 192 10 variable "FiltOut" was declared but never referenced
W "../src/InterruptHandlers.c" 193 10 variable "FilterOut" was declared but never referenced
N}
N
N/*----------------------------------------------------------------------**
N** 	Function Name : saradc_isr                                          **
N** 	Description : 	-   The SARADC ISR is used to identify the switch   **
N** 						Detection whenever a switch is pressed by user. **
N** 						The ISR sets the KeyCount and Zoom Flag         **
N** 						according to the switches  selected by the user **
N** 	                                                                    **
N**----------------------------------------------------------------------*/
NUint16 LeadSelect = 1;
Xunsigned short LeadSelect = 1;
NUint16 KeyPressed = 0;
Xunsigned short KeyPressed = 0;
N   
Ninterrupt void saradc_isr()
N{
N	return;		
N}
N/*EOF*/
