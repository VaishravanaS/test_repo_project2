L 1 "../src/sar.c"
N/******************************************************************************
N**File Name			: SAR.c
N**File Description	:
N**Created By		:
N**Creation Date		:
N**Last modified date:
N**Remarks			:
N*******************************************************************************/
N
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
L 11 "../src/sar.c" 2
N#include <std.h>
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
L 12 "../src/sar.c" 2
N#include "stdtypes.h"
L 1 "../common_inc/stdtypes.h" 1
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
L 1 "..\common_inc\tistdtypes.h" 1
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
L 46 "../common_inc/stdtypes.h" 2
N
N#endif /* _STDTYPES_H_ */
N
L 13 "../src/sar.c" 2
N#include "sar.h"  
L 1 "../inc/sar.h" 1
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
L 11 "../inc/sar.h" 2
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
L 14 "../src/sar.c" 2
N
N/*-----------------------------------------------------------------**
N** 	Function  Name: Init_SAR                                       **
N** 	Description : 	- 	The function initializes the SAR module to **
N** 						select GPAIN1(AIN3) Also set the saradc    **
N** 						interrupt for all key press                **
N** 	Parameters	:	- 	None                                       **
N**                                                                 **
N**-----------------------------------------------------------------*/
N
Nvoid Init_SAR(void)
N{
N   	
N	
N  	*SARCTRL = 0x0000;		/* reset the CTRL Reg */
X  	*((ioport volatile unsigned*)0x7012) = 0x0000;		 
N    *SARCTRL = 0x3400; 		/* select AIN3, which is GPAIN1 */
X    *((ioport volatile unsigned*)0x7012) = 0x3400; 		 
N	*SARCLKCTRL = 0x0064;   /* 100/100 = 1MHz */ 
X	*((ioport volatile unsigned*)0x7016) = 0x0064;     
N    *SARPINCTRL = 0x7104;   /* Set the AD reference & PIN control values */
X    *((ioport volatile unsigned*)0x7018) = 0x7104;    
N    *SARGPOCTRL = 0x0000;   /* Set the PEN interrupt */
X    *((ioport volatile unsigned*)0x701A) = 0x0000;    
N 
N
N   
N}
N
N/*---------------------------------------------------------------------**
N** 	Function Name : Get_Sar_Key                                        **
N** 	Description : 	- 	This function reads the value corresponding to **
N** 						the selected switch Also set the saradc        **
N** 						interrupt for all key press.                   **
N**                                                                     **
N** 	Parameters	:	- None                                             **
N**                                                                     **
N**---------------------------------------------------------------------*/
N
NUint16 Get_Sar_Key(void)
N{
N    Uint16 val;
N
N   	val = *SARDATA;
X   	val = *((ioport volatile unsigned*)0x7014);
N	*SARCTRL = 0xB400;   
X	*((ioport volatile unsigned*)0x7012) = 0xB400;   
N
N    return(val & 0x3ff);
N}
N/*EOF*/
