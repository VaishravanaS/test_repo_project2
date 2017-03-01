L 1 "../src/evm5515.c"
N /******************************************************************************
N**File Name			: evm5515.c
N**File Description	:
N**Created By		:
N**Creation Date		:
N**Last modified date:
N**Remarks			:
N*******************************************************************************/
N
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
L 11 "../src/evm5515.c" 2
N
N/* ------------------------------------------------------------------------ *
N *                                                                          *
N *  EVM5515_wait( delay )                                                          *
N *                                                                          *
N *      Wait in a software loop for 'x' delay                               *
N *                                                                          *
N * ------------------------------------------------------------------------ */
Nvoid EVM5515_wait( Uint32 delay )
Xvoid EVM5515_wait( unsigned long delay )
N{
N    volatile Uint32 i;
X    volatile unsigned long i;
N    for ( i = 0 ; i < delay ; i++ ){ };
N}
N
N/* ------------------------------------------------------------------------ *
N *                                                                          *
N *  _waitusec( usec )                                                       *
N *                                                                          *
N *      Wait in a software loop for 'x' microseconds                        *
N *                                                                          *
N * ------------------------------------------------------------------------ */
Nvoid EVM5515_waitusec( Uint32 usec )
Xvoid EVM5515_waitusec( unsigned long usec )
N{
N    EVM5515_wait( (Uint32)usec * 8 );
X    EVM5515_wait( (unsigned long)usec * 8 );
N}
N
N/* ------------------------------------------------------------------------ *
N *                                                                          *
N *  EVM5515_init( )                                                      *
N *                                                                          *
N *      Setup board board functions                                         *
N *                                                                          *
N * ------------------------------------------------------------------------ */
NInt16 EVM5515_init( )
Xshort EVM5515_init( )
N{
N    /* Enable clocks to all peripherals */
N    SYS_PCGCR1 = 0x0000;
X    *(volatile ioport unsigned short*)(0x1c02) = 0x0000;
N    SYS_PCGCR2 = 0x0000;
X    *(volatile ioport unsigned short*)(0x1c03) = 0x0000;
N	
N    return 0;
N}
N/*EOF*/
