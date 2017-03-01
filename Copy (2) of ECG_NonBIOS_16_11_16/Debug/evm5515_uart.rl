L 1 "../src/evm5515_uart.c"
N /******************************************************************************
N**File Name			: evm5515_uart.c
N**File Description	: UART implementation
N**Created By		:
N**Creation Date		:
N**Last modified date:
N**Remarks			:
N*******************************************************************************/
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
L 10 "../src/evm5515_uart.c" 2
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
L 11 "../src/evm5515_uart.c" 2
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
N/* ------------------------------------------------------------------------ *
N *                                                                          *
N *  _UART_open( divisor )                                                   *
N *                                                                          *
N *      Open UART handle                                                    *
N *                                                                          *
N * ------------------------------------------------------------------------ */
NInt16 EVM5515_UART_open( void)
Xshort EVM5515_UART_open( void)
N{
N		Uint16 dummy;
X		unsigned short dummy;
N
N        SYS_EXBUSSEL &= ~0xF000;   /**/
X        *(volatile ioport unsigned short*)(0x1c00) &= ~0xF000;    
N        SYS_EXBUSSEL |=  0x1000;   /* Set parallel port to mode 1 (SPI, GPIO, UART, and I2S2)*/
X        *(volatile ioport unsigned short*)(0x1c00) |=  0x1000;    
N        SYS_PRCNTRLR &= ~0x0080;   /* Make sure UART is out of reset*/
X        *(volatile ioport unsigned short*)(0x1c05) &= ~0x0080;    
N        SYS_PCGCR1   &= ~0x0004;   /* Enable UART clock*/
X        *(volatile ioport unsigned short*)(0x1c02)   &= ~0x0004;    
N        UART_PWREMU_MGMT &= ~0xe000; /* Place the UART transmitter and receiver in reset*/
X        *(volatile ioport unsigned short*)(0x1B18) &= ~0xe000;  
N        /*  
N         * Configuring for baud rate of 115200
N         * Divisor = UART input clock frequency / (Desired baud rate*16)
N         * = 100MHz / 115200 / 16
N         * = 54
N         */
N
N        UART_DLL = 0x36;  			  /* Set baud rate*/
X        *(volatile ioport unsigned short*)(0x1B10) = 0x36;  			   
N        UART_DLH = 0x00;
X        *(volatile ioport unsigned short*)(0x1B12) = 0x00;
N        
N        UART_FCR = 0x0007;            /* Clear UART TX & RX FIFOs*/
X        *(volatile ioport unsigned short*)(0x1B04) = 0x0007;             
N        UART_FCR = 0x0000;            /* Non-FIFO mode*/
X        *(volatile ioport unsigned short*)(0x1B04) = 0x0000;             
N        UART_IER = 0x0007;            /* Enable interrupts*/
X        *(volatile ioport unsigned short*)(0x1B02) = 0x0007;             
N        UART_LCR = 0x0003;            /* 8-bit words,*/
X        *(volatile ioport unsigned short*)(0x1B06) = 0x0003;             
N                                      /* 1 STOP bit generated,*/
N                                      /* No Parity, No Stick paritiy,*/
N                                      /* No Break control*/
N        UART_MCR = 0x0000;            /* RTS & CTS disabled,*/
X        *(volatile ioport unsigned short*)(0x1B08) = 0x0000;             
N                                      /* Loopback mode disabled,*/
N                                      /* Autoflow disabled*/
N
N        UART_PWREMU_MGMT = 0xe001;    /* Enable TX & RX componenets*/
X        *(volatile ioport unsigned short*)(0x1B18) = 0xe001;     
N
N		/* Clear any pre-existing characters*/
N        dummy = UART_THR;
X        dummy = *(volatile ioport unsigned short*)(0x1B00);
N
N	    return 0;
W "../src/evm5515_uart.c" 31 10 variable "dummy" was set but never used
N}
N
N/* ------------------------------------------------------------------------ *
N *                                                                          *
N *  _UART_putChar( uart_handle, data )                                      *
N *                                                                          *
N *      Send 1 byte of serial data                                          *
N *                                                                          *
N * ------------------------------------------------------------------------ */
NInt16 EVM5515_UART_putChar( Uint8 data )
Xshort EVM5515_UART_putChar( unsigned char data )
N{
N    UART_THR = data;
X    *(volatile ioport unsigned short*)(0x1B00) = data;
N
N    return 0;
N}
N
N/* ------------------------------------------------------------------------ *
N *                                                                          *
N *  _UART_getChar( uart_handle, data )                                      *
N *                                                                          *
N *      Recv 1 byte of serial data                                          *
N *                                                                          *
N * ------------------------------------------------------------------------ */
NInt16 EVM5515_UART_getChar( Uint8* data )
Xshort EVM5515_UART_getChar( unsigned char* data )
N{
N    *data = UART_RBR;
X    *data = *(volatile ioport unsigned short*)(0x1B00);
N
N    return 0;
N}
N/*EOF*/
