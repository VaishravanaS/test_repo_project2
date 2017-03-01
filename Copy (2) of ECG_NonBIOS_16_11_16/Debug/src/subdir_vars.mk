################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
ASM_SRCS += \
../src/FIR_Filter.asm \
../src/vectors.asm 

C_SRCS += \
../src/ADS1298Init.c \
../src/ADS1298_Pacer.c \
../src/Arrythmia.c \
../src/Communication_protocol.c \
../src/DiagnosticsTMS.c \
../src/ECGSystemFunctions.c \
../src/ECGSystemInit.c \
../src/ECGSystem_main.c \
../src/ECG_Filter_Co-efficients.c \
../src/Filtering.c \
../src/I2C.c \
../src/InterruptHandlers.c \
../src/Respiration_Module.c \
../src/SPO2_Init_Functions.c \
../src/SPO2_functions.c \
../src/Status_Config_registers.c \
../src/cpu_clock_init.c \
../src/dda_spi.c \
../src/dda_uart.c \
../src/dda_uartCfg.c \
../src/ddc_spi.c \
../src/ddc_uart.c \
../src/evm5515.c \
../src/evm5515_uart.c \
../src/lcd.c \
../src/llc_spi.c \
../src/llc_uart.c \
../src/pal_osList.c \
../src/pal_osSem.c \
../src/pal_osTime.c \
../src/pal_osprotect.c \
../src/sar.c \
../src/util_circ.c 

OBJS += \
./ADS1298Init.obj \
./ADS1298_Pacer.obj \
./Arrythmia.obj \
./Communication_protocol.obj \
./DiagnosticsTMS.obj \
./ECGSystemFunctions.obj \
./ECGSystemInit.obj \
./ECGSystem_main.obj \
./ECG_Filter_Co-efficients.obj \
./FIR_Filter.obj \
./Filtering.obj \
./I2C.obj \
./InterruptHandlers.obj \
./Respiration_Module.obj \
./SPO2_Init_Functions.obj \
./SPO2_functions.obj \
./Status_Config_registers.obj \
./cpu_clock_init.obj \
./dda_spi.obj \
./dda_uart.obj \
./dda_uartCfg.obj \
./ddc_spi.obj \
./ddc_uart.obj \
./evm5515.obj \
./evm5515_uart.obj \
./lcd.obj \
./llc_spi.obj \
./llc_uart.obj \
./pal_osList.obj \
./pal_osSem.obj \
./pal_osTime.obj \
./pal_osprotect.obj \
./sar.obj \
./util_circ.obj \
./vectors.obj 

ASM_DEPS += \
./src/FIR_Filter.pp \
./src/vectors.pp 

C_DEPS += \
./src/ADS1298Init.pp \
./src/ADS1298_Pacer.pp \
./src/Arrythmia.pp \
./src/Communication_protocol.pp \
./src/DiagnosticsTMS.pp \
./src/ECGSystemFunctions.pp \
./src/ECGSystemInit.pp \
./src/ECGSystem_main.pp \
./src/ECG_Filter_Co-efficients.pp \
./src/Filtering.pp \
./src/I2C.pp \
./src/InterruptHandlers.pp \
./src/Respiration_Module.pp \
./src/SPO2_Init_Functions.pp \
./src/SPO2_functions.pp \
./src/Status_Config_registers.pp \
./src/cpu_clock_init.pp \
./src/dda_spi.pp \
./src/dda_uart.pp \
./src/dda_uartCfg.pp \
./src/ddc_spi.pp \
./src/ddc_uart.pp \
./src/evm5515.pp \
./src/evm5515_uart.pp \
./src/lcd.pp \
./src/llc_spi.pp \
./src/llc_uart.pp \
./src/pal_osList.pp \
./src/pal_osSem.pp \
./src/pal_osTime.pp \
./src/pal_osprotect.pp \
./src/sar.pp \
./src/util_circ.pp 

C_DEPS__QUOTED += \
"src\ADS1298Init.pp" \
"src\ADS1298_Pacer.pp" \
"src\Arrythmia.pp" \
"src\Communication_protocol.pp" \
"src\DiagnosticsTMS.pp" \
"src\ECGSystemFunctions.pp" \
"src\ECGSystemInit.pp" \
"src\ECGSystem_main.pp" \
"src\ECG_Filter_Co-efficients.pp" \
"src\Filtering.pp" \
"src\I2C.pp" \
"src\InterruptHandlers.pp" \
"src\Respiration_Module.pp" \
"src\SPO2_Init_Functions.pp" \
"src\SPO2_functions.pp" \
"src\Status_Config_registers.pp" \
"src\cpu_clock_init.pp" \
"src\dda_spi.pp" \
"src\dda_uart.pp" \
"src\dda_uartCfg.pp" \
"src\ddc_spi.pp" \
"src\ddc_uart.pp" \
"src\evm5515.pp" \
"src\evm5515_uart.pp" \
"src\lcd.pp" \
"src\llc_spi.pp" \
"src\llc_uart.pp" \
"src\pal_osList.pp" \
"src\pal_osSem.pp" \
"src\pal_osTime.pp" \
"src\pal_osprotect.pp" \
"src\sar.pp" \
"src\util_circ.pp" 

OBJS__QUOTED += \
"ADS1298Init.obj" \
"ADS1298_Pacer.obj" \
"Arrythmia.obj" \
"Communication_protocol.obj" \
"DiagnosticsTMS.obj" \
"ECGSystemFunctions.obj" \
"ECGSystemInit.obj" \
"ECGSystem_main.obj" \
"ECG_Filter_Co-efficients.obj" \
"FIR_Filter.obj" \
"Filtering.obj" \
"I2C.obj" \
"InterruptHandlers.obj" \
"Respiration_Module.obj" \
"SPO2_Init_Functions.obj" \
"SPO2_functions.obj" \
"Status_Config_registers.obj" \
"cpu_clock_init.obj" \
"dda_spi.obj" \
"dda_uart.obj" \
"dda_uartCfg.obj" \
"ddc_spi.obj" \
"ddc_uart.obj" \
"evm5515.obj" \
"evm5515_uart.obj" \
"lcd.obj" \
"llc_spi.obj" \
"llc_uart.obj" \
"pal_osList.obj" \
"pal_osSem.obj" \
"pal_osTime.obj" \
"pal_osprotect.obj" \
"sar.obj" \
"util_circ.obj" \
"vectors.obj" 

ASM_DEPS__QUOTED += \
"src\FIR_Filter.pp" \
"src\vectors.pp" 

C_SRCS__QUOTED += \
"../src/ADS1298Init.c" \
"../src/ADS1298_Pacer.c" \
"../src/Arrythmia.c" \
"../src/Communication_protocol.c" \
"../src/DiagnosticsTMS.c" \
"../src/ECGSystemFunctions.c" \
"../src/ECGSystemInit.c" \
"../src/ECGSystem_main.c" \
"../src/ECG_Filter_Co-efficients.c" \
"../src/Filtering.c" \
"../src/I2C.c" \
"../src/InterruptHandlers.c" \
"../src/Respiration_Module.c" \
"../src/SPO2_Init_Functions.c" \
"../src/SPO2_functions.c" \
"../src/Status_Config_registers.c" \
"../src/cpu_clock_init.c" \
"../src/dda_spi.c" \
"../src/dda_uart.c" \
"../src/dda_uartCfg.c" \
"../src/ddc_spi.c" \
"../src/ddc_uart.c" \
"../src/evm5515.c" \
"../src/evm5515_uart.c" \
"../src/lcd.c" \
"../src/llc_spi.c" \
"../src/llc_uart.c" \
"../src/pal_osList.c" \
"../src/pal_osSem.c" \
"../src/pal_osTime.c" \
"../src/pal_osprotect.c" \
"../src/sar.c" \
"../src/util_circ.c" 

ASM_SRCS__QUOTED += \
"../src/FIR_Filter.asm" \
"../src/vectors.asm" 


