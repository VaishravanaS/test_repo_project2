################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

CG_TOOL_ROOT := C:/ti/ccsv6/tools/compiler/c5500_4.4.1

GEN_OPTS__FLAG := 
GEN_CMDS__FLAG := 

ORDERED_OBJS += \
"../c5505.cmd" \
"./ADS1298Init.obj" \
"./ADS1298_Pacer.obj" \
"./Arrythmia.obj" \
"./Communication_protocol.obj" \
"./DiagnosticsTMS.obj" \
"./ECGSystemFunctions.obj" \
"./ECGSystemInit.obj" \
"./ECGSystem_main.obj" \
"./ECG_Filter_Co-efficients.obj" \
"./FIR_Filter.obj" \
"./Filtering.obj" \
"./I2C.obj" \
"./InterruptHandlers.obj" \
"./Respiration_Module.obj" \
"./SPO2_Init_Functions.obj" \
"./SPO2_functions.obj" \
"./Status_Config_registers.obj" \
"./cpu_clock_init.obj" \
"./dda_spi.obj" \
"./dda_uart.obj" \
"./dda_uartCfg.obj" \
"./ddc_spi.obj" \
"./ddc_uart.obj" \
"./evm5515.obj" \
"./evm5515_uart.obj" \
"./lcd.obj" \
"./llc_spi.obj" \
"./llc_uart.obj" \
"./pal_osList.obj" \
"./pal_osSem.obj" \
"./pal_osTime.obj" \
"./pal_osprotect.obj" \
"./sar.obj" \
"./util_circ.obj" \
"./vectors.obj" \
$(GEN_CMDS__FLAG) \
-l"C:\ti\ccsv6\tools\compiler\c5500_4.4.1\lib\libc.a" \

-include ../makefile.init

RM := DEL /F
RMDIR := RMDIR /S/Q

# All of the sources participating in the build are defined here
-include sources.mk
-include subdir_vars.mk
-include src/subdir_vars.mk
-include subdir_rules.mk
-include src/subdir_rules.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(S62_DEPS)),)
-include $(S62_DEPS)
endif
ifneq ($(strip $(C64_DEPS)),)
-include $(C64_DEPS)
endif
ifneq ($(strip $(ASM_DEPS)),)
-include $(ASM_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(S55_DEPS)),)
-include $(S55_DEPS)
endif
ifneq ($(strip $(C67_DEPS)),)
-include $(C67_DEPS)
endif
ifneq ($(strip $(CLA_DEPS)),)
-include $(CLA_DEPS)
endif
ifneq ($(strip $(C??_DEPS)),)
-include $(C??_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(S??_DEPS)),)
-include $(S??_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(C62_DEPS)),)
-include $(C62_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(ASM_UPPER_DEPS)),)
-include $(ASM_UPPER_DEPS)
endif
ifneq ($(strip $(K_DEPS)),)
-include $(K_DEPS)
endif
ifneq ($(strip $(C43_DEPS)),)
-include $(C43_DEPS)
endif
ifneq ($(strip $(INO_DEPS)),)
-include $(INO_DEPS)
endif
ifneq ($(strip $(S67_DEPS)),)
-include $(S67_DEPS)
endif
ifneq ($(strip $(SA_DEPS)),)
-include $(SA_DEPS)
endif
ifneq ($(strip $(S43_DEPS)),)
-include $(S43_DEPS)
endif
ifneq ($(strip $(OPT_DEPS)),)
-include $(OPT_DEPS)
endif
ifneq ($(strip $(PDE_DEPS)),)
-include $(PDE_DEPS)
endif
ifneq ($(strip $(S64_DEPS)),)
-include $(S64_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
ifneq ($(strip $(C55_DEPS)),)
-include $(C55_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXE_OUTPUTS += \
ECGSystem.out \

EXE_OUTPUTS__QUOTED += \
"ECGSystem.out" \

BIN_OUTPUTS += \
ECGSystem.hex \

BIN_OUTPUTS__QUOTED += \
"ECGSystem.hex" \


# All Target
all: ECGSystem.out

# Tool invocations
ECGSystem.out: $(OBJS) $(CMD_SRCS) $(GEN_CMDS)
	@echo 'Building target: $@'
	@echo 'Invoking: C5500 Linker'
	"C:/ti/ccsv6/tools/compiler/c5500_4.4.1/bin/cl55" -vcore:3.3 --memory_model=large -g --multibyte_chars --define="_DEBUG" --define="C55X" --define=c5515 --display_error_number --diag_warning=225 --ptrdiff_size=16 --algebraic --opt_for_speed=0 --printf_support=full --misra_required=error --misra_advisory=warning -z -m"ECGSystem.map" --stack_size=1000 --heap_size=0x400 -i"E:/C5515EVM/Copy (2) of ECG_NonBIOS" -i"E:/C5515EVM/Copy (2) of ECG_NonBIOS/Debug/src" -i"E:/C5515EVM/Copy (2) of ECG_NonBIOS/Debug" -i"E:/C5515EVM/Copy (2) of ECG_NonBIOS" -i"E:/C5515EVM/Copy (2) of ECG_NonBIOS/src" -i"E:/C5515EVM/Copy (2) of ECG_NonBIOS/inc" -i"E:/C5515EVM/Copy (2) of ECG_NonBIOS/common_inc" -i"C:/ti/bios_5_42_01_09/packages/ti/bios/include" -i"C:/ti/ccsv6/tools/compiler/c5500_4.4.1/lib" -i"/lib" --reread_libs --display_error_number --warn_sections --xml_link_info="Copy (2) of ECG_NonBIOS_linkInfo.xml" --entry_point=reset_isr --rom_model --sys_stacksize=1000 -o "ECGSystem.out" $(ORDERED_OBJS)
	@echo 'Finished building target: $@'
	@echo ' '

ECGSystem.hex: $(EXE_OUTPUTS)
	@echo 'Invoking: C5500 Hex Utility'
	"C:/ti/ccsv6/tools/compiler/c5500_4.4.1/bin/hex55"  -o "ECGSystem.hex" $(EXE_OUTPUTS__QUOTED)
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(EXE_OUTPUTS__QUOTED)$(BIN_OUTPUTS__QUOTED)
	-$(RM) "src\ADS1298Init.pp" "src\ADS1298_Pacer.pp" "src\Arrythmia.pp" "src\Communication_protocol.pp" "src\DiagnosticsTMS.pp" "src\ECGSystemFunctions.pp" "src\ECGSystemInit.pp" "src\ECGSystem_main.pp" "src\ECG_Filter_Co-efficients.pp" "src\Filtering.pp" "src\I2C.pp" "src\InterruptHandlers.pp" "src\Respiration_Module.pp" "src\SPO2_Init_Functions.pp" "src\SPO2_functions.pp" "src\Status_Config_registers.pp" "src\cpu_clock_init.pp" "src\dda_spi.pp" "src\dda_uart.pp" "src\dda_uartCfg.pp" "src\ddc_spi.pp" "src\ddc_uart.pp" "src\evm5515.pp" "src\evm5515_uart.pp" "src\lcd.pp" "src\llc_spi.pp" "src\llc_uart.pp" "src\pal_osList.pp" "src\pal_osSem.pp" "src\pal_osTime.pp" "src\pal_osprotect.pp" "src\sar.pp" "src\util_circ.pp" 
	-$(RM) "ADS1298Init.obj" "ADS1298_Pacer.obj" "Arrythmia.obj" "Communication_protocol.obj" "DiagnosticsTMS.obj" "ECGSystemFunctions.obj" "ECGSystemInit.obj" "ECGSystem_main.obj" "ECG_Filter_Co-efficients.obj" "FIR_Filter.obj" "Filtering.obj" "I2C.obj" "InterruptHandlers.obj" "Respiration_Module.obj" "SPO2_Init_Functions.obj" "SPO2_functions.obj" "Status_Config_registers.obj" "cpu_clock_init.obj" "dda_spi.obj" "dda_uart.obj" "dda_uartCfg.obj" "ddc_spi.obj" "ddc_uart.obj" "evm5515.obj" "evm5515_uart.obj" "lcd.obj" "llc_spi.obj" "llc_uart.obj" "pal_osList.obj" "pal_osSem.obj" "pal_osTime.obj" "pal_osprotect.obj" "sar.obj" "util_circ.obj" "vectors.obj" 
	-$(RM) "src\FIR_Filter.pp" "src\vectors.pp" 
	-@echo 'Finished clean'
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets

