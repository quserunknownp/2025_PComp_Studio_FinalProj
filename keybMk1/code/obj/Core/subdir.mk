################################################################################
# MRS Version: 2.3.0
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
c:/01_Code_Workspace/003_phycom/finproj/keybMk1/SRC/Core/core_riscv.c 

C_DEPS += \
./Core/core_riscv.d 

OBJS += \
./Core/core_riscv.o 

DIR_OBJS += \
./Core/*.o \

DIR_DEPS += \
./Core/*.d \

DIR_EXPANDS += \
./Core/*.234r.expand \


# Each subdirectory must supply rules for building sources it contributes
Core/core_riscv.o: c:/01_Code_Workspace/003_phycom/finproj/keybMk1/SRC/Core/core_riscv.c
	@	riscv-none-embed-gcc -march=rv32imacxw -mabi=ilp32 -msmall-data-limit=8 -msave-restore -fmax-errors=20 -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized -g -I"c:/01_Code_Workspace/003_phycom/finproj/keybMk1/SRC/Debug" -I"c:/01_Code_Workspace/003_phycom/finproj/keybMk1/SRC/Core" -I"c:/01_Code_Workspace/003_phycom/finproj/keybMk1/code/User" -I"c:/01_Code_Workspace/003_phycom/finproj/keybMk1/SRC/Peripheral/inc" -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

