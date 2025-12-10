################################################################################
# MRS Version: 2.3.0
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../User/ch32v30x_it.c \
../User/ch32v30x_usbhs_device.c \
../User/main.c \
../User/system_ch32v30x.c \
../User/usbd_compostie_km.c \
../User/usbd_desc.c 

C_DEPS += \
./User/ch32v30x_it.d \
./User/ch32v30x_usbhs_device.d \
./User/main.d \
./User/system_ch32v30x.d \
./User/usbd_compostie_km.d \
./User/usbd_desc.d 

OBJS += \
./User/ch32v30x_it.o \
./User/ch32v30x_usbhs_device.o \
./User/main.o \
./User/system_ch32v30x.o \
./User/usbd_compostie_km.o \
./User/usbd_desc.o 

DIR_OBJS += \
./User/*.o \

DIR_DEPS += \
./User/*.d \

DIR_EXPANDS += \
./User/*.234r.expand \


# Each subdirectory must supply rules for building sources it contributes
User/%.o: ../User/%.c
	@	riscv-none-embed-gcc -march=rv32imacxw -mabi=ilp32 -msmall-data-limit=8 -msave-restore -fmax-errors=20 -O0 -fmessage-length=0 -fsigned-char -ffunction-sections -fdata-sections -fno-common -Wunused -Wuninitialized -g -I"c:/01_Code_Workspace/003_phycom/finproj/keybMk1/SRC/Debug" -I"c:/01_Code_Workspace/003_phycom/finproj/keybMk1/SRC/Core" -I"c:/01_Code_Workspace/003_phycom/finproj/keybMk1/code/User" -I"c:/01_Code_Workspace/003_phycom/finproj/keybMk1/SRC/Peripheral/inc" -std=gnu99 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -c -o "$@" "$<"

