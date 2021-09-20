################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/003_SPITesting.c \
../Src/syscalls.c \
../Src/sysmem.c 

OBJS += \
./Src/003_SPITesting.o \
./Src/syscalls.o \
./Src/sysmem.o 

C_DEPS += \
./Src/003_SPITesting.d \
./Src/syscalls.d \
./Src/sysmem.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32F410RBTx -DSTM32 -DNUCLEO_F410RB -DSTM32F4 -c -I../Inc -I"G:/Microcontroller and driver dev/MCU projects/MCU1/STM32_DRIVERS/drivers/Inc" -I"G:/Microcontroller and driver dev/MCU projects/MCU1/STM32_DRIVERS/drivers/Src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

