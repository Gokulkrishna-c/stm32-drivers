################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (9-2020-q2-update)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/Src/stm32f410xx_gpio_driver.c \
../drivers/Src/stm32f410xx_spi_driver.c 

OBJS += \
./drivers/Src/stm32f410xx_gpio_driver.o \
./drivers/Src/stm32f410xx_spi_driver.o 

C_DEPS += \
./drivers/Src/stm32f410xx_gpio_driver.d \
./drivers/Src/stm32f410xx_spi_driver.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/Src/%.o: ../drivers/Src/%.c drivers/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DSTM32F410RBTx -DSTM32 -DNUCLEO_F410RB -DSTM32F4 -c -I../Inc -I"G:/Microcontroller and driver dev/MCU projects/MCU1/STM32_DRIVERS/drivers/Inc" -I"G:/Microcontroller and driver dev/MCU projects/MCU1/STM32_DRIVERS/drivers/Src" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfloat-abi=soft -mthumb -o "$@"

