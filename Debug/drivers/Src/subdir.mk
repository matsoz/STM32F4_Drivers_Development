################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../drivers/Src/stm32f407xx_gpio_driver.c 

OBJS += \
./drivers/Src/stm32f407xx_gpio_driver.o 

C_DEPS += \
./drivers/Src/stm32f407xx_gpio_driver.d 


# Each subdirectory must supply rules for building sources it contributes
drivers/Src/stm32f407xx_gpio_driver.o: ../drivers/Src/stm32f407xx_gpio_driver.c
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DSTM32 -DSTM32F4 -DDEBUG -DSTM32F407ZGTx -c -I../Inc -I"C:/Ac6/STM32Cube/STM32F4_Drivers_Development/drivers/Inc" -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -MMD -MP -MF"drivers/Src/stm32f407xx_gpio_driver.d" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

