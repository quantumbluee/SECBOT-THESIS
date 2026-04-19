################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../MBEDTLS/App/mbedtls.c 

OBJS += \
./MBEDTLS/App/mbedtls.o 

C_DEPS += \
./MBEDTLS/App/mbedtls.d 


# Each subdirectory must supply rules for building sources it contributes
MBEDTLS/App/%.o MBEDTLS/App/%.su MBEDTLS/App/%.cyclo: ../MBEDTLS/App/%.c MBEDTLS/App/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F723xx '-DMBEDTLS_CONFIG_FILE=<mbedtls_config.h>' -c -I../Core/Inc -I../USB_HOST/App -I../USB_HOST/Target -I../Drivers/STM32F7xx_HAL_Driver/Inc -I../Drivers/STM32F7xx_HAL_Driver/Inc/Legacy -I../Middlewares/ST/STM32_USB_Host_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Host_Library/Class/AUDIO/Inc -I../Middlewares/ST/STM32_USB_Host_Library/Class/MSC/Inc -I../Drivers/CMSIS/Device/ST/STM32F7xx/Include -I../Drivers/CMSIS/Include -I../MBEDTLS/App -I../Middlewares/Third_Party/mbedTLS/include/mbedtls -I../Middlewares/Third_Party/mbedTLS/include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-MBEDTLS-2f-App

clean-MBEDTLS-2f-App:
	-$(RM) ./MBEDTLS/App/mbedtls.cyclo ./MBEDTLS/App/mbedtls.d ./MBEDTLS/App/mbedtls.o ./MBEDTLS/App/mbedtls.su

.PHONY: clean-MBEDTLS-2f-App

