################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Drivers/ST7789_Driver/Src/ST7789.c 

OBJS += \
./Drivers/ST7789_Driver/Src/ST7789.o 

C_DEPS += \
./Drivers/ST7789_Driver/Src/ST7789.d 


# Each subdirectory must supply rules for building sources it contributes
Drivers/ST7789_Driver/Src/%.o Drivers/ST7789_Driver/Src/%.su Drivers/ST7789_Driver/Src/%.cyclo: ../Drivers/ST7789_Driver/Src/%.c Drivers/ST7789_Driver/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m7 -std=gnu11 -g3 -DDEBUG -DUSE_PWR_LDO_SUPPLY -DUSE_HAL_DRIVER -DSTM32H743xx -c -I../Core/Inc -I../Drivers/SSD1306_Driver/Inc -I../Drivers/XBEE_Driver/Inc -I../Drivers/ST7789_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc -I../Drivers/STM32H7xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32H7xx/Include -I../Drivers/CMSIS/Include -I../USB_DEVICE/App -I../USB_DEVICE/Target -I../Middlewares/ST/STM32_USB_Device_Library/Core/Inc -I../Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc -I../Utilities/JPEG -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Drivers-2f-ST7789_Driver-2f-Src

clean-Drivers-2f-ST7789_Driver-2f-Src:
	-$(RM) ./Drivers/ST7789_Driver/Src/ST7789.cyclo ./Drivers/ST7789_Driver/Src/ST7789.d ./Drivers/ST7789_Driver/Src/ST7789.o ./Drivers/ST7789_Driver/Src/ST7789.su

.PHONY: clean-Drivers-2f-ST7789_Driver-2f-Src

