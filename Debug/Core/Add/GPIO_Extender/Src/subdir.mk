################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Add/GPIO_Extender/Src/PCF_8574.c 

OBJS += \
./Core/Add/GPIO_Extender/Src/PCF_8574.o 

C_DEPS += \
./Core/Add/GPIO_Extender/Src/PCF_8574.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Add/GPIO_Extender/Src/%.o Core/Add/GPIO_Extender/Src/%.su Core/Add/GPIO_Extender/Src/%.cyclo: ../Core/Add/GPIO_Extender/Src/%.c Core/Add/GPIO_Extender/Src/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32WB55xx -c -I../Core/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc -I../Drivers/STM32WBxx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32WBxx/Include -I../Drivers/CMSIS/Include -I../STM32_WPAN/App -I../Utilities/lpm/tiny_lpm -I../Middlewares/ST/STM32_WPAN -I../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread -I../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/tl -I../Middlewares/ST/STM32_WPAN/interface/patterns/ble_thread/shci -I../Middlewares/ST/STM32_WPAN/utilities -I../Middlewares/ST/STM32_WPAN/ble/core -I../Middlewares/ST/STM32_WPAN/ble/core/auto -I../Middlewares/ST/STM32_WPAN/ble/core/template -I../Middlewares/ST/STM32_WPAN/ble/svc/Inc -I../Middlewares/ST/STM32_WPAN/ble/svc/Src -I../Utilities/sequencer -I../Middlewares/ST/STM32_WPAN/ble -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-Core-2f-Add-2f-GPIO_Extender-2f-Src

clean-Core-2f-Add-2f-GPIO_Extender-2f-Src:
	-$(RM) ./Core/Add/GPIO_Extender/Src/PCF_8574.cyclo ./Core/Add/GPIO_Extender/Src/PCF_8574.d ./Core/Add/GPIO_Extender/Src/PCF_8574.o ./Core/Add/GPIO_Extender/Src/PCF_8574.su

.PHONY: clean-Core-2f-Add-2f-GPIO_Extender-2f-Src

