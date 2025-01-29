################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (12.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../ThirdParty/SystemView_Src_V354/Config/SEGGER_SYSVIEW_Config_FreeRTOS.c 

OBJS += \
./ThirdParty/SystemView_Src_V354/Config/SEGGER_SYSVIEW_Config_FreeRTOS.o 

C_DEPS += \
./ThirdParty/SystemView_Src_V354/Config/SEGGER_SYSVIEW_Config_FreeRTOS.d 


# Each subdirectory must supply rules for building sources it contributes
ThirdParty/SystemView_Src_V354/Config/%.o ThirdParty/SystemView_Src_V354/Config/%.su ThirdParty/SystemView_Src_V354/Config/%.cyclo: ../ThirdParty/SystemView_Src_V354/Config/%.c ThirdParty/SystemView_Src_V354/Config/subdir.mk
	arm-none-eabi-gcc "$<" -mcpu=cortex-m4 -std=gnu11 -g3 -DDEBUG -DUSE_HAL_DRIVER -DSTM32F407xx -c -I"C:/Users/julio/Downloads/STM32F407_FreeRTOS/App_014_Example_5.2_Block_When_Send_Queue_and_Send_Struct_Queue/ThirdParty/FreeRTOS/Source/include" -I"C:/Users/julio/Downloads/STM32F407_FreeRTOS/App_014_Example_5.2_Block_When_Send_Queue_and_Send_Struct_Queue/ThirdParty/SystemView_Src_V354/Sample" -I"C:/Users/julio/Downloads/STM32F407_FreeRTOS/App_014_Example_5.2_Block_When_Send_Queue_and_Send_Struct_Queue/ThirdParty/SystemView_Src_V354/SEGGER" -I"C:/Users/julio/Downloads/STM32F407_FreeRTOS/App_014_Example_5.2_Block_When_Send_Queue_and_Send_Struct_Queue/ThirdParty/SystemView_Src_V354/Config" -I"C:/Users/julio/Downloads/STM32F407_FreeRTOS/App_014_Example_5.2_Block_When_Send_Queue_and_Send_Struct_Queue/ThirdParty/FreeRTOS/Source/portable/GCC/ARM_CM4F" -I"C:/Users/julio/Downloads/STM32F407_FreeRTOS/App_014_Example_5.2_Block_When_Send_Queue_and_Send_Struct_Queue/ThirdParty/FreeRTOS" -I../Core/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc -I../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy -I../Drivers/CMSIS/Device/ST/STM32F4xx/Include -I../Drivers/CMSIS/Include -O0 -ffunction-sections -fdata-sections -Wall -fstack-usage -fcyclomatic-complexity -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -o "$@"

clean: clean-ThirdParty-2f-SystemView_Src_V354-2f-Config

clean-ThirdParty-2f-SystemView_Src_V354-2f-Config:
	-$(RM) ./ThirdParty/SystemView_Src_V354/Config/SEGGER_SYSVIEW_Config_FreeRTOS.cyclo ./ThirdParty/SystemView_Src_V354/Config/SEGGER_SYSVIEW_Config_FreeRTOS.d ./ThirdParty/SystemView_Src_V354/Config/SEGGER_SYSVIEW_Config_FreeRTOS.o ./ThirdParty/SystemView_Src_V354/Config/SEGGER_SYSVIEW_Config_FreeRTOS.su

.PHONY: clean-ThirdParty-2f-SystemView_Src_V354-2f-Config

